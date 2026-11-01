#!/usr/bin/env python3
"""
Gazebo初期状態復元ノード (ROS2 パッケージ版)

初期状態（シミュレーション開始時に読み取った値）を直接書き込んで復元する。
  - アーム crane_plus : 各関節が少し曲がった姿勢（すべて正の値）
      -> ros2_control の joint_trajectory トピックへ直接発行（MoveIt計画なし）
  - キューブ wood_cube_5cm : 位置 (0.0, -0.1, 1.015), 姿勢 (0,0,0,1)
      -> Gazebo サービス /world/default/set_pose を直接呼び出し

使い方:
    ros2 run crane_plus_automator reset_state            # 復元 + 検証
    ros2 run crane_plus_automator reset_state --verify   # 検証のみ
"""

import argparse
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------------------------------------------------------------------
# 復元姿勢（関節リミット内で、アームが visibly 曲がるよう各関節に正の値を与える）
# リミット: joint1 ±2.618 / joint2 -1.825~1.784 / joint3 -2.557~2.454 /
#           joint4 -1.840~1.768 / joint_hand -0.709(開)~0.668(閉)
# ---------------------------------------------------------------------------
INITIAL_JOINT_POSITIONS = {
    "crane_plus_joint1": 0.0,      # 基部は正面
    "crane_plus_joint2": 0.01,     # ほぼ伸ばした状態
    "crane_plus_joint3": -0.01,
    "crane_plus_joint4": -0.01,
    "crane_plus_joint_hand": 0.0,  # グリッパは中間
}
ARM_JOINTS = [
    "crane_plus_joint1",
    "crane_plus_joint2",
    "crane_plus_joint3",
    "crane_plus_joint4",
]
GRIPPER_JOINTS = ["crane_plus_joint_hand"]
GRIPPER_OPEN_POSITION = -0.709   # joint_hand の開側リミット（全開）

INITIAL_CUBE_NAME = "wood_cube_5cm"
INITIAL_CUBE_POSE = {  # position / quaternion(w)
    "x": 0.0, "y": -0.1, "z": 1.015,
    "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
}

JOINT_TOLERANCE = 1e-2   # [rad]
POSE_TOLERANCE = 2e-2    # [m]


# ---------------------------------------------------------------------------
# キューブ: ign サービス /world/default/set_pose を直接呼び出す
# ---------------------------------------------------------------------------
def reset_cube() -> bool:
    p = INITIAL_CUBE_POSE
    req = (
        f'name: "{INITIAL_CUBE_NAME}" '
        f"position {{ x: {p['x']} y: {p['y']} z: {p['z']} }} "
        f"orientation {{ x: {p['qx']} y: {p['qy']} z: {p['qz']} w: {p['qw']} }}"
    )
    cmd = [
        "ign", "service", "-s", "/world/default/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000",
        "-r", req,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    ok = "data: true" in result.stdout
    print(f"[cube ] set_pose -> {'OK' if ok else 'FAILED'}")
    if not ok:
        print(result.stdout + result.stderr)
    return ok


def read_cube_position():
    """Gazebo /world/default/pose/info からキューブの現在位置を読む。"""
    cmd = ["ign", "topic", "-e", "-t", "/world/default/pose/info", "-n", "1"]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=20).stdout
    m = re.search(
        re.escape(f'name: "{INITIAL_CUBE_NAME}"') + r"\s*id:\s*\d+\s*"
        r"position\s*\{\s*x:\s*([-\d.e+]+)\s*y:\s*([-\d.e+]+)\s*z:\s*([-\d.e+]+)",
        out,
    )
    if not m:
        return None
    return tuple(float(v) for v in m.groups())


# ---------------------------------------------------------------------------
# アーム: ros2_control のジョイントトラジェクトリへ目標位置を直接書き込む
# ---------------------------------------------------------------------------
class ArmResetter(Node):
    def __init__(self):
        super().__init__("reset_state_node")
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", 10
        )
        self.joint_state = {}
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_state[name] = pos

    def publish_home(self, joints, publisher, targets, duration_sec=3.0):
        traj = JointTrajectory()
        traj.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [targets[j] for j in joints]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = 0
        traj.points = [point]
        publisher.publish(traj)

    def wait_converged(self, targets, timeout_sec=15.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(
                j in self.joint_state
                and abs(self.joint_state[j] - targets[j]) < JOINT_TOLERANCE
                for j in targets
            ):
                return True
        return False


def reset_arm(node: ArmResetter) -> bool:
    # コントローラの購読待ち
    for _ in range(50):
        if node.arm_pub.get_subscription_count() and node.gripper_pub.get_subscription_count():
            break
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.05)

    # 1) アームを動かす前にグリッパを開く
    gripper_open = {j: GRIPPER_OPEN_POSITION for j in GRIPPER_JOINTS}
    node.publish_home(GRIPPER_JOINTS, node.gripper_pub, gripper_open)
    print(f"[arm  ] opening gripper first: "
          f"{GRIPPER_JOINTS[0]} -> {GRIPPER_OPEN_POSITION:+.3f} rad")
    if node.wait_converged(gripper_open, timeout_sec=10.0):
        print("[arm  ] gripper open OK")
    else:
        print("[arm  ] WARN: gripper not fully open, continuing")

    # 2) グリッパを開いた状態でアームをホーム姿勢へ（グリッパは中間位置へ戻す）
    node.publish_home(ARM_JOINTS, node.arm_pub, INITIAL_JOINT_POSITIONS)
    node.publish_home(GRIPPER_JOINTS, node.gripper_pub, INITIAL_JOINT_POSITIONS)
    print(f"[arm  ] joint targets {INITIAL_JOINT_POSITIONS} -> controllers")

    ok = node.wait_converged(INITIAL_JOINT_POSITIONS)
    print(f"[arm  ] {'OK: all joints at target' if ok else 'WARN: not converged'}")
    return ok


# ---------------------------------------------------------------------------
# 検証
# ---------------------------------------------------------------------------
def verify(node: ArmResetter) -> bool:
    print("--- 検証 ---")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = True
    for j, target in INITIAL_JOINT_POSITIONS.items():
        pos = node.joint_state.get(j)
        good = pos is not None and abs(pos - target) < JOINT_TOLERANCE
        ok &= good
        print(f"  {j:24s}: {pos:+.6f} rad  {'OK' if good else 'NG'}")

    cube = read_cube_position()
    if cube is None:
        print(f"  {INITIAL_CUBE_NAME}: pose read FAILED")
        ok = False
    else:
        p = INITIAL_CUBE_POSE
        err = max(abs(a - b) for a, b in
                  zip(cube, (p["x"], p["y"], p["z"])))
        good = err < POSE_TOLERANCE
        ok &= good
        print(f"  {INITIAL_CUBE_NAME}: pos=({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f})  "
              f"{'OK' if good else 'NG'}")
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(description="Restore initial Gazebo state.")
    parser.add_argument("--verify", action="store_true",
                        help="復元せず現在状態の検証のみ行う")
    args = parser.parse_args(argv)

    rclpy.init()
    node = ArmResetter()
    try:
        if not args.verify:
            arm_ok = reset_arm(node)
            cube_ok = reset_cube()
            if not (cube_ok and arm_ok):
                print("復元に問題がありました。")
        ok = verify(node)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
