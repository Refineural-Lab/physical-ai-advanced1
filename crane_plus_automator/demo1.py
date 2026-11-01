#!/usr/bin/env python3
"""キューブの位置を Gazebo から取得してグリッパで掴むノード (ROS2 パッケージ版).

手順:
    1. Gazebo の /world/default/pose/info トピック (SceneBroadcaster) から
       キューブ (既定: wood_cube_5cm) とロボット (crane_plus) の現在姿勢を取得し、
       ロボット基底座標系 (crane_plus_base) 上のキューブ位置を算出する
    2. MoveIt の /compute_ik サービスで逆運動学を解き、腕はコントローラの
       joint_trajectory トピックへ直接発行して実行する
       home -> キューブ真上 (pre-grasp) -> 下降 -> グリッパ閉
       -> 把持したまま home (最初の姿勢) へ復帰 (--no-ret 指定時は省略)
       (把持後の上昇 move_pose(lift_z) は行わない。成否は home 復帰後に確認する)
    3. home 復帰後にキューブが桌面から離れたかを確認して成否を報告する
       (多少滑り落ちても桌面から浮いていれば成功とみなす。)

使い方:
    source /opt/ros/humble/setup.bash
    ros2 run crane_plus_automator demo1 [オプション]

主なオプション:
    --model NAME     掴むモデル名 (既定: wood_cube_5cm)
    --grasp-z Z      把持時の endtip の高さ [m, base座標系 z] (既定: 0.140)
    --pre-z Z        真上待機時の endtip の高さ [m] (既定: 0.17)
    --jitter         endtip 目標 xy に ±5mm のランダムジッターを加える
                     (既定: 無効。掴み位置を毎回少しずらして把持の
                      バラつきを試したいときに使う)
    --no-ret         把持後に home へ復帰しない。home 復帰後の成否確認も
                     省略し、キューブの状態を調べず成功 (終了コード 0) を返す
                     (既定: 掴んだまま home へ復帰し、復帰後に成否確認する)
    --dry-run        位置取得と到達性判定まで行い、動作はしない

記録について:
    - 腕・グリッパとも MoveIt action (/move_action, /execute_trajectory) 経由では
      なく、teleop_keyboard と同じくコントローラの joint_trajectory トピックへ
      JointTrajectory を **常時 20Hz** で直接発行し続ける (teleop と同一形式)。
      (アクション内部トピックは hidden のため `ros2 bag record -a` で記録されない。
       トピック経由にすれば teleop と同一形式で腕の指令が bag に残る)
      - 移動していない区間も現在の目標角度が 20Hz で送り続けられる。
      - 起動時は /joint_states の現在角度で目標を初期化してから送信するため,
        指令開始の瞬間に腕が飛ぶことはない。
    - IK には MoveIt の /compute_ik サービスを引き続き使用する。

注意:
    - crane_plus_with_table.launch.py では endtip_offset が 0 のため
      endtip = link4 原点。グリッパは endtip より GRIP_Z_OFFSET (45mm) 下にある
      ため、把持高さにはその分を反映している (既定 grasp-z 0.140)。
    - 把持可能範囲は endtip 高さ 0.08m 付近で 基底から 約0.08〜0.17m。
      キューブが近すぎ/遠すぎるとエラー終了します (GUI で位置を動かすか
      --allow-unreachable で抑止)。
"""
import argparse
import math
import os
import random
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

BASE_FRAME = 'crane_plus_base'
ENDTIP_FRAME = 'crane_plus_link_endtip'
ARM_JOINT_NAMES = [
    'crane_plus_joint1', 'crane_plus_joint2',
    'crane_plus_joint3', 'crane_plus_joint4',
]
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'
GRIPPER_OPEN = -math.radians(40.62) + 0.001
GRIPPER_CLOSE = math.radians(38.27) - 0.001
# 最初の姿勢 (home): 直立 (全関節 0) 姿勢.
# teleop_keyboard の X キー (ARM_INIT = [0,0,0,0]) / reset_arm.py と同一。
HOME_POSE = [0.0, 0.0, 0.0, 0.0]

# 腕・グリッパの直接制御 (トピック経由) 設定.
# MoveIt action は ros2 bag record -a で記録されないため、
# teleop_keyboard と同じくコントローラの joint_trajectory トピックへ
# 常時 20Hz で直接発行して動かす.
ARM_TOPIC = '/crane_plus_arm_controller/joint_trajectory'
GRIPPER_TOPIC = '/crane_plus_gripper_controller/joint_trajectory'
ARM_PUBLISH_PERIOD = 0.05           # 指令送信周期 [s] (teleop と同じ 20Hz)
ARM_TRAJ_DURATION = 0.1             # time_from_start [s] (teleop と同じ)
ARM_VEL_LIMIT = 0.5                 # 目標への追従速度上限 [rad/s]
ARM_GOAL_TOL = math.radians(1.0)    # 到達とみなす関節誤差 [rad]
ARM_MOVE_TIMEOUT = 20.0             # 1 ウェイポイントの移動タイムアウト [s]

# gripper 真下向き姿勢 (base座標系で y軸まわり180度)
Q_DOWN = (0.0, 1.0, 0.0, 0.0)

# グリッパ把持中心の y 方向オフセット補正 [m]
# hand ジョイントは link4 origin から y=-0.0145 にオフセットしており、
# 真下向き姿勢でもグリッパ中心が endtip から y=-0.0145 ずれる。
# キューブ中心にグリッパ中心を合わせるため endtip 目標を +y へ平行移動する。
GRIP_Y_OFFSET = 0.0145

# 真下向き姿勢で endtip (link4 原点) からグリッパ (hand ジョイント) までの
# 下方オフセット [m]。hand ジョイントは link4 origin から z=+0.045 にあり、
# 真下向き姿勢ではこの +z が base 座標系の真下を向くため、
# グリッパは endtip 目標より 45mm 下 (指先はさらにその下) に来る。
# ※ endtip_offset_xyz='0 0 0.09' を使う launch
#    (endtip_crane_plus_with_table.launch.py) ではこのオフセットは不要なので
#    --grasp-z を 0.075 程度 (把持中心そのもの) に戻すこと。
GRIP_Z_OFFSET = 0.045

# 把持後のキューブとハンド (link_hand 原点) 間の距離がこの値未満なら
# 「ハンドと一緒に運ばれている」とみなす [m]
CARRIED_DIST_MAX = 0.12

# キューブ中心が静止高さ (base座標 z=0, 桌面+2.5cm) よりこの値以上
# 高くなければ「桌面から離れていない」とみなす [m]
# 数十 mm の押し動き・傾き・物理ジッタを誤検知させないための下限。
# 上昇中の滑り (数 cm 下がる程度) は許容し、グリッパが低い位置で
# 把持したままでも桌面から十分離れていれば成功とみなす。
OFF_TABLE_Z = 0.015

# --jitter 指定時に endtip 目標 xy に加える一様乱数の振幅 [m] (±5mm)
JITTER_XY = 0.005


def ign_topic_once(topic, timeout=10):
    """ignition (Gazebo) トピックの最新1件をテキストで取得する."""
    env = dict(os.environ, IGN_IP='127.0.0.1')
    out = subprocess.run(
        ['ign', 'topic', '-t', topic, '-e', '-n', '1'],
        capture_output=True, text=True, timeout=timeout, env=env)
    if out.returncode != 0:
        raise RuntimeError(f'ign topic failed: {out.stderr.strip()}')
    return out.stdout


def parse_model_pose(text, model_name):
    """Pose_V テキストから指定モデルの position/orientation を取り出す."""
    # pose { ... name: "xxx" ... position { ... } orientation { ... } }
    pattern = re.compile(
        r'pose\s*\{[^{}]*?name:\s*"' + re.escape(model_name) + r'"'
        r'[^{}]*?position\s*\{([^{}]*)\}'
        r'[^{}]*?orientation\s*\{([^{}]*)\}', re.S)
    m = pattern.search(text)
    if not m:
        return None

    def vals(block):
        d = {}
        for key in ('x', 'y', 'z', 'w'):
            mm = re.search(key + r':\s*([-0-9.eE+]+)', block)
            if mm:
                d[key] = float(mm.group(1))
        return d

    p = vals(m.group(1))
    q = vals(m.group(2))
    return {
        'position': (p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)),
        'orientation': (q.get('x', 0.0), q.get('y', 0.0),
                        q.get('z', 0.0), q.get('w', 1.0)),
    }


def get_world_poses(model_names):
    text = ign_topic_once('/world/default/pose/info')
    return {name: parse_model_pose(text, name) for name in model_names}


def world_to_base(p_world, robot_pose):
    """world座標 -> crane_plus_base座標 (ロボットのyawのみ考慮)."""
    rx, ry, rz = robot_pose['position']
    qx, qy, qz, qw = robot_pose['orientation']
    yaw = 2.0 * math.atan2(qz, qw)  # 平面近似 (ロボットは水平設置)
    dx, dy = p_world[0] - rx, p_world[1] - ry
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy, p_world[2] - rz)


def apply_jitter(x, y, enabled, amp=JITTER_XY):
    """--jitter 有効時 endtip 目標 xy に ±amp [m] の一様乱数を加える.

    (x, y, jx, jy) を返す。無効時は jx=jy=0 で座標はそのまま。
    """
    if not enabled:
        return x, y, 0.0, 0.0
    jx = random.uniform(-amp, amp)
    jy = random.uniform(-amp, amp)
    return x + jx, y + jy, jx, jy


def down_orientation(x, y):
    """目標地点真下を向く姿勢 quaternion.

    Rz(j1) * Ry(180deg) = 水平軸 (水平面内で方位角 j1 の半角) まわり 180 度。
    グリッパを真下に向けたまま、腕の回転 (joint1) と整合する roll を与える。
    """
    j1 = math.atan2(y, x)
    return (-math.sin(j1 / 2.0), math.cos(j1 / 2.0), 0.0, 0.0)


def solve_ik(node, x, y, z, seed, cbg):
    """compute_ik で把持姿勢の逆運動学を解く. 成功時は関節角リストを返す."""
    cli = node.create_client(GetPositionIK, '/compute_ik', callback_group=cbg)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('/compute_ik サービスが見つかりません')
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'arm'
    req.ik_request.ik_link_name = ENDTIP_FRAME
    req.ik_request.avoid_collisions = True
    req.ik_request.timeout.sec = 5
    ps = PoseStamped()
    ps.header.frame_id = BASE_FRAME
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    qx, qy, qz, qw = down_orientation(x, y)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    req.ik_request.pose_stamped = ps
    js = JointState()
    js.name = ARM_JOINT_NAMES
    js.position = [float(v) for v in (seed if seed else HOME_POSE)]
    req.ik_request.robot_state.joint_state = js
    req.ik_request.robot_state.is_diff = True
    fut = cli.call_async(req)
    while rclpy.ok() and not fut.done():
        time.sleep(0.05)
    r = fut.result()
    node.destroy_client(cli)
    if r.error_code.val != 1:
        return None
    # 戻り値には gripper 等も含まれるため ARM_JOINT_NAMES の順で抽出
    pos = dict(zip(r.solution.joint_state.name,
                   r.solution.joint_state.position))
    return [pos[n] for n in ARM_JOINT_NAMES]


class JointStateReader:
    """/joint_states を購読して最新角度を保持する (executor スレッドで callback)."""

    def __init__(self, node, cbg):
        self.positions = {}
        self._sub = node.create_subscription(
            JointState, '/joint_states', self._cb, 10, callback_group=cbg)

    def _cb(self, msg):
        self.positions.update(dict(zip(msg.name, msg.position)))

    def get(self, name, default=0.0):
        return self.positions.get(name, default)

    def wait(self, names, timeout=5.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if all(n in self.positions for n in names):
                return True
            time.sleep(0.05)
        return all(n in self.positions for n in names)


class TrajectoryStreamer:
    """teleop_keyboard と同一形式の JointTrajectory を常時 20Hz で発行し続ける.

    腕・グリッパそれぞれの目標値 (target) を保持し、タイマーで毎周期
    /crane_plus_arm_controller/joint_trajectory と
    /crane_plus_gripper_controller/joint_trajectory へ送信する。
    移動ロジック (arm_move_to / gripper_move) は set_* で目標を更新するだけで、
    送信自体はストリーマーが担う。
    """

    def __init__(self, node, cbg, arm_initial, gripper_initial,
                 period=ARM_PUBLISH_PERIOD, duration=ARM_TRAJ_DURATION):
        self._lock = threading.Lock()
        self._arm = [float(v) for v in arm_initial]
        self._gripper = float(gripper_initial)
        self._duration = duration
        self._arm_pub = node.create_publisher(
            JointTrajectory, ARM_TOPIC, 10, callback_group=cbg)
        self._gripper_pub = node.create_publisher(
            JointTrajectory, GRIPPER_TOPIC, 10, callback_group=cbg)
        node.create_timer(period, self._tick, callback_group=cbg)

    def set_arm(self, positions):
        with self._lock:
            self._arm = [float(v) for v in positions]

    def set_gripper(self, position):
        with self._lock:
            self._gripper = float(position)

    def get_targets(self):
        with self._lock:
            return list(self._arm), self._gripper

    def _tick(self):
        arm, grip = self.get_targets()
        # Arm (teleop_keyboard の publish_commands と同一形式)
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = arm
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(self._duration * 1e9)
        jt.points.append(pt)
        self._arm_pub.publish(jt)
        # Gripper
        jt_g = JointTrajectory()
        jt_g.joint_names = [GRIPPER_JOINT_NAME]
        pt_g = JointTrajectoryPoint()
        pt_g.positions = [grip]
        pt_g.time_from_start.sec = 0
        pt_g.time_from_start.nanosec = int(self._duration * 1e9)
        jt_g.points.append(pt_g)
        self._gripper_pub.publish(jt_g)


def arm_move_to(streamer, js, q_target, timeout=ARM_MOVE_TIMEOUT):
    """ストリーマーの目標を q_target へ更新して腕を動かす (送信は 20Hz 常時).

    現在の目標位置から q_target まで ARM_VEL_LIMIT を上限として線形補間し,
    補間した中間目標をストリーマーに書き込み続ける
    (bag には teleop と同じ形式で指令が流れ続ける)。
    全関節の誤差が ARM_GOAL_TOL 未満なら True、タイムアウトなら False。
    """
    q0, _ = streamer.get_targets()
    max_delta = max(abs(t - c) for t, c in zip(q_target, q0))
    dur = max(0.8, max_delta / ARM_VEL_LIMIT)
    t0 = time.monotonic()
    while rclpy.ok() and (time.monotonic() - t0) < timeout:
        alpha = min(1.0, (time.monotonic() - t0) / dur)
        streamer.set_arm(
            [c + (t - c) * alpha for c, t in zip(q0, q_target)])
        err = max(abs(js.get(n) - t)
                  for n, t in zip(ARM_JOINT_NAMES, q_target))
        if err < ARM_GOAL_TOL:
            return True
        time.sleep(0.01)
    return False


def shutdown_ros(executor, spin_thread, node):
    """executor -> spin thread -> node の順で安全に終了する.

    spin スレッドが生きたまま node を destroy すると segfault するため、
    executor.shutdown() 後にスレッドを join してから破棄する。
    """
    executor.shutdown()
    spin_thread.join(timeout=3.0)
    node.destroy_node()
    rclpy.try_shutdown()


def grasp_cube(model, grasp_z, pre_z, dry_run, allow_unreachable,
               y_offset=GRIP_Y_OFFSET, dz_threshold=0.005, strict=False,
               ret=True, jitter=False):
    # --- 1. 位置取得 (home 移動後にもう一度取得し直す) ----------------------
    print(f'[1/7] Fetching pose of "{model}" from Gazebo ...')
    poses = get_world_poses([model, 'crane_plus'])
    cube, robot = poses[model], poses['crane_plus']
    if cube is None:
        print(f'Error: モデル "{model}" が Gazebo に見つかりません', file=sys.stderr)
        return 1
    if robot is None:
        print('Error: ロボット "crane_plus" の姿勢が見つかりません', file=sys.stderr)
        return 1

    def cube_in_base():
        p = get_world_poses([model, 'crane_plus'])
        c_, r_ = p[model], p['crane_plus']
        if c_ is None:
            raise RuntimeError(f'モデル "{model}" が見つかりません')
        cx, cy, cz = world_to_base(c_['position'], r_)
        yaw = 2.0 * math.atan2(c_['orientation'][2], c_['orientation'][3])
        print(f'  cube (world): ({c_["position"][0]:+.3f}, '
              f'{c_["position"][1]:+.3f}, {c_["position"][2]:+.3f})')
        print(f'  cube (base) : ({cx:+.3f}, {cy:+.3f}, {cz:+.3f})  '
              f'yaw={math.degrees(yaw):+.1f} deg')
        return c_, (cx, cy, cz)

    cube, (cx, cy, z) = cube_in_base()
    x, y, jx, jy = apply_jitter(cx, cy + y_offset, jitter)
    print(f'  endtip 目標 : ({x:+.3f}, {y:+.3f}, {grasp_z:+.3f}) (y補正 {y_offset:+.4f})'
          + (f' [jitter {jx*1000:+.1f}, {jy*1000:+.1f} mm]' if jitter else ''))

    rclpy.init()
    node = rclpy.create_node('demo1')
    cbg = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 把持姿勢の逆運動学 (到達性判定を兼ねる)
    grasp_q = solve_ik(node, x, y, grasp_z, HOME_POSE, cbg)
    if grasp_q is None:
        dist = math.hypot(x, y)
        print(f'Error: 把持姿勢 (endtip: {x:+.3f}, {y:+.3f}, {grasp_z}) は'
              f'到達可能範囲外です (基底からの距離 {dist:.3f} m)。'
              'GUI でキューブを動かし再実行してください。'
              ' (概ね 0.08〜0.17 m の範囲)', file=sys.stderr)
        if not allow_unreachable:
            shutdown_ros(executor, spin_thread, node)
            return 2
        print('[WARN] allow-unreachable 指定のため続行します')
    else:
        print('  reachability: OK  q=[' +
              ', '.join(f'{v:+.2f}' for v in grasp_q) + ']')

    if dry_run:
        print('dry-run のため動作をスキップします。')
        shutdown_ros(executor, spin_thread, node)
        return 0

    # --- 2. 制御準備 (腕・グリッパとも 20Hz 常時直接発行 = bag に記録される) --
    print('[2/7] Initializing arm controller interface ...')
    js = JointStateReader(node, cbg)
    if not js.wait(ARM_JOINT_NAMES, timeout=5.0):
        print('Error: /joint_states を取得できません', file=sys.stderr)
        shutdown_ros(executor, spin_thread, node)
        return 4

    # 現在角度で目標を初期化してから 20Hz 常時送信を開始する
    # (teleop と違いゼロ位置初期化ではないため、指令開始時に腕が飛ばない)
    streamer = TrajectoryStreamer(
        node, cbg,
        arm_initial=[js.get(n) for n in ARM_JOINT_NAMES],
        gripper_initial=GRIPPER_OPEN)

    def gripper_move(position):
        streamer.set_gripper(position)
        time.sleep(1.2)  # 動作完了待ち

    # ポーズゴールでのプランニングは失敗しやすいため、
    # ウェイポイントごとに IK を解いて関節空間で移動する
    last_q = HOME_POSE[:]

    def move_pose(zz):
        nonlocal last_q
        q = solve_ik(node, x, y, zz, last_q, cbg)
        if q is None:
            raise RuntimeError(f'IK 失敗 (target z={zz})')
        if not arm_move_to(streamer, js, q):
            raise RuntimeError(
                f'腕の移動がタイムアウトしました (target z={zz})')
        last_q = q

    try:
        # --- 3. home + グリッパ開 ------------------------------------------
        print('[3/7] Move to home, open gripper ...')
        gripper_move(GRIPPER_OPEN)
        if not arm_move_to(streamer, js, HOME_POSE):
            raise RuntimeError('home 移動がタイムアウトしました')

        # home 退避後に再取得 (腕の上に乗っていた場合などに備える)
        print('  re-reading cube pose after moving to home ...')
        time.sleep(1.0)  # キューブが落ち着くのを待つ
        cube, (cx, cy, z) = cube_in_base()
        x, y, jx, jy = apply_jitter(cx, cy + y_offset, jitter)
        print(f'  endtip 目標 : ({x:+.3f}, {y:+.3f}, {grasp_z:+.3f})'
              + (f' [jitter {jx*1000:+.1f}, {jy*1000:+.1f} mm]' if jitter else ''))
        q = solve_ik(node, x, y, grasp_z, HOME_POSE, cbg)
        if q is None:
            raise RuntimeError(
                f'再取得した位置 ({x:+.3f}, {y:+.3f}, {grasp_z}) で IK 失敗')
        last_q = HOME_POSE[:]

        # --- 4. キューブ真上へ ---------------------------------------------
        print(f'[4/7] Move above cube ({x:+.3f}, {y:+.3f}, {pre_z}) ...')
        move_pose(pre_z)

        # --- 5. 下降して把持 -----------------------------------------------
        print(f'[5/7] Descend to z={grasp_z} and close gripper ...')
        # 下降先は grasp_z (既定 0.140)。0.080 / 0.100 / 0.120 では
        # 位置が低すぎたため段階的に引き上げている
        move_pose(grasp_z)
        gripper_move(GRIPPER_CLOSE)

        # --- 6. キューブを掴んだまま最初の姿勢 (home) へ復帰 --------------
        #   グリッパは閉じたまま腕だけを home 関節角へ戻す。
        #   把持失敗時も机の上に戻す意味で同様に復帰する。
        #   ※ 把持後の上昇 (move_pose(lift_z)) は行わず、
        #     home 復帰後の段階 (step 7) で成否確認する。
        if ret:
            print('[6/7] Return to initial pose (home), gripper kept closed ...')
            if not arm_move_to(streamer, js, HOME_POSE):
                print('[WARN] home 復帰がタイムアウトしました', file=sys.stderr)
        else:
            print('[6/7] Skip home return (--no-ret) ...')

        # --- 7. 成否確認 (home 復帰後の段階) --------------------------------
        #   --no-ret 時は home 復帰しないためキューブ状態を調べず成功とする
        if not ret:
            print('[7/7] Skip grasp verification (--no-ret) ...')
            code = 0
            return code
        print('[7/7] Verify grasp result ...')
        time.sleep(1.0)  # キューブが安定するのを待つ
        poses_after = get_world_poses(
            [model, 'crane_plus_link_hand', 'crane_plus'])
        cube_after = poses_after[model]
        hand_after = poses_after.get('crane_plus_link_hand')
        robot_after = poses_after['crane_plus']
        dz = cube_after['position'][2] - cube['position'][2]

        # 判定1: baseline からの z 上昇量
        lifted = dz > dz_threshold

        # 判定2: ハンドと一緒に運ばれているか
        #   (baseline が汚染されていた場合でも成立する補助判定)
        carried = False
        dist_hand = None
        if hand_after is not None:
            dist_hand = math.dist(cube_after['position'],
                                  hand_after['position'])
            carried = dist_hand < CARRIED_DIST_MAX

        print(f'  cube z: {cube["position"][2]:.3f} -> '
              f'{cube_after["position"][2]:.3f} '
              f'(dz={dz:+.3f} m, threshold {dz_threshold})')
        if dist_hand is not None:
            print(f'  dist(cube, hand) = {dist_hand:.3f} m '
                  f'(< {CARRIED_DIST_MAX} -> carried)')

        # 判定3: 桌面から実際に離れているか
        #   base座標系でのキューブ中心の高さ (静止時は z=0 = 桌面+2.5cm)
        cube_z_base = cube_after['position'][2] - robot_after['position'][2]
        off_table = cube_z_base > OFF_TABLE_Z
        print(f'  cube z (base frame): {cube_z_base:+.3f} m '
              f'(> {OFF_TABLE_Z} -> off_table)')

        print(f'  lifted={lifted} carried={carried} off_table={off_table}')
        # 判定方針: 桌面から離れていれば成功とする。
        #   - 上昇中にキューブが多少滑り落ちても、机から浮いていれば
        #     把持成功とみなす (OFF_TABLE_Z が実質的な下限)
        #   - lifted / carried は誤認防止の必須条件から外し、
        #     診断用の参考情報として表示する
        #   - 把持できず桌面に残った場合のみ off_table=False で FAIL
        # --strict 時は旧判定 (off_table かつ lifted/carried) を要求
        if strict:
            if hand_after is None:
                success = lifted and off_table
            else:
                success = off_table and (lifted or carried)
        else:
            success = off_table
        print('GRASP SUCCESS' if success else 'GRASP FAILED (cube not lifted)')
        code = 0 if success else 3
    except RuntimeError as e:
        print(f'Error: {e}', file=sys.stderr)
        code = 4
    finally:
        shutdown_ros(executor, spin_thread, node)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Gazebo からキューブ位置を取得してグリッパで掴む',
    )
    parser.add_argument('--model', default='wood_cube_5cm',
                        help='掴むモデル名 (既定: wood_cube_5cm)')
    parser.add_argument('--grasp-z', type=float,
                        default=0.075 + GRIP_Z_OFFSET + 0.02,
                        help='把持時の endtip 高さ [m] base座標系 '
                             '(既定: 0.140 = 把持中心 0.075 + GRIP_Z_OFFSET '
                             f'{GRIP_Z_OFFSET} + 0.02) ※ 0.080/0.100/0.120 '
                             'では位置が低すぎたため段階的に引き上げ済み')
    parser.add_argument('--pre-z', type=float, default=0.17,
                        help='真上待機時の endtip 高さ [m] (既定: 0.17)')
    parser.add_argument('--dry-run', action='store_true',
                        help='位置取得と到達性判定のみ行う')
    parser.add_argument('--allow-unreachable', action='store_true',
                        help='到達不能でも実行を試みる')
    parser.add_argument('--y-offset', type=float, default=GRIP_Y_OFFSET,
                        help='グリッパ中心の y 方向オフセット補正 [m] (既定: 0.0145)')
    parser.add_argument('--dz-threshold', type=float, default=0.005,
                        help='成功とみなす baseline からの z 上昇量 [m] '
                             '(既定: 0.005, --strict 時のみ使用)')
    parser.add_argument('--strict', action='store_true',
                        help='厳しい判定 (off_table かつ lifted/carried) を要求する '
                             '(既定: 桌面から離れていれば成功)')
    parser.add_argument('--jitter', action='store_true',
                        help=f'endtip 目標 xy に ±{JITTER_XY * 1000:.0f}mm の'
                             'ランダムジッターを加える (既定: 無効)')
    parser.add_argument('--no-ret', dest='ret', action='store_false',
                        help='把持後に最初の姿勢 (home = 直立) へ復帰しない。'
                             'home 復帰後の成否確認も省略し成功 (0) を返す '
                             '(既定: 掴んだまま home へ復帰し、'
                             '復帰後に成否確認する)')
    args = parser.parse_args(argv)
    return grasp_cube(args.model, args.grasp_z, args.pre_z,
                      args.dry_run, args.allow_unreachable,
                      args.y_offset, args.dz_threshold, args.strict,
                      args.ret, args.jitter)


if __name__ == '__main__':
    sys.exit(main())
