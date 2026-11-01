# phisical-ai-advanced1
PhisicalAI応用1講座 最終課題用のリポジトリ

内容:
- crane_plus_automator
- 応用

# crane_plus_automator

これは、Gazebo の `/world/default/pose/info` (SceneBroadcaster) からキューブ位置を取得し、
MoveIt の `/compute_ik` で逆運動学を解いて crane_plus のグリッパで掴むパッケージ。
JointTrajectory を常時 20Hz で発行し続け`ros2 bag record`で記録可能です。

## ビルド

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select grab_cube
source install/setup.bash
```

## 実行

Gazebo + MoveIt (crane_plus_with_table.launch.py 等) が起動した状態で:

```bash
ros2 run crane_plus_automator demo1            # 掴んで home 復帰し成否確認
ros2 run crane_plus_automator demo1 --jitter   # 目標値に+-5mmのジッタを与える
ros2 run crane_plus_automator demo1 --no-ret   # キューブを掴んだ時点で停止する
```

## reset_state (Gazebo 初期状態復元)

```bash
ros2 run crane_plus_automator reset_state            # 復元 + 検証
ros2 run crane_plus_automator reset_state --verify   # 検証のみ
```

- アーム: ros2_control の joint_trajectory トピックへ目標を直接発行
- キューブ: Gazebo サービス `/world/default/set_pose` を `ign service` で直接呼び出し


# 応用

## GazeboとCRANE+ V2マニピュレータを用いて、訓練データ作り＆ファインチューニングを試す

Docker containerを起動する。そして、`http://localhost:6080'にブラウザでアクセスして「接続」のボタンを押しUbuntuに接続する。

```
docker pull airobotbook/ros2-desktop-ai-robot-book-humble:latest
docker run -p 6080:80 --shm-size=512m airobotbook/ros2-desktop-
ai-robot-book-humble:latest
```

Ubuntuに接続したら、Terminalを開く。

pixi をインストールする。
```
curl -fsSL https://pixi.sh/install.sh | bash
echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
source ~/.bashrc
```

pai_training をセットアップする。
```
cd ~ && git clone --recursive https://github.com/matsuolab/pai_training
pixi install
```

本リポジトリ(crane_plus_automator)をセットアップする。
```
mkdir -p ~/pai_ws/src
cd ~/pai_ws/src && git clone https://github.com/refineural_lab/crane_plus_automator.git
cd ~/pai_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select crane_plus_automator
source install/setup.bash
```

Gazebo & CRANE+ V2 起動する。
```
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

訓練データを作る。episodeを１つ生成し記録する。

```
ros2 run crane_plus_automator reset_state
ros2 bag record -a -o ~/dataset/rosbags/pekori/ep0 \
& ros2 run crane_plus_automator demo1 --jitter ; fg
# Ctrl+C でrecodingを終了
```

変換する。
```
cd ~/pai_training
pixi run convert
```

訓練設定を調整する。`pai_training/configs/train_config.yaml`の中でも、特に`action`パラメータを20Hzに対応するように変更する。
```
delta_timestamps:
  action: [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75]
```

ファインチューニングする。
```
cd ~/pai_training
WANDB_MODE=offline pixi run train
```

デプロイする。
```
cd ~/pai_training
ros2 run crane_plus_automator reset_state
pixi run infer --model-path ~/checkpoints/train/crane_plus_pekori_act
```