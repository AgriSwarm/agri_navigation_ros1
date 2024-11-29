import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# ROSバッグを絶対パスで指定
bag_path = '/home/tomoking/catkin_ws/rosbag/acc_2024-11-29-16-01-18.bag'

# ROSバッグを開く
bag = rosbag.Bag(bag_path)

# データを格納するリスト
times = []
velocities = []
accelerations = []
jerks = []

# オドメトリデータを読み込む
for topic, msg, t in bag.read_messages(topics=['/d2vins/odometry']):
    # 時間を記録（最初の時刻を0とする）
    if not times:
        start_time = t.to_sec()
    times.append(t.to_sec() - start_time)
    
    # 速度のノルムを計算
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    v_norm = np.sqrt(vx**2 + vy**2)
    velocities.append(v_norm)

# 加速度を計算（差分）
dt = np.diff(times)
dv = np.diff(velocities)
accelerations = dv / dt

# ジャークを計算（加速度の差分）
dt2 = np.diff(times[1:])  # 加速度の時刻差分
da = np.diff(accelerations)
jerks = da / dt2

# プロット
plt.figure(figsize=(12, 12))

# 速度のプロット
plt.subplot(3, 1, 1)
plt.plot(times, velocities)
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('Velocity Norm')

# 加速度のプロット
plt.subplot(3, 1, 2)
plt.plot(times[1:], accelerations)
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s²]')
plt.title('Acceleration Norm')

# ジャークのプロット
plt.subplot(3, 1, 3)
plt.plot(times[2:], jerks)
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Jerk [m/s³]')
plt.title('Jerk Norm')

plt.tight_layout()
plt.show()

# 統計情報の表示
print(f"Velocity - Mean: {np.mean(velocities):.3f} m/s, Max: {np.max(velocities):.3f} m/s")
print(f"Acceleration - Mean: {np.mean(np.abs(accelerations)):.3f} m/s², Max: {np.max(np.abs(accelerations)):.3f} m/s²")
print(f"Jerk - Mean: {np.mean(np.abs(jerks)):.3f} m/s³, Max: {np.max(np.abs(jerks)):.3f} m/s³")

# バッグを閉じる
bag.close()