import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math

# 表示オプションを設定（小数点以下3桁まで表示）
np.set_printoptions(suppress=True, precision=6, threshold=np.inf, linewidth=np.inf)

def calc_diff_deg_2_vector_3d(v1, v2):
    """
    2つの3Dベクトル間の角度を計算します。
    v1, v2: 入力ベクトル
    """
    v1 = np.array(v1)
    v2 = np.array(v2)
    
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    
    angle_rad = np.arccos(dot_product / (norm_v1 * norm_v2))
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg


# 視野角と画像サイズの設定
fov = 60
w = 640
h = 480

# RaycastingSceneの生成
scene = o3d.t.geometry.RaycastingScene()

# ピンホールカメラのrayを生成
rays = scene.create_rays_pinhole(
    fov_deg=fov,
    center=[0., 1., 0.],
    eye=[0., 0., 0.],
    up=[0., 0., 1.],
    width_px=w,
    height_px=h
)

# numpy配列に変換
rays_np = rays.numpy()

# 各rayを単位ベクトルに変換
rays_np[..., 3:] /= np.linalg.norm(rays_np[..., 3:], axis=-1, keepdims=True)

# 中央の縦一列を取得
center_col_rays = rays_np[:, w // 2, 3:]

# 縦一列の角度差分を計算
angle_diffs_col = []
for i in range(len(center_col_rays) - 1):
    angle_diff = calc_diff_deg_2_vector_3d(center_col_rays[i], center_col_rays[i + 1])
    angle_diffs_col.append(angle_diff)

# 中央の横一列を取得
center_row_rays = rays_np[h // 2, :, 3:]

# 横一列の角度差分を計算
angle_diffs_row = []
for i in range(len(center_row_rays) - 1):
    angle_diff = calc_diff_deg_2_vector_3d(center_row_rays[i], center_row_rays[i + 1])
    angle_diffs_row.append(angle_diff)

# 中央の縦一列のベクトルを表示
print("Center column vectors:")
print(center_col_rays)

# 中央の横一列のベクトルを表示
print("Center row vectors:")
print(center_row_rays)

# 中央の縦一列の角度差分の折れ線グラフを表示
plt.figure()
plt.plot(angle_diffs_col, marker='o')
plt.xlabel('Ray Index')
plt.ylabel('Angle Difference (degrees)')
plt.title('Angle Differences Between Adjacent Rays in Center Column')
plt.grid(True)

# 中央の縦一列の角度差分のヒストグラムを表示
plt.figure()
plt.hist(angle_diffs_col, bins=20, edgecolor='black')
plt.xlabel('Angle Difference (degrees)')
plt.ylabel('Frequency')
plt.title('Histogram of Angle Differences Between Adjacent Rays in Center Column')
plt.grid(True)

# 中央の横一列の角度差分の折れ線グラフを表示
plt.figure()
plt.plot(angle_diffs_row, marker='o')
plt.xlabel('Ray Index')
plt.ylabel('Angle Difference (degrees)')
plt.title('Angle Differences Between Adjacent Rays in Center Row')
plt.grid(True)

# 中央の横一列の角度差分のヒストグラムを表示
plt.figure()
plt.hist(angle_diffs_row, bins=20, edgecolor='black')
plt.xlabel('Angle Difference (degrees)')
plt.ylabel('Frequency')
plt.title('Histogram of Angle Differences Between Adjacent Rays in Center Row')
plt.grid(True)

plt.show()
