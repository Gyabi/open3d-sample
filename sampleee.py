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

def create_rays_from_camera_params(fov_deg, width, height, camera_origin, camera_axis, camera_up):
    # Compute intrinsic matrix
    fx = width / (2 * np.tan(np.radians(fov_deg / 2)))
    fy = fx
    cx = width / 2
    cy = height / 2
    intrinsic_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # Construct extrinsic matrix
    z_axis = -camera_axis / np.linalg.norm(camera_axis)
    x_axis = np.cross(camera_up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
    extrinsic_matrix = np.eye(4)
    extrinsic_matrix[:3, :3] = rotation_matrix
    extrinsic_matrix[:3, 3] = camera_origin

    # Call Open3D function to create rays
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        o3d.core.Tensor(intrinsic_matrix, dtype=o3d.core.Dtype.Float64),
        o3d.core.Tensor(extrinsic_matrix, dtype=o3d.core.Dtype.Float64),
        width, height
    )

    return rays

def show_graph(cols, rows):
    # 中央の縦一列の角度差分の折れ線グラフを表示
    plt.figure()
    plt.plot(cols, marker='o')
    plt.xlabel('Ray Index')
    plt.ylabel('Angle Difference (degrees)')
    plt.title('Angle Differences Between Adjacent Rays in Center Column')
    plt.grid(True)

    # 中央の縦一列の角度差分のヒストグラムを表示
    plt.figure()
    plt.hist(cols, bins=20, edgecolor='black')
    plt.xlabel('Angle Difference (degrees)')
    plt.ylabel('Frequency')
    plt.title('Histogram of Angle Differences Between Adjacent Rays in Center Column')
    plt.grid(True)

    # 中央の横一列の角度差分の折れ線グラフを表示
    plt.figure()
    plt.plot(rows, marker='o')
    plt.xlabel('Ray Index')
    plt.ylabel('Angle Difference (degrees)')
    plt.title('Angle Differences Between Adjacent Rays in Center Row')
    plt.grid(True)

    # 中央の横一列の角度差分のヒストグラムを表示
    plt.figure()
    plt.hist(rows, bins=20, edgecolor='black')
    plt.xlabel('Angle Difference (degrees)')
    plt.ylabel('Frequency')
    plt.title('Histogram of Angle Differences Between Adjacent Rays in Center Row')
    plt.grid(True)

    plt.show()
    

def show(rays):
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
    # print("Center column vectors:")
    # print(center_col_rays)

    # 中央の横一列のベクトルを表示
    # print("Center row vectors:")
    # print(center_row_rays)
    
    # show_graph(center_col_rays, center_row_rays)
    print(calc_diff_deg_2_vector_3d(center_col_rays[0], center_col_rays[-1]))


    

# Example usage:
fov_deg = 60.0  # Example FOV
w = 640
h = 480
camera_origin = np.array([0.0, 0.0, 0.0])  # Example camera origin
camera_axis = np.array([0.0, 1.0, 0.0])  # Example camera axis direction (negative z-axis)
camera_up = np.array([0.0, 0.0, 1.0])  # Example camera up direction

rays_1 = create_rays_from_camera_params(fov_deg, w, h, camera_origin, camera_axis, camera_up)
rays_2 = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
    fov_deg=fov_deg,
    center=[0., 1., 0.],
    eye=[0., 0., 0.],
    up=[0., 0., 1.],
    width_px=w,
    height_px=h
)

show(rays_1)
show(rays_2)


rays1_np = rays_1.numpy()
rays2_np = rays_2.numpy()