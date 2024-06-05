import open3d as o3d
import numpy as np

# 点群データを生成
print("点群データを生成しています...")
point_cloud = o3d.geometry.PointCloud()

# ランダムな点を生成
num_points = 100  # 生成する点の数
np.random.seed(42)  # 結果の再現性のためのシード
points = np.random.rand(num_points, 3)  # 0から1の範囲でランダムな点を生成

point_cloud.points = o3d.utility.Vector3dVector(points)

# 点群データを表示
o3d.visualization.draw_geometries([point_cloud])
