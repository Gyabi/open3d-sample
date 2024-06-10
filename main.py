import open3d as o3d
import numpy as np
import point_cloud_utils as pcu
from scipy.spatial import KDTree
from scipy.interpolate import griddata, CloughTocher2DInterpolator, LinearNDInterpolator,RBFInterpolator

def visualize(target):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True)
    vis.get_render_option().background_color = [0,0,1]
    vis.add_geometry(target)
    
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100.0, origin=[0,0,0])
    vis.add_geometry(axis)
    
    vis.run()
    vis.destroy_window()
    
def generate_point_cloud(x_range:tuple, y_range:tuple, z_range:tuple, num_points:int, slope:float=0) -> o3d.geometry.PointCloud:
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    z = np.random.uniform(z_range[0], z_range[1], num_points) + slope * y
    
    points = np.vstack((x, y, z)).T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    return point_cloud

def generate_point_cloud_sin_slope(x_range:tuple, y_range:tuple, z_range:tuple, num_points:int, slope:float=0) -> o3d.geometry.PointCloud:
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    z = np.random.uniform(z_range[0], z_range[1], num_points) + slope * np.sin(y)
    
    points = np.vstack((x, y, z)).T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    return point_cloud


def extend_point_cloud(pcd:o3d.geometry.PointCloud, x_range_min:float=-50, x_range_max:float=50, y_range_min:float=-1, y_range_max:float=300, grid_size:float=1) -> o3d.geometry.PointCloud:
    points = np.asarray(pcd.points)
    
    # xy座標抽出
    points_xy = points[:, :2]
    # z座標抽出
    points_z = points[:, 2]
    
    # 最近傍検索用のKDTree作成
    pcd_tree = KDTree(points_xy)
    
    # 四方の点を計算
    for x in [x_range_min, x_range_max]:
        for y in [y_range_min, y_range_max]:
            points_xy = np.append(points_xy, np.array([[x, y]]), axis=0)
            # 最近傍の点を検索
            nearest_point = pcd_tree.query([x, y], k=1)
            # 最近傍の点のz座標を取得
            nearest_point_z = points_z[nearest_point[1]]
            print(nearest_point_z)
            points_z = np.append(points_z, nearest_point_z)
    
    # meshgrid作成
    grid_x, grid_y = np.meshgrid(
        np.arange(x_range_min, x_range_max, grid_size),
        np.arange(y_range_min, y_range_max, grid_size)
    )
    
    # 拡張する点を計算
    new_points = []
    # interpolator = CloughTocher2DInterpolator(points_xy, points_z)
    interpolator = LinearNDInterpolator(points_xy, points_z)
    
    for (x_row, y_row) in zip(grid_x, grid_y):
        for (x, y) in zip(x_row, y_row):
            new_points.append([x, y, interpolator(x, y)])
    
    extended_points = np.vstack((points, np.array(new_points)))
    extended_pcd = o3d.geometry.PointCloud()
    extended_pcd.points = o3d.utility.Vector3dVector(extended_points)
    return extended_pcd
   
# pcd = generate_point_cloud_sin_slope((-10, 10), (-1, 250), (-0.1, 0.1), 1000, slope=0.5)
pcd = generate_point_cloud((-10, 10), (-1, 250), (-0.1, 0.1), 1000, slope=0.5)
# visualize(pcd)

# pcdを拡張
extended_pcd = extend_point_cloud(pcd)

visualize(extended_pcd)

# デプスマップを生成して表示