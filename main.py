import open3d as o3d
import numpy as np
import point_cloud_utils as pcu
from scipy.spatial import KDTree

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


def extend_point_cloud(pcd:o3d.geometry.PointCloud, x_range_min:float=-50, x_range_max:float=50, y_range_min:float=-1, y_range_max:float=300, grid_size:float=1) -> o3d.geometry.PointCloud:
    # x_range_minからx_range_maxをgrid_sizeの幅で分割
    x_range = np.arange(x_range_min, x_range_max, grid_size)
    # y_range_minからy_range_maxをgrid_sizeの幅で分割
    y_range = np.arange(y_range_min, y_range_max, grid_size)
    
    # 最近傍探索用にKDTreeを作成
    tree_data = np.asarray(pcd.points)[:, :2]
    pcd_tree = KDTree(tree_data)
    
    # 点群データをコピー
    points = np.asarray(pcd.points)
    
    # 拡張座標を計算
    new_points = []
    for x in x_range:
        for y in y_range:            
            distance, index = pcd_tree.query([x, y], k=1)
            nearest_point = points[index]
            new_points.append([x, y, nearest_point[2]])
    
    extended_points = np.vstack((points, np.array(new_points)))
    extended_pcd = o3d.geometry.PointCloud()
    extended_pcd.points = o3d.utility.Vector3dVector(extended_points)
    return extended_pcd

pcd = generate_point_cloud((-10, 10), (-1, 250), (-0.1, 0.1), 1000, slope=0.5)
# visualize(pcd)

# pcdを拡張
extended_pcd = extend_point_cloud(pcd)
visualize(extended_pcd)
