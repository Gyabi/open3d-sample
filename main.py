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
            # ボクセル化とgriddataを組み合わせる！     
            distance, index = pcd_tree.query([x, y], k=1)
            nearest_point = points[index]
            new_points.append([x, y, nearest_point[2]])
    
    extended_points = np.vstack((points, np.array(new_points)))
    extended_pcd = o3d.geometry.PointCloud()
    extended_pcd.points = o3d.utility.Vector3dVector(extended_points)
    return extended_pcd

def expand_point_cloud(point_cloud, space_size, resolution=0.1):
    grid_x, grid_y = np.meshgrid(
        np.arange(0, space_size, resolution),
        np.arange(0, space_size, resolution)
    )
   
    points = np.asarray(point_cloud.points)
    points_2d = points[:, :2]
    values = points[:, 2]
    
    points_2d = np.append(points_2d, np.array([[300, 300]]), axis=0)
    points_2d = np.append(points_2d, np.array([[300, 0]]), axis=0)
    points_2d = np.append(points_2d, np.array([[-300, 300]]), axis=0)
    points_2d = np.append(points_2d, np.array([[-300, 0]]), axis=0)
    
    # values = np.append(values, 0.5*np.sin(300))
    values = np.append(values, 0.5*300)
    values = np.append(values, 0)
    # values = np.append(values, 0.5*np.sin(300))
    values = np.append(values, 0.5*300)
    values = np.append(values, 0)
    
    grid_z = griddata(points_2d, values, (grid_x, grid_y), method='linear')
    # grid_z = griddata(points_2d, values, (grid_x, grid_y), method='cubic')
   
    #元のポイントクラウドへgridデータを追加
    new_points = []
    for (x_row,y_row) in zip(grid_x, grid_y):
        for (x,y) in zip(x_row, y_row):
                new_points.append([x, y, grid_z[int(y)][int(x)]])
    
    
    expanded_points = np.vstack((points, np.array(new_points)))
    new_point_cloud = o3d.geometry.PointCloud()
    new_point_cloud.points = o3d.utility.Vector3dVector(expanded_points)
    return new_point_cloud
   
def x(point_cloud, space_size, resolution=0.1) -> o3d.geometry.PointCloud:
    points = np.asarray(point_cloud.points)
    points_2d = points[:, :2]
    values = points[:, 2]
    
    points_2d = np.append(points_2d, np.array([[300, 300]]), axis=0)
    points_2d = np.append(points_2d, np.array([[300, 0]]), axis=0)
    points_2d = np.append(points_2d, np.array([[-300, 300]]), axis=0)
    points_2d = np.append(points_2d, np.array([[-300, 0]]), axis=0)
    
    values = np.append(values, 0.5*np.sin(300))
    # values = np.append(values, 0.5*300)
    values = np.append(values, 0)
    values = np.append(values, 0.5*np.sin(300))
    # values = np.append(values, 0.5*300)
    values = np.append(values, 0)
    
    interpolator = CloughTocher2DInterpolator(points_2d, values)
    # interpolator = LinearNDInterpolator(points_2d, values)
    
    grid_x, grid_y = np.meshgrid(
       np.arange(0, space_size, resolution),
       np.arange(0, space_size, resolution)
    )
    

    new_points = []
    for (x_row,y_row) in zip(grid_x, grid_y):
        for (x,y) in zip(x_row, y_row):
            new_points.append([x, y, interpolator(x, y)])
            
    expanded_points = np.vstack((points, np.array(new_points)))
    new_point_cloud = o3d.geometry.PointCloud()
    new_point_cloud.points = o3d.utility.Vector3dVector(expanded_points)
    return new_point_cloud


# pcd = generate_point_cloud_sin_slope((-10, 10), (-1, 250), (-0.1, 0.1), 1000, slope=0.5)
pcd = generate_point_cloud((-10, 10), (-1, 250), (-0.1, 0.1), 1000, slope=0.5)
# visualize(pcd)

# pcdを拡張
# extended_pcd = extend_point_cloud(pcd)
# extended_pcd = expand_point_cloud(pcd, 500, resolution=1)
a = x(pcd, 400, resolution=1)


# visualize(extended_pcd)
visualize(a)




# デプスマップを生成して表示