import open3d as o3d
import numpy as np
import point_cloud_utils as pcu
from scipy.spatial import KDTree
from scipy.interpolate import griddata, CloughTocher2DInterpolator, LinearNDInterpolator,RBFInterpolator
import copy

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

def my_CT(xy, z):
    """CT interpolator + nearest-neighbor extrapolation.

    Parameters
    ----------
    xy : ndarray, shape (npoints, ndim)
        Coordinates of data points
    z : ndarray, shape (npoints)
        Values at data points

    Returns
    -------
    func : callable
        A callable object which mirrors the CT behavior,
        with an additional neareast-neighbor extrapolation
        outside of the data range.
    """
    x = xy[:, 0]
    y = xy[:, 1]
    f = CloughTocher2DInterpolator(xy, z)
    # f = LinearNDInterpolator(xy, z)

    # this inner function will be returned to a user
    def new_f(xx, yy):
        # evaluate the CT interpolator. Out-of-bounds values are nan.
        zz = f(xx, yy)
        nans = np.isnan(zz)

        if nans.any():
            # for each nan point, find its nearest neighbor
            inds = np.argmin(
                (x[:, None] - xx[nans])**2 +
                (y[:, None] - yy[nans])**2
                , axis=0)
            # ... and use its value
            zz[nans] = z[inds]
        return zz

    return new_f

def extend_point_cloud(pcd:o3d.geometry.PointCloud, x_range_min:float=-50, x_range_max:float=50, y_range_min:float=-1, y_range_max:float=300, grid_size:float=1) -> o3d.geometry.PointCloud:
    points = np.asarray(pcd.points)
    
    # xy座標抽出
    points_xy = points[:, :2]
    # z座標抽出
    points_z = points[:, 2]
    
    # 最近傍検索用のKDTree作成
    pcd_tree = KDTree(points_xy)
    
    # 補助用の点を計算
    inter_points_xy = points_xy.copy()
    inter_points_z = points_z.copy()
    
    for x in [x_range_min, x_range_max, np.max(points_xy[:, 0])]:
        for y in [y_range_min, y_range_max, np.max(points_xy[:, 1])]:
            inter_points_xy = np.append(inter_points_xy, np.array([[x, y]]), axis=0)
            # 最近傍の点を検索
            nearest_point = pcd_tree.query([x, y], k=1)
            # 最近傍の点のz座標を取得
            nearest_point_z = points_z[nearest_point[1]]
            inter_points_z = np.append(inter_points_z, nearest_point_z)
    
    # meshgrid作成
    grid_x, grid_y = np.meshgrid(
        np.arange(x_range_min, x_range_max, grid_size),
        np.arange(y_range_min, y_range_max, grid_size)
    )
    
    # 拡張する点を計算
    new_points = []
    # interpolator = CloughTocher2DInterpolator(inter_points_xy, inter_points_z)
    interpolator = LinearNDInterpolator(inter_points_xy, inter_points_z)
    # interpolator = my_CT(inter_points_xy, inter_points_z)
    
    for (x_row, y_row) in zip(grid_x, grid_y):
        for (x, y) in zip(x_row, y_row):
            new_points.append([x, y, interpolator(x, y)])
    
    extended_points = np.vstack((points, np.array(new_points)))
    extended_pcd = o3d.geometry.PointCloud()
    extended_pcd.points = o3d.utility.Vector3dVector(extended_points)
    return extended_pcd

def extend_point_cloud_voxel(pcd:o3d.geometry.PointCloud, x_range_min:float=-50, x_range_max:float=50, y_range_min:float=-1, y_range_max:float=300, grid_size:float=1) -> o3d.geometry.PointCloud:
    # 1m単位のボクセルに対して1点取るようにしてデータを削減
    # visualize(pcd)
    voxel_pcd = pcd.voxel_down_sample(voxel_size=10.)
    visualize(voxel_pcd)
    
    points = np.asarray(pcd.points)
    downed_points = np.asarray(voxel_pcd.points)
    
    # xy座標抽出
    points_xy = downed_points[:, :2]
    # z座標抽出
    points_z = downed_points[:, 2]
    
    # meshgrid作成
    grid_x, grid_y = np.meshgrid(
        np.arange(x_range_min, x_range_max, grid_size),
        np.arange(y_range_min, y_range_max, grid_size)
    )
    
    # 拡張する点を計算
    new_points = []
    interpolator = my_CT(points_xy, points_z)
    
    for (x_row, y_row) in zip(grid_x, grid_y):
        for (x, y) in zip(x_row, y_row):
            new_points.append([x, y, interpolator(x, y)])
    
    extended_points = np.vstack((points, np.array(new_points)))
    extended_pcd = o3d.geometry.PointCloud()
    extended_pcd.points = o3d.utility.Vector3dVector(extended_points)
    return extended_pcd
   
pcd = generate_point_cloud((-10, 10), (-1, 250), (-0., 0.), 1000, slope=0.5)
# visualize(pcd)

# pcdを拡張
extended_pcd = extend_point_cloud(pcd)
# extended_pcd = extend_point_cloud_voxel(pcd)

visualize(extended_pcd)

# デプスマップを生成して表示