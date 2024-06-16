# 2次元の点群データを1次関数に近似し、X軸からの角度を求めるコード例
import open3d as o3d
import numpy as np
import point_cloud_utils as pcu
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.widgets import Button
import math

def create_wave_slope_point_cloud(width, length, amplitude, frequency, resolution=100):
    """
    遠ざかるにつれて下がって上がるような勾配のある平面のポイントクラウドを生成する関数。

    Parameters:
    - width: 平面の幅
    - length: 平面の長さ
    - amplitude: 波の振幅
    - frequency: 波の周波数
    - resolution: ポイントクラウドの解像度（デフォルトは100）

    Returns:
    - point_cloud: Open3Dのポイントクラウドオブジェクト
    """
    # x, y座標の範囲
    x_range = np.linspace(-width / 2, width / 2, 1)
    y_range = np.linspace(-length / 2, length / 2, resolution)

    # グリッドを作成
    x_grid, y_grid = np.meshgrid(x_range, y_range)
    # サイン波を適用してz座標を計算
    z_grid = amplitude * np.sin(frequency * y_grid)

    # 点群を作成
    points = np.vstack((x_grid.flatten(), y_grid.flatten(), z_grid.flatten())).T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    return point_cloud
def generate_point_cloud(x_range:tuple, y_range:tuple, z_range:tuple, num_points:int, slope:float=0) -> o3d.geometry.PointCloud:
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    z = np.random.uniform(z_range[0], z_range[1], num_points) + slope * y
    
    points = np.vstack((x, y, z)).T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    return point_cloud

def visualize(target):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True)
    vis.get_render_option().background_color = [0,0,1]
    vis.add_geometry(target)
    
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0,0,0])
    vis.add_geometry(axis)
    
    vis.run()
    vis.destroy_window()
    
def divide_range(min_val, max_val, num_divisions):
    num_divisions_with_endpoints = num_divisions +1 
    values = np.linspace(min_val, max_val, num_divisions_with_endpoints)
    return values[1:-1].tolist()


def show_deg_input_ui(x_datas:np.ndarray,y_datas:np.ndarray):
    points = []
    lines = []
    angle_degrees = []
    
    # クリック時のイベント処理
    def onclick(event):
        if event.button == 1:
            points.append((event.xdata, event.ydata))
            if len(points) == 2:
                draw_line()
                fig.canvas.draw()

    # 線分を描画する関数
    def draw_line():
        line = Line2D([points[0][0], points[1][0]], [points[0][1], points[1][1]], color='r')
        lines.append(line)
        ax.add_line(line)

    # 決定ボタンが押された時の処理
    def on_button_click(event):
        slope = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
        angle_degrees.append(np.degrees(np.arctan(slope)))
        
        plt.close()

    fig, ax = plt.subplots()
    ax.set_xlim(-50, 50)
    ax.set_ylim(-100, 100)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    
    # x,yをプロット
    ax.scatter(x_datas, y_datas, color='b', label='Original Data')
    
    # 決定ボタンの作成
    button_ax = plt.axes([0.8, 0.05, 0.1, 0.075])
    button = Button(button_ax, 'Submit')
    button.on_clicked(on_button_click)

    plt.show()
    
    return angle_degrees[0]

def rotate_pcd_z(pcd:o3d.geometry.PointCloud, angle:float) -> o3d.geometry.PointCloud:
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(pcd.points)
    R = mesh.get_rotation_matrix_from_xyz((0,0,np.radians(angle)))
    mesh.rotate(R, center=(0, 0, 0))
    pcd.points = o3d.utility.Vector3dVector(mesh.vertices)
    return pcd
    
def extend_points(pcd:o3d.geometry.PointCloud, extend_distance_y:float=1000, extend_distance_x:float=1000) -> np.ndarray:
    """_summary_

    Args:
        pcd (o3d.geometry.PointCloud): _description_
        extend_distance (float, optional): +-m分の路面を作成する Defaults to 300.

    Returns:
        o3d.geometry.PointCloud: _description_
    """
    points = np.asarray(pcd.points)
    x = points[:,0]
    y = points[:,1]
    
    
    # 角度を入力するためのUIを生成し、角度取得
    angle = show_deg_input_ui(x,y)
    
    # 取得した角度方向に回転
    rotated_pcd = rotate_pcd_z(pcd, -angle+90)
    
    points = np.asarray(rotated_pcd.points)
    
    # 奥行方向の拡張
    # Yが最大値をとっているpointsを取り出す
    point_y_max = points[np.where(points[:, 1] == np.max(points[:, 1]))]
    # Yが最小値をとっているpointsを取り出す
    point_y_min = points[np.where(points[:, 1] == np.min(points[:, 1]))]
    
    # ±extended_distanceの分だけデータが欠けているところに1mおきにY軸上のデータを追加していく
    for i in range(int(point_y_max[0][1]), extend_distance_y, 1):
        # pointsへ追加
        points = np.append(points, [[0, i, point_y_max[0][2]]], axis=0)
    for i in range(int(point_y_min[0][1]), -extend_distance_y, -1):
        points = np.append(points, [[0, i, point_y_min[0][2]]], axis=0)
        
    # 一度可視化
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(points)
    # visualize(p)
    
    # 垂直方向の拡張(1m単位で拡張していく)
    # 計算用にpointsをコピー
    points_copy = points.copy()
    # pointsのxをゼロに入れ替えたもの
    points_copy[:, 0] = 0
    # points_copyをyの降順にソート
    points_copy = points_copy[points_copy[:, 1].argsort()[::-1]]
    # 1m単位でX軸方向にグリッドを入れる想定
    output_points = []
    # points_copyを逆向きにループ
    for p in points_copy:
        row = []
        for x in range(-extend_distance_x, extend_distance_x, 100):
            row.append([x, p[1], p[2]])
        output_points.append(row)
    
    output_points = np.array(output_points)
    # 形状を変数にメモ
    output_points_shape = output_points.shape
    
    # 再度データを回転させpcdに格納
    output = o3d.geometry.PointCloud()
    output.points = o3d.utility.Vector3dVector(output_points.reshape(-1, 3))
    output = rotate_pcd_z(output, -(-angle+90))
    
    # visualize(output)
    
    output_points = np.asarray(output.points).reshape(output_points_shape)
    print(output_points_shape)
    print(output_points.shape)
    
    # 返却
    return output_points
    
def convert_mesh(grid_points:np.ndarray):
    mesh = o3d.geometry.TriangleMesh()
    
    vertices = grid_points.reshape(-1,3)
    
    def calc_hw_to_vertices_idx(current_h, current_w, full_h, full_w):
        return int(full_w * current_h + current_w)
    
    triangles = []
    height = grid_points.shape[0]
    width = grid_points.shape[1]
    
    for h in range(height):
        if h != height-1:
            for w in range(width):
                if w != width-1:
                    triangles.append([calc_hw_to_vertices_idx(h,w,height,width),  calc_hw_to_vertices_idx(h+1,w,height,width),calc_hw_to_vertices_idx(h+1,w+1,height,width)])
                    triangles.append([calc_hw_to_vertices_idx(h,w,height,width), calc_hw_to_vertices_idx(h+1,w+1,height,width), calc_hw_to_vertices_idx(h,w+1,height,width)])
                    # triangles.append([calc_hw_to_vertices_idx(h,w,height,width), calc_hw_to_vertices_idx(h+1,w+1,height,width), calc_hw_to_vertices_idx(h+1,w,height,width)])
                    # triangles.append([calc_hw_to_vertices_idx(h,w,height,width), calc_hw_to_vertices_idx(h,w+1,height,width), calc_hw_to_vertices_idx(h+1,w+1,height,width)])
    
    triangles = np.array(triangles)
    
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    
    mesh.compute_vertex_normals()
    return mesh

def rotate_vector_around_axis(vector: np.ndarray, axis_str: str, angle_degrees: float) -> np.ndarray:
    """
    Rotate a vector around an axis by a specified angle in degrees.

    Parameters:
    - vector: The vector to rotate.
    - axis_str: The axis around which to rotate the vector specified as 'x', 'y', or 'z'.
    - angle_degrees: The angle in degrees by which to rotate the vector.

    Returns:
    - rotated_vector: The vector after rotation.
    """
    axis_dict = {'x': np.array([1, 0, 0]),
                 'y': np.array([0, 1, 0]),
                 'z': np.array([0, 0, 1])}
    
    axis = axis_dict.get(axis_str.lower(), np.array([1, 0, 0]))
    
    angle_radians = np.radians(angle_degrees)
    rotation_matrix = np.array([[np.cos(angle_radians) + axis[0]**2 * (1 - np.cos(angle_radians)),
                                 axis[0] * axis[1] * (1 - np.cos(angle_radians)) - axis[2] * np.sin(angle_radians),
                                 axis[0] * axis[2] * (1 - np.cos(angle_radians)) + axis[1] * np.sin(angle_radians)],
                                [axis[1] * axis[0] * (1 - np.cos(angle_radians)) + axis[2] * np.sin(angle_radians),
                                 np.cos(angle_radians) + axis[1]**2 * (1 - np.cos(angle_radians)),
                                 axis[1] * axis[2] * (1 - np.cos(angle_radians)) - axis[0] * np.sin(angle_radians)],
                                [axis[2] * axis[0] * (1 - np.cos(angle_radians)) - axis[1] * np.sin(angle_radians),
                                 axis[2] * axis[1] * (1 - np.cos(angle_radians)) + axis[0] * np.sin(angle_radians),
                                 np.cos(angle_radians) + axis[2]**2 * (1 - np.cos(angle_radians))]])
    
    rotated_vector = np.dot(rotation_matrix, vector)
    
    return rotated_vector

def calc_pinhole_raycast(mesh):
    h_fov = 60
    width = 640
    height = 480
    
    tilt = 30
    
    center = rotate_vector_around_axis([0.,1.,0.], 'x', -tilt)
    eye = [0.,0.,0.]
    up = rotate_vector_around_axis([0.,0.,1.], 'x', -tilt)
    
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=h_fov,
        width_px=width,
        height_px=height,
        center=center,
        eye=eye,
        up=up
    )
    
    # シーンの作成
    scene = o3d.t.geometry.RaycastingScene()
    # meshをo3d.tに変換
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene.add_triangles(mesh_t)
    
    # レイキャスト
    result = scene.cast_rays(rays)
    
    hit = result["t_hit"].isfinite()
    points = rays[hit][:,:3] + rays[hit][:,3:]*result["t_hit"][hit].reshape((-1,1))
    pcd = o3d.t.geometry.PointCloud(points)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0,0,0])

    o3d.visualization.draw_geometries([pcd.to_legacy(), mesh, axis])

# main2を踏襲してmeshを生成する
pcd = generate_point_cloud((-1,1), (-1, 250), (-5, -5), 1000, 0)
# pcd = generate_point_cloud((1,1), (-1, 250), (-5, -5), 1000, 0.5)
    
# 点群データを指向基準で拡張
grid_points = extend_points(pcd)
    
# グリッド点群からメッシュ作成
mesh = convert_mesh(grid_points)
# visualize(mesh)

# 生成したメッシュに対して、原点からピンホールのraycastを計算して飛ばす
calc_pinhole_raycast(mesh)

# 結果を3d表示する