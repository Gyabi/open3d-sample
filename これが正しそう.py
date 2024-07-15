import numpy as np

def angle_between_vectors(v1, v2, axis):
    # ベクトルを正規化
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    axis = axis / np.linalg.norm(axis)
    
    # ベクトルの投影を計算
    v1_proj = v1 - np.dot(v1, axis) * axis
    v2_proj = v2 - np.dot(v2, axis) * axis
    
    # 投影ベクトルを正規化
    v1_proj = v1_proj / np.linalg.norm(v1_proj)
    v2_proj = v2_proj / np.linalg.norm(v2_proj)
    
    # ベクトルの内積を使用して角度を計算
    cos_theta = np.dot(v1_proj, v2_proj)
    angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    
    # 外積を使用して回転方向を判断
    cross_prod = np.cross(v1_proj, v2_proj)
    if np.dot(cross_prod, axis) < 0:
        angle = -angle
    
    return np.degrees(angle)
def calculate_unit_vector(u, v, W, H, fov_h, fov_v):
    # 水平および垂直のtan(FOV/2)を計算
    tan_fov_h_2 = np.tan(np.radians(fov_h) / 2)
    tan_fov_v_2 = np.tan(np.radians(fov_v) / 2)
    
    # 画素のカメラ座標系における位置を計算
    x = (u - W / 2) * 2 * tan_fov_h_2 / W
    z = (v - H / 2) * 2 * tan_fov_v_2 / H
    y = 1  # 奥行き方向の単位長さ
    
    # 単位ベクトルを計算
    vector = np.array([x, y, z])
    unit_vector = vector / np.linalg.norm(vector)
    return unit_vector

# パラメータ
W = 640  # 画像の幅 (ピクセル)
H = 480  # 画像の高さ (ピクセル)
fov_h = 60  # 水平視野角 (度)
fov_v = 45  # 垂直視野角 (度)

# 画素 (u, v) の方向単位ベクトルを計算
u = 320   # 画素のx座標 (例：画像の中央)
v = 240   # 画素のy座標 (例：画像の中央)
unit_vector1 = calculate_unit_vector(u, v, W, H, fov_h, fov_v)

u = 639   # 画素のx座標 (例：画像の中央)
v = 359   # 画素のy座標 (例：画像の中央)
unit_vector2 = calculate_unit_vector(u, v, W, H, fov_h, fov_v)

u = 320   # 画素のx座標 (例：画像の中央)
v = 0   # 画素のy座標 (例：画像の中央)
unit_vector3 = calculate_unit_vector(u, v, W, H, fov_h, fov_v)

deg1 = angle_between_vectors(unit_vector1, unit_vector2, np.array([0,0,1]))
deg2 = angle_between_vectors(unit_vector1, unit_vector3, np.array([1,0,0]))
deg3 = angle_between_vectors(unit_vector1, unit_vector2, np.array([1,0,0]))

print(unit_vector1)
print(unit_vector2)
print(unit_vector3)

print(deg1)
print(deg2)
print(deg3)