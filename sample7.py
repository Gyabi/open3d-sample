import numpy as np

# カメラ内部パラメータ
fov_horizontal_deg = 60.0   # 水平方向の視野角（度）
sensor_size_x = 36.0        # センサーの横幅（例）
sensor_size_y = 24.0        # センサーの縦幅（例）
image_width = 640           # 画像の幅
image_height = 480          # 画像の高さ

# 水平FOVをラジアンに変換
fov_horizontal_rad = np.deg2rad(fov_horizontal_deg)

# 焦点距離の計算
focal_length = sensor_size_x / (2.0 * np.tan(fov_horizontal_rad / 2.0))

# カメラの位置と向き（仮の値）
camera_position = np.array([0, 0, 0])   # カメラの位置
camera_direction = np.array([0, 1, 0])  # カメラの向き（例: z軸方向）

# ピクセル座標（例: 画像の中心にあるピクセル）
pixel_x = image_width // 2
pixel_y = image_height // 2

# ピクセル座標を射影線に変換
normalized_pixel_coords = np.array([
    (pixel_x + 0.5) / image_width,
    (pixel_y + 0.5) / image_height,
    1.0  # ここでは射影点までの距離を1と仮定します
])  # 画像のピクセル座標を[0,1]の範囲に正規化している

# 射影線の方向ベクトルを計算
ray_direction = normalized_pixel_coords - np.array([0.5, 0.5, 0.5])
ray_direction /= np.linalg.norm(ray_direction)  # 正規化

# 射影線の方向ベクトルを表示
print("射影線の方向ベクトル:", ray_direction)
