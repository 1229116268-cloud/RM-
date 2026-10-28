import numpy as np
import cv2
import os

# ================= 1. 读取 PCD =================
# 请确认路径是否正确
pcd_path = "/home/yyc/RM-/maps/map_3d.pcd"
out_dir = "/home/yyc/RM-/maps"

print("正在读取 PCD 文件...")
with open(pcd_path, 'rb') as f:
    while True:
        line = f.readline().decode('utf-8').strip()
        if 'DATA' in line:
            break
    # 读取二进制数据，假设每个点32字节 (8个float32)
    data = np.frombuffer(f.read(), dtype=np.float32)
    points = data.reshape(-1, 8)
    xyz = points[:, :3]

print(f"成功读取点云！总点数: {len(xyz)}")

# ================= 2. 去除极端离群值 =================
z_mean = np.mean(xyz[:, 2])
z_std = np.std(xyz[:, 2])
# 只去飞点，绝对不切地面！
mask_z = (xyz[:, 2] > z_mean - 3 * z_std) & (xyz[:, 2] < z_mean + 3 * z_std)
xyz_filtered = xyz[mask_z]

# ================= 3. 投影到 2D 并画格子 =================
resolution = 0.1
# 把连续坐标变成格子编号
grid_xy = np.floor(xyz_filtered[:, :2] / resolution).astype(int)
# 去重，防止重叠点
unique_grids = np.unique(grid_xy, axis=0)

# 计算图片边界
min_x, min_y = np.min(unique_grids, axis=0)
max_x, max_y = np.max(unique_grids, axis=0)

width = max_x - min_x + 1
height = max_y - min_y + 1

# ⭐ 关键：使用 205 作为底色（未知区域）
map_img = np.full((height, width), 205, dtype=np.uint8)

# ================= 4. 把障碍物涂黑 =================
for x, y in unique_grids:
    # 图像Y轴是向下的，所以要翻转一下
    img_y = int(height - 1 - (y - min_y))
    img_x = int(x - min_x)
    map_img[img_y, img_x] = 0 # 0 代表占用/障碍物

# 保存图片
pgm_path = os.path.join(out_dir, "map.pgm")
cv2.imwrite(pgm_path, map_img)
print(f"2D地图已保存至: {pgm_path}")

# ================= 5. 生成 yaml =================
yaml_path = os.path.join(out_dir, "map.yaml")
with open(yaml_path, 'w') as f:
    f.write(f"image: map.pgm\n")
    f.write(f"resolution: {resolution}\n")
    # 计算真实世界原点
    f.write(f"origin: [{min_x * resolution:.4f}, {min_y * resolution:.4f}, 0.0]\n")
    f.write("negate: 0\n")
    f.write("occupied_thresh: 0.65\n")
    f.write("free_thresh: 0.196\n")

print("✅ 大功告成！")