import numpy as np
import cv2
import os

# ================= 1. 读取 PCD =================
pcd_path = "/home/yyc/RM-/maps/map_3d.pcd"
out_dir = "/home/yyc/RM-/maps"

with open(pcd_path, 'rb') as f:
    while True:
        line = f.readline().decode('utf-8').strip()
        if 'DATA' in line:
            break
    data = np.frombuffer(f.read(), dtype=np.float32)
    points = data.reshape(-1, 8)
    xyz = points[:, :3]

print(f"原始点云数量: {len(xyz)}")

# ================= 2. 去除极端离群值 =================
# 去除 Z 轴上极其离谱的点（比如天上飞的、地底下的）
z_mean = np.mean(xyz[:, 2])
z_std = np.std(xyz[:, 2])
# 保留在 3 个标准差以内的点（过滤掉极少数飞点）
mask_z = (xyz[:, 2] > z_mean - 3 * z_std) & (xyz[:, 2] < z_mean + 3 * z_std)
xyz_filtered = xyz[mask_z]

# ================= 4. 投影到 2D 并画格子 =================
resolution = 0.1
grid_xy = np.floor(xyz_filtered[:, :2] / resolution).astype(int)
unique_grids = np.unique(grid_xy, axis=0)

min_x, min_y = np.min(unique_grids, axis=0)
max_x, max_y = np.max(unique_grids, axis=0)

width = max_x - min_x + 1
height = max_y - min_y + 1

# ⭐ 使用 205 作为空闲底色
map_img = np.full((height, width), 205, dtype=np.uint8)

# 假设 unique_grids 是你的障碍物点集
# 我们只随机抽取 20000 个点来画射线（如果你的点少于2万，就全画）
import random

if len(unique_grids) > 20000:
    # 随机挑 2 万个点画线，剩下的 10 万个点只当做黑色障碍物
    ray_casting_points = unique_grids[np.random.choice(len(unique_grids), 20000, replace=False)]
else:
    ray_casting_points = unique_grids

# ⭐ 补上这两行：计算雷达原点在图片中的像素坐标
origin_x = int((0 - min_x) / resolution)
origin_y = int(height - 1 - (0 - min_y) / resolution)

# 然后再去画线
for x, y in ray_casting_points:
    img_y = int(height - 1 - (y - min_y))
    img_x = int(x - min_x)
    cv2.line(map_img, (origin_x, origin_y), (img_x, img_y), 254, 1)

# 然后用 ray_casting_points 去画线
for x, y in ray_casting_points:
    img_y = int(height - 1 - (y - min_y))
    img_x = int(x - min_x)
    cv2.line(map_img, (origin_x, origin_y), (img_x, img_y), 254, 1)

# 画完射线后，依然用全量的 unique_grids 把黑色的墙画出来
for x, y in unique_grids:
    img_y = int(height - 1 - (y - min_y))
    img_x = int(x - min_x)
    map_img[img_y, img_x] = 0

cv2.imwrite(os.path.join(out_dir, "map.pgm"), map_img)
print("2D地图已保存！")

# ================= 5. 生成 yaml =================
with open(os.path.join(out_dir, "map.yaml"), 'w') as f:
    f.write(f"image: map.pgm\n")
    f.write(f"resolution: {resolution}\n")
    f.write(f"origin: [{min_x * resolution:.4f}, {min_y * resolution:.4f}, 0.0]\n")
    f.write("negate: 0\n")
    f.write("occupied_thresh: 0.65\n")
    f.write("free_thresh: 0.196\n")
print("大功告成！")