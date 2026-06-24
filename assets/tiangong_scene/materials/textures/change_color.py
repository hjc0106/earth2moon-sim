import cv2
import numpy as np

def change_color(image_path, new_color_rgba, output_path):
    # 读取图像
    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

    # 检查图像是否包含alpha通道
    if image.shape[2] == 4:
        # 分离alpha通道
        b, g, r, a = cv2.split(image)
    else:
        # 如果没有alpha通道，创建一个全白的alpha通道
        b, g, r = cv2.split(image)
        a = np.ones(b.shape, dtype=b.dtype) * 255

    # 将颜色转换为0-255范围
    new_color = [int(c * 255) for c in new_color_rgba]

    # 创建新的颜色图层
    color_layer = np.zeros_like(image, dtype=np.uint8)
    color_layer[:, :, 0] = new_color[2]  # B
    color_layer[:, :, 1] = new_color[1]  # G
    color_layer[:, :, 2] = new_color[0]  # R
    color_layer[:, :, 3] = new_color[3]  # A

    # 替换图像颜色
    output_image = cv2.merge([color_layer[:, :, 0], color_layer[:, :, 1], color_layer[:, :, 2], a])

    # 保存图像
    cv2.imwrite(output_path, output_image)

# 修改base_texture.png的颜色
change_color('base_texture.png', (0.913725, 0.913725, 0.847059, 1.0), 'base_texture.png')

# 修改wheel_texture.png的颜色
change_color('wheel_texture.png', (0.213725, 0.213725, 0.247059, 1.0), 'wheel_texture.png')

