import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

# 读取并转换为灰度图的函数
def load_and_convert_image(file_path):
    # 从文件中读取图片
    img = Image.open(file_path)
    
    # 检查是否为 RGB 图片
    if img.mode != 'RGB':
        raise ValueError("The image must be an RGB image.")
    
    # 检查图片尺寸是否为 200x200
    if img.size != (200, 200):
        raise ValueError("The image must be 200x200 pixels.")
    
    # 将 RGB 图片转换为灰度图片
    gray_img = img.convert('L')
    
    # 转换为 NumPy 数组
    gray_array = np.array(gray_img)
    
    return gray_array

# 示例：从文件读取并转换为灰度图片
file_path = 'suz_ama01a.jpg'  # 替换为你的图片文件路径
pic = load_and_convert_image(file_path)

# 假设 bw 是二值化处理后的图像数据，或者可以直接使用 pic
bw = pic.copy()

# 初始化用于保存转换结果的数组
new = np.zeros_like(bw)

# 检查图像尺寸是否正确
if pic.shape != (200, 200):
    raise ValueError("The image must be a 200x200 grayscale image.")

# 打开并写入到文件
with open('map_new.txt', 'w') as file:
    file.write('uint8_t bitmap_new[20000] = {')
    for ii in range(pic.shape[0]):
        for jj in range(pic.shape[1]):
            if jj % 2 == 0:
                file.write('0x')
            
            pix = bw[ii, jj]
            if pix >= 255 * 0.75:
                file.write('F')
                new[ii, jj] = 3
            elif pix >= 255 * 0.6:
                file.write('B')
                new[ii, jj] = 2
            elif pix >= 255 * 0.3:
                file.write('8')
                new[ii, jj] = 1
            else:
                file.write('0')
                new[ii, jj] = 0
            
            if jj % 2 == 1:
                file.write(',')
        
        file.write('\n')
    file.write('};')
# 使用 Matplotlib 显示图像
plt.imshow(new, cmap='gray')
plt.show()
