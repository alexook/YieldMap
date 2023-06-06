import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 假设你有一个N*3的numpy数组points，表示点云的坐标
points = np.random.rand(100, 3)

# 创建一个3D绘图对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 画出点云
ax.scatter(points[:, 0], points[:, 1], points[:, 2])

# 画出坐标轴
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()

