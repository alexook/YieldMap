import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

myfont = FontProperties(fname='/usr/share/fonts/WindowsFonts/simsun.ttc', size=14)

with open('path.txt', 'r') as f:
    robot_path = f.readlines()

# 提取机器人路径的x和y坐标
robot_path_x = []
robot_path_y = []
for point in robot_path:
    x, y = map(float, point.strip().split())
    robot_path_x.append(x)
    robot_path_y.append(y)


with open('pointcloud_xy.txt', 'r') as f:
    landmark_points = f.readlines()

# 提取路标点的x和y坐标
landmark_points_x = []
landmark_points_y = []
for point in landmark_points:
    x, y = map(float, point.strip().split())
    landmark_points_x.append(x)
    landmark_points_y.append(y)


plt.rcParams['font.family'] = 'Times New Roman'

fig, ax = plt.subplots()
ax.grid(True, linestyle='-', alpha=0.5)
ax.plot(robot_path_x, robot_path_y, 'g-', label='UAV Path')

# 画路标点
ax.scatter(landmark_points_x, landmark_points_y, s=50, c='r', marker='^', label='Apple Mark')
ax.tick_params(axis='both', labelsize=12)

ax.legend()
ax.set_xlabel('x')
ax.set_ylabel('y')

ax.set_title("作物产量分布图", fontproperties=myfont)
plt.savefig('robot_path_and_landmark_points.png', dpi=800)

