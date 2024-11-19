import numpy as np
# import matplotlib.pyplot as plt

# 定义二阶贝塞尔曲线生成函数
def bezier_curve(t, start_point, control_point, end_point):
    t2 = t * t
    t3 = t2 * t
    mt = 1 - t
    mt2 = mt * mt
    mt3 = mt2 * mt
    
    x = mt3 * start_point[0] + 3 * mt2 * t * control_point[0] + 3 * mt * t2 * control_point[0] + t3 * end_point[0]
    y = mt3 * start_point[1] + 3 * mt2 * t * control_point[1] + 3 * mt * t2 * control_point[1] + t3 * end_point[1]
    
    # x = mt3 * start_point[0] + 2 * mt * t * control_point[0] + t2 * end_point[0]
    # y = mt3 * start_point[1] + 2 * mt * t * control_point[1] + t2 * end_point[1]

    return x, y

# 输入起始点、控制点和结束点
start_point = (100, -400)
end_point = (0, -650)
control_point = (5, -448)

# 生成曲线轨迹的数据点
t = np.linspace(0, 1, 10)
trajectory = [bezier_curve(ti, start_point, control_point, end_point) for ti in t]

# print(trajectory)
# # 提取 x 坐标和 y 坐标
# x_coords, y_coords = zip(*trajectory)

# print(x_coords,y_coords)
# # 绘制曲线轨迹
# plt.plot(x_coords, y_coords)
# plt.scatter([start_point[0], control_point[0], end_point[0]], [start_point[1], control_point[1], end_point[1]], color='red')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Bezier Curve')
# plt.grid(True)
# plt.show()

def Add_path(start_position, x, y, z, rx, ry, rz):
    """
    生成新的路径点
    :param start_position: 当前起始位置
    :param x: 新路径点的X坐标
    :param y: 新路径点的Y坐标
    :param z: 新路径点的Z坐标
    :param rx: 新路径点的RX旋转角度
    :param ry: 新路径点的RY旋转角度
    :param rz: 新路径点的RZ旋转角度
    :return: 新的路径点
    """
    new_path = [x, y, z, rx, ry, rz]
    return new_path