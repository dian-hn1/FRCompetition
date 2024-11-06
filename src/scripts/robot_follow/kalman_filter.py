import numpy as np

# 初始化卡尔曼滤波器的参数
dt = 0.05
F = np.array([[1, 0, 0, dt, 0, 0],
              [0, 1, 0, 0, dt, 0],
              [0, 0, 1, 0, 0, dt],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])
H = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0]])
Q = np.eye(6) * 0.001
R = np.eye(3) * 0.1
x = np.zeros((6, 1))
P = np.eye(6)

# 从传感器获取测量数据
# 这里假设有一个函数 get_sensor_data() 返回当前的测量值
def get_sensor_data():
    # 这是一个模拟函数，实际中应从传感器读取数据
    return np.random.randn(3) + [1, 2, 3]

# 定义卡尔曼滤波器函数
def kalman_filter(measurements,x=x, P=P, F=F, H=H, Q=Q, R=R):
    estimates = []
    for z in measurements:
        z = z[0:3].reshape(3, 1)

        # 预测阶段
        x = np.dot(F, x)
        P = np.dot(np.dot(F, P), F.T) + Q

        # 计算卡尔曼增益
        S = np.dot(H, np.dot(P, H.T)) + R
        K = np.dot(np.dot(P, H.T), np.linalg.inv(S))

        # 更新阶段
        y = z - np.dot(H, x)
        x = x + np.dot(K, y)
        P = P - np.dot(np.dot(K, H), P)

        # 保存估计值
        estimates.append(x[:3].flatten())
    
    return estimates

def kalman_filter_pose(measurements,x=x, P=P, F=F, H=H, Q=Q, R=R):
    estimates = []
    for z in measurements:
        z = z[3:6].reshape(3, 1)

        # 预测阶段
        x = np.dot(F, x)
        P = np.dot(np.dot(F, P), F.T) + Q

        # 计算卡尔曼增益
        S = np.dot(H, np.dot(P, H.T)) + R
        K = np.dot(np.dot(P, H.T), np.linalg.inv(S))

        # 更新阶段
        y = z - np.dot(H, x)
        x = x + np.dot(K, y)
        P = P - np.dot(np.dot(K, H), P)

        # 保存估计值
        estimates.append(x[:3].flatten())
    
    return estimates


# 运行卡尔曼滤波器，假设有4个时间步长
measurements = np.array([get_sensor_data() for _ in range(4)])
print(f"测量值: {measurements}")
estimates = kalman_filter(measurements)
print(f"估计值: {estimates}")

# 打印结果
for i, estimate in enumerate(estimates):
    print(f"测量值: {measurements[i]}, 估计值: {estimate}")
