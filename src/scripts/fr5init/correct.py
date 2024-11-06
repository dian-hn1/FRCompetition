import numpy as np
from sklearn.linear_model import LinearRegression
import joblib
import matplotlib.pyplot as plt

# 从文本文件中加载数据
data = np.loadtxt('data.txt', delimiter=' ', dtype=float)

# 提取预测值和真实值
xy_pred = data[:, 2:4]  # shape: (36, 2)
xy_true = data[:, 0:2]  # shape: (36, 2)

# 线性回归模型
# model = LinearRegression()

# 加载模型
model = joblib.load('model.pkl')

# 训练模型
# model.fit(xy_pred, xy_true)

# 保存模型
# joblib.dump(model, 'model.pkl')

# 预测校正后的值
xy_pred_corrected = model.predict([[211,-816]])


print("机器的预测值：")
print(xy_pred)
print("校正后的预测值：")
print(xy_pred_corrected)

# # 获取真实值和预测值
# x_true = data[:, 0]
# y_true = data[:, 1]
# x_pred = data[:, 2]
# y_pred = data[:, 3]

# # 绘制散点图
# plt.scatter(x_true, y_true, label='True')
# plt.scatter(x_pred, y_pred, label='Predicted')
# plt.scatter(xy_pred_corrected[0][0], xy_pred_corrected[0][1], label='Corrected')
# # 设置图例、x轴和y轴标签
# plt.legend()
# plt.xlabel('X')
# plt.ylabel('Y')

# # 显示图像
# plt.show()