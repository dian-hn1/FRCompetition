'''
YoloV8目标检测+定位
化学仪器
'''
import os
import time
import numpy as np
import cv2
# 阿凯机器人工具箱
from kyle_robot_toolbox.camera import Gemini2
from kyle_robot_toolbox.yolov8 import YoloV8Detect

import socket
import json

import numpy as np


def transform_coordinates(coords):
	# 仿射变换矩阵
	affine_matrix = np.array([
		[-1.62966442e-18, -1.01522843e-03, 7.95431472e-01],
		[-9.90099010e-04, -1.00517666e-05, -1.17867015e-01],
		[0, 0, 1]
	])

	# 将输入坐标转换为齐次坐标（x, y, 1）
	coords_homogeneous = np.append(coords, 1)  # 添加1使其成为齐次坐标

	# 应用仿射变换
	transformed_coords = affine_matrix @ coords_homogeneous  # 使用矩阵乘法进行变换

	# 返回变换后的坐标（不包括齐次坐标的最后一位）
	return transformed_coords[:2]

udp_ip = "127.0.0.1"  # 目标IP地址
udp_port = 12346  # 目标端口
sock = socket.socket(socket.AF_INET,  # Internet
					 socket.SOCK_DGRAM)  # UDP

# 创建相机对象
camera = Gemini2()
# 模型初始化
print("[INFO] 开始YoloV8模型加载")
# 模型路径
model_path = os.path.join(os.path.abspath("."), "weights", "/home/wangzy/FR5_exp/scripts/models/yiqi6.pt")
print(f"模型路径: {model_path}")
# 载入目标检测模型(使用绝对路径)
model = YoloV8Detect(model_path)
# 配置模型参数(可选)
# - 图像尺寸(必须是32的倍数)
model.IMAGE_SIZE= 1088
# - 置信度
model.CONFIDENCE = 0.8
# - IOU 
model.IOU = 0.8
print("[INFO] 完成YoloV8模型加载")

# 创建窗口
cv2.namedWindow('canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
#cv2.namedWindow('depth_canvas', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)





while True:
	# 开始时间
	try:
		t_start = time.time()

		# 读取图像
		# 读取彩图
		img_bgr = camera.read_color_img()
		# 采集深度图, 单位为mm
		# 数据格式为np.float32
		depth_img = camera.read_depth_img()
		# 将深度转换为可视化画布
		# 根据实际情况调整深度范围 [min_distance, max_distance]

		depth_canvas_tmp = camera.depth_img2canvas(depth_img)
		# 为了兼容Gemini深度图与彩图尺寸不一致的情况
		# 要做一下特殊处理
		dp_h, dp_w, dp_ch = depth_canvas_tmp.shape
		depth_canvas = np.zeros_like(img_bgr)
		if img_bgr is None:
			print("颜色图获取失败")
			continue
		depth_canvas[:dp_h, :dp_w] = depth_canvas_tmp
		# YoloV8 目标检测
		canvas, class_id_list, xyxy_list, conf_list = model.detect(img_bgr, \
													draw_label=False)
		center_list = []
		send_list = []

		# 不同类型对应的偏移值
		offset_dict = [0, 20, 18]

		for xyxy in xyxy_list:
			# 获取类型
			class_id = class_id_list[xyxy_list.index(xyxy)]
			# 读取矩形框
			x1, y1, x2, y2 = xyxy
			# 计算中心点
			cx = int((x1 + x2)/2)
			cy = int((y1 + y2)/2)

			if class_id == 0:
			# 按照中心点在图片的位置，调整中心点的位置
				offsetx = (630 - cx) / 630
				offsety = (360 - cy) / 360
				cx = cx + int((x2 - x1) * offsetx / 1.5)
				cy = cy + int((y2 - y1) * offsety / 2.5)

			elif class_id == 1:
				# 计算底部中心点，先判断在第几象限，如果在第一象限，坐标为左下角加上一定值......图像中心为630, 360
				# 先求偏离绝对值，如果偏离过小，就不进行偏移
				if abs(cx - 630) > 20:
					if cx > 630 and cy < 360:
						cx = x1 + offset_dict[class_id]
					elif cx < 630 and cy < 360:
						cx = x2 - offset_dict[class_id]
					elif cx < 630 and cy > 360:
						cx = x2 - offset_dict[class_id]
					elif cx > 630 and cy > 360:
						cx = x1 + offset_dict[class_id]
				if abs(cy - 360) > 20:
					if cx > 630 and cy < 360:
						cy = y2 - offset_dict[class_id]
					elif cx < 630 and cy < 360:
						cy = y2 - offset_dict[class_id]
					elif cx < 630 and cy > 360:
						cy = y1 + offset_dict[class_id]
					elif cx > 630 and cy > 360:
						cy = y1 + offset_dict[class_id]
			else:
				# 计算底部中心点，先判断在第几象限，如果在第一象限，坐标为左下角加上一定值......图像中心为630, 360
				if cx > 630 and cy < 360:
					cx = x1 + offset_dict[class_id]
					cy = y2 - offset_dict[class_id]
				elif cx < 630 and cy < 360:
					cx = x2 - offset_dict[class_id]
					cy = y2 - offset_dict[class_id]
				elif cx < 630 and cy > 360:
					cx = x2 - offset_dict[class_id]
					cy = y1 + offset_dict[class_id]
				elif cx > 630 and cy > 360:
					cx = x1 + offset_dict[class_id]
					cy = y1 + offset_dict[class_id]

			# 绘制中心点
			cv2.circle(canvas, [cx, cy], 5, (255, 255, 0), -1)
			cv2.circle(depth_canvas, [cx, cy], 5, (255, 0, 255), -1)


			center = [cx, cy]
			center_list.append(center)

			px, py = center
			# 判断坐标是否在深度图的有效范围内
			if px >= dp_w or py >= dp_h:
				continue
			# 读取深度值
			depth_value = depth_img[py, px]
			# 深度值无效
			cam_point3d = camera.depth_pixel2cam_point3d(\
											px, py, depth_value=depth_value)
			# 计算三维坐标
			cam_x, cam_y, cam_z = cam_point3d
			send_x, send_y = transform_coordinates([cam_x, cam_y])
			# 在画面上绘制坐标
			tag = f"{send_x:.2f}, {send_y:.2f}, {cam_z:.0f}"
			cv2.putText(canvas, text=tag,\
				org=(px-80, py-60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
				fontScale=0.8, thickness=1, lineType=cv2.LINE_AA, color=(255, 0, 255))
			send_list.append([class_id, send_x, send_y])
			print(f"send_list: {send_list}")

		# 统计帧率
		t_end = time.time()
		t_pass = t_end - t_start
		fps = int(1/t_pass)

		# 绘制帧率
		cv2.putText(canvas, text=f"FPS:{fps}",\
			org=(20, 20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, \
			fontScale=0.8, thickness=2, lineType=cv2.LINE_AA, color=(0, 0, 255))

		cv2.imshow("canvas", canvas)
		#cv2.imshow("depth_canvas", depth_canvas)

		# 发送数据
		send_data = json.dumps(send_list)
		sock.sendto(send_data.encode(), (udp_ip, udp_port))


		key = cv2.waitKey(1)
		if key == ord('q'):
			break
	except:
		print("Error")
		continue

cv2.destroyAllWindows()
camera.release()


