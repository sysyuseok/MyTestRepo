import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import apriltag
import cv2
import os
import math

class AprilTagPublisherNode(Node):
	def __init__(self):
		super().__init__('april_tag_publisher')
		self.tag_detected = False

		# Publisher 설정
		self.id_subscription = self.create_subscription(
				Int32, '/id_value', self.id_value_callback, 10)
		# self.pose_publisher = self.create_publisher(Pose, '/april_tag_pose', 10)
		self.dir_publisher = self.create_publisher(
				Float32MultiArray, '/goal_moving', 10)
		# 구독자 설정: image_raw 또는 gray 토픽을 받음
		self.image_sub = self.create_subscription(
				Image,
				'/raw_image/gray',  # 또는 'gray' 토픽을 사용할 수 있습니다.
				self.image_callback,
				10
		)
		# CvBridge 초기화 (ROS와 OpenCV 간 이미지 변환)
		self.bridge = CvBridge()

		# AprilTag 탐지기 설정
		self.detector = apriltag.Detector()
		
		# 카메라 메트릭스 및 왜곡 계수 로드
		#self.matrix_coefficients = np.load('/home/cowin-ys/work_space/cowin_bot_ws/src/vision/vision/calibration_value/camera_matrix.npy')
		#self.distortion_coefficients = np.load('/home/cowin-ys/work_space/cowin_bot_ws/src/vision/vision/calibration_value/dist_coefficients.npy')
		self.matrix_coefficients = np.load('/home/cowin/cowin_ws/src/april_parking/april_parking/calibration_value1080_720/camera_matrix.npy')
		self.distortion_coefficients = np.load('/home/cowin/cowin_ws/src/april_parking/april_parking/calibration_value1080_720/dist_coefficients.npy')

		# Tag 크기
		tag_size = 0.031  # Tag size in meters
		self.tag_3d_points = np.array([
				[-tag_size / 2, -tag_size / 2, 0],
				[tag_size / 2, -tag_size / 2, 0],
				[tag_size / 2, tag_size / 2, 0],
				[-tag_size / 2, tag_size / 2, 0]
		], dtype=np.float32)

		
		
		# 수신된 ID 저장
		self.received_id = None
		
		# 수신한 이미지 저장
		self.gray_image = None
		
		self.pub_switch = True

		self.offset = 0.031
		self.parking_signal = 3
		
		self.translation_vectors=[]
		self.rotation_vectors=[]
		
		self.get_logger().info("find_path_newlogic.py released!")
		
		
			

	def id_value_callback(self, msg):
		# 수신된 ID 저장
		self.received_id = msg.data
		# self.get_logger().info(f'수신된 마커 ID: {self.received_id}')
		self.pub_switch = True

	def image_callback(self, msg):
		try:
			# ROS 이미지 메시지를 OpenCV 형식으로 변환
			frame = self.bridge.imgmsg_to_cv2(msg, 'mono8')
			#resize
			#frame = cv2.resize(frame,(1080,720))
			
			if self.tag_detected:
				return
			
			# AprilTag 탐지
			detections = self.detector.detect(frame)

			for detection in detections:
				if detection.tag_id== self.parking_signal :
					#self.tag_detected = True
					#detection = detections[0]
					corners = detection.corners.astype(int)
					top_left, top_right, bottom_right, bottom_left = corners
					image_points = np.array(
							[top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
					# self.get_logger().info(f"coordinates\n{top_left}   {top_right} \n{bottom_left}   {bottom_right}\n")
					
					# SolvePnP를 사용하여 회전 벡터와 이동 벡터 추정
					success, rotation_vector, translation_vector = cv2.solvePnP(
							self.tag_3d_points, image_points, self.matrix_coefficients, self.distortion_coefficients)
					slope = abs(math.tan(rotation_vector[1])) # 기울기
					
					if success:
						self.translation_vectors.append(translation_vector)
						self.rotation_vectors.append(rotation_vector)
						
						if len(self.translation_vectors) >= 5:
							# 메시지 발행
							os.system('cls' if os.name == 'nt' else 'clear')
							median_translation_vectors = np.median(self.translation_vectors, axis=0)
							median_rotation_vectors = np.median(self.rotation_vectors,axis=0)
							
							x=median_translation_vectors[0]
							z=median_translation_vectors[2]
							ry=median_rotation_vectors[1]
							
							print(math.degrees(ry))
							pose_msg = Float32MultiArray()
							pose_msg.data = [0.0] * 4
							if ry < 0:
								if (x+z*slope) > self.offset:
									print("case 1")
									new_x = float(((slope**2)*self.offset + x + z*slope)/(slope**2+1))
									new_z = float(slope*((x+z*slope-self.offset)/(slope**2+1)))
									pose_msg.data[0] = float(abs(ry))
									pose_msg.data[1]= float(math.sqrt((new_x-self.offset)**2+new_z**2))
									pose_msg.data[2] = float(math.radians(90.0))
									pose_msg.data[3]=float((z-new_z)*math.sqrt(1+slope**2)-0.15)
								else:
									print("case 2")
									moving_distance = self.offset - x - (z * slope)
									pose_msg.data[0] = float(math.radians(90.0))
									pose_msg.data[1] = float(moving_distance-0.055*math.sin(ry))
									pose_msg.data[2] = float(math.radians(-90.0) + abs(rotation_vector[1]))
									pose_msg.data[3] = float((z+0.055*(1-math.cos(abs(ry)))) * math.sqrt(1 +(slope ** 2))-0.15)
								
							else:
								if (x-z*slope) > self.offset :
									print("case 3")
									moving_distance = x - self.offset - (z * slope)
									pose_msg.data[0] = float(math.radians(-90.0))
									pose_msg.data[1] = float(moving_distance-0.055*math.sin(ry))
									pose_msg.data[2] = float(math.radians(90.0) - abs(ry))
									pose_msg.data[3] = float((z+0.055*(1-math.cos(abs(ry)))) * math.sqrt(1 +(slope ** 2))-0.15)
								else:
									print("case 4")
									new_x = float(((slope**2)*self.offset - x - z*slope)/(slope**2+1))
									new_z = float(slope*((-x-z*slope-self.offset*(1+2*(slope**2)))/(slope**2+1)))
									moving_distance = self.offset - x + (z * slope)
									pose_msg.data[0] = float(math.radians(90.0))
									pose_msg.data[1]= float(math.sqrt((new_x+self.offset)**2+new_z**2))
									pose_msg.data[2] = float(-math.radians(90.0) - abs(ry))
									pose_msg.data[3]=float((z-new_z)*math.sqrt(1+slope**2)-0.15)
                  
							
					
					
					#pose_msg.data[3] = float(translation_vector[2] * math.sqrt(1 +(slope ** 2))+self.delta_z)
					for i in range(4):
						print(f"data {i} : {pose_msg.data[i]}")
					
					self.dir_publisher.publish(pose_msg)
					
					# Initialize the vectors
					self.translation_vectors=[]
					self.rotation_vectors=[]
					
					print( f"\nPublished ID: {detection.tag_id}, \nPosition:")
					print(f"{translation_vector[0]},\n {translation_vector[1]},\n {translation_vector[2]}")
					print(f"Rotation:\n {rotation_vector[0]},\n {rotation_vector[1]},\n {rotation_vector[2]}")
		except Exception as e:
			self.get_logger().error(f"Error in processing image: {e}")


def main(args=None):
	rclpy.init(args=args)

	april_tag_publisher_node = AprilTagPublisherNode()

	rclpy.spin(april_tag_publisher_node)

	rclpy.shutdown()


if __name__ == '__main__':
	main()
