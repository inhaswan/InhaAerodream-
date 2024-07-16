#!/usr/bin/env python3
############################################################################
#
# point cloud data 전처리 및 클러스터링 진행 노드
# point cloud data를 토픽으로 받아온 후
# 전처리 및 클러스터링 해서 그걸 다시 pub 하는 노드
# 전처리: 다운샘플링, 노이즈 제거(아웃라이어 제거), 평면 제거(RANSAC)
# 클러스터링: dbscan 알고리즘 사용 + bounding box
# 선택 사항(1): 클러스터에 색깔을 입힐 수 있음, line 95~98
# 선택 사항(2): bounding box의 중심점, 체적, 코너점들 추출 가능, line 100 ~ 117
# 클러스터의 중심점, bounding box의 최외각점 산출
#
############################################################################

import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np
import time

class PointCloudProcessing(Node):
    def __init__(self):
        super().__init__("point_cloud_processing")
        self.get_logger().info("Under Initializing ...")

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST,
                                 depth=1)
        # px4랑 통신하기 위해서 임의로 맞춰줌

        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.point_cloud_callback,
            10
        )
        # self.mapping_publisher = self.create_publisher()
        # mapping을 하기 위해서 클러스터들과 bounding box 정보를 보내주는 publisher

        self.farthest_corner_publisher = self.create_publisher(
            Point, 'farthest_corner', qos_profile)
        # 최외각점을 pub하기 위한 publisher 선언
        self.center_point_publisher = self.create_publisher(
            Point, 'center_point', qos_profile)
        # 중심점을 pub하기 위한 publisher 선언
        self.distance_publisher = self.create_publisher(
            Float64, 'distance', qos_profile)
        # 최외각점과 중심점 사이의 거리(최대 거리)를 pub하기 위한 publisher 선언

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Open3D") # 실시간 시각화 객체 생성

        self.pcd = o3d.geometry.PointCloud() # 포인트 클라우드 객체 생성
        self.farthest_corner_msg = Point() # 최외각점을 geometry_msgs의 Point 메시지 객체로 선언
        self.center_point_msg = Point() # 중심점을 geometry_msgs의 Point 메시지 객체로 선언
        self.distance_msg = Float64() # 거리를 std_msgs의 Float64 메시지 객체로 선언

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.vis.destroy_window() # __enter__와 __exit__는 메모리 사용량 감소를 위하 추가한 함수(큰 의미X)

    def point_cloud_callback(self, msg):
        if not isinstance(msg, PointCloud2):
            self.get_logger().error(f"Type of msg is not PointCloud2: {type(msg)}")
            # 만약 들어오는 msg가 PointCloud2 토픽이 아니라면, 오류 발생
            return

        points = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        # 포인트 클라우드 데이터 정보 중 x,y,z만 추출

        if len(points) == 0:
            self.get_logger().info("Received Empty Point Cloud, Waiting for next message..")
            return

        filtered_points = [p for p in points if 0.3 <= p[2] <= 6]
        # gazebo 시뮬레이션 상으로 이미지 평면이 불필요하게 나오기 때문에,
        # depth camera 최대 탐지범위를 D455보다 길게 잡아놓고, 이미지 좌표계상 z축으로 6m까지만 필터링하면
        # 실제 D455 카메라와 동일한 범위를 탐지할 수 있음
        np_points = np.asarray(filtered_points)
        np_points[:,1] *= -1
        # np_points[:,2] *= -1
        # 카메라 좌표계와 ROS2 좌표계가 다르기 때문에 좌표 변환
        self.pcd.points = o3d.utility.Vector3dVector(np_points)
        # point cloud 데이터 입력 -> o3d에서 포인트 클라우드를 다룰 수 있도록 세팅

        self.pcd = self.pcd.voxel_down_sample(voxel_size = 0.15)
        # 최적의 voxel_size 찾을 필요 있음

        if len(np.asarray(self.pcd.points)) < 3:
            self.get_logger().warn("Not enough points for plane segmentation. Skipping this callback.")
            return

        self.pcd, inlier = self.pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
        self.pcd = self.pcd.select_by_index(inlier)
        # outlier 제거

        plane_model, inliers = self.pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
        self.pcd = self.pcd.select_by_index(inliers, invert = True)
        # 평면 제거

        labels = np.array(self.pcd.cluster_dbscan(eps=1.0, min_points=15, print_progress=True))
        # 최적의 클러스터링 파라미터 찾아야함

        if labels.size == 0:
            self.get_logger().warn("No clusters found. Skipping this callback.")
            # 만약 아무 클러스터도 형성되지 않으면, 들어올 때까지 경고 메시지를 출력하며 대기
            return

        max_label = labels.max()
        # colors = plt.get_cmap("tab20")(labels/(max_label+1))
        # colors[labels<0] = 0
        # pcd.colors = o3d.utility.Vector3dVector(colors[:,:3])
        # 클러스터에 색 입히는 코드들

        bounding_boxes = []
        # clusters_center_points = []
        # clusters_volume = []

        for i in range(max_label+1): # 클러스터들에 bounding box를 씌우는 코드
            cluster_indices = np.where(labels == i)[0]
            cluster_cloud = self.pcd.select_by_index(cluster_indices)
            bounding_box = cluster_cloud.get_axis_aligned_bounding_box()
            # axis_aligned 형태의 bounding box 생성 후 씌우기
            bounding_box.color = (1,0,0)
            bounding_boxes.append(bounding_box)

            cluster_center = bounding_box.get_center() # 중심점 추출
            # cluster_volume = bounding_box.volume() # 체적 계산
            # clusters_center_points.append(cluster_center) # 만약 클러스터가 여러개라면 중심점
            # clusters_volume.append(cluster_volume) # 만약 클러스터가 여러개라면 체적

            corners = np.asarray(bounding_box.get_box_points()) # bounding box의 코너점들 추출
            distances = np.linalg.norm(corners - cluster_center, axis=1) # 코너점들과 중심점 사이 거리
            farthest_corner = corners[distances.argmax()] # 그 중 가장 거리가 먼 점을 추출
            max_distance = distances.max() # 최대 거리도 추출

            self.farthest_corner_msg.x = farthest_corner[0]
            self.farthest_corner_msg.y = farthest_corner[1]
            self.farthest_corner_msg.z = farthest_corner[2]
            self.farthest_corner_publisher.publish(self.farthest_corner_msg)
            self.get_logger().info("Farthest Corner Point is Published") # 최외각점 pub

            self.center_point_msg.x = cluster_center[0]
            self.center_point_msg.y = cluster_center[1]
            self.center_point_msg.z = cluster_center[2]
            self.center_point_publisher.publish(self.center_point_msg)
            self.get_logger().info(f"Center Point is published: {cluster_center}") # 중심점 pub

            self.distance_msg.data = max_distance
            self.distance_publisher.publish(self.distance_msg)
            self.get_logger().info("Distance is published") # 최외각점과 중심점 사이 거리 pub
        # bounding box 연산
        # center_point, volume 산출

        # 실시간 시각화 코드 + bounding box 추가
        self.vis.clear_geometries()
        self.vis.add_geometry(self.pcd)
        for bounding_box in bounding_boxes:
            self.vis.add_geometry(bounding_box)
        self.vis.poll_events()
        self.vis.update_renderer()

def main(args=None):
    try:
        rclpy.init(args=args)
        try:
            point_cloud = PointCloudProcessing()
            rclpy.spin(point_cloud)
        except KeyboardInterrupt:
            point_cloud.get_logger().info("Bye")
        finally:
            point_cloud.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
