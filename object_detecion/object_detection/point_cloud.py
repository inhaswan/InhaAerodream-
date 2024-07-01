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
#
############################################################################

import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import time

class PointCloudProcessing(Node):
    def __init__(self):
        super().__init__("point_cloud_processing")
        self.get_logger().info("Under Initializing ...")

        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.point_cloud_callback,
            10
        )
        # self.mapping_publisher = self.create_publisher()
        # mapping을 하기 위해서 클러스터들과 bounding box 정보를 보내주는 publisher

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Open3D")
        
        self.pcd = o3d.geometry.PointCloud()
        
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.vis.destroy_window()
    
    def point_cloud_callback(self, msg):
        if not isinstance(msg, PointCloud2):
            self.get_logger().error(f"Type of msg is not PointCloud2: {type(msg)}")
            return
    
        points = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
    
        while len(points) == 0:
            self.get_logger().warn("Received empty point cloud. Waiting for next message...")
            time.sleep(1)  # 1초 대기
        
            # 새로운 메시지 받기
            msg = self.subscription.take(1)
            if not msg:
                continue
        
            points = list(pc2.read_points(msg[0], field_names=('x','y','z'), skip_nans=True))

        filtered_points = [p for p in points if 0.3 <= p[2] <= 6]
        # gazebo 시뮬레이션 상으로 이미지 평면이 불필요하게 나오기 때문에,
        # depth camera 최대 탐지범위를 D455보다 길게 잡아놓고, 이미지 좌표계상 z축으로 6m까지만 필터링하면
        # 실제 D455 카메라와 동일한 범위를 탐지할 수 있음
        np_points = np.asarray(filtered_points)
        np_points[:,1] *= -1
        np_points[:,2] *= -1
        # 카메라 좌표계와 ROS2 좌표계가 다르기 때문에 좌표 변환
        self.pcd.points = o3d.utility.Vector3dVector(np_points)
        # point cloud 데이터 입력

        self.pcd = self.pcd.voxel_down_sample(voxel_size = 0.25)
        # 최적의 voxel_size 찾을 필요 있음

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
            return

        max_label = labels.max()
        # colors = plt.get_cmap("tab20")(labels/(max_label+1))
        # colors[labels<0] = 0
        # pcd.colors = o3d.utility.Vector3dVector(colors[:,:3])
        # 클러스터에 색 입히는 코드들

        bounding_boxes = []
        clusters_center_points = []
        clusters_volume = []

        for i in range(max_label+1):
            cluster_indices = np.where(labels == i)[0]
            cluster_cloud = self.pcd.select_by_index(cluster_indices)
            bounding_box = cluster_cloud.get_axis_aligned_bounding_box()
            bounding_box.color = (1,0,0)
            bounding_boxes.append(bounding_box)

            cluster_center = bounding_box.get_center()
            cluster_volume = bounding_box.volume()
            clusters_center_points.append(cluster_center)
            clusters_volume.append(cluster_volume)

            corners = np.asarray(bounding_box.get_box_points())
            print(corners)
        # bounding box 연산
        # center_point, volume 산출

        # 시각화 코드
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
