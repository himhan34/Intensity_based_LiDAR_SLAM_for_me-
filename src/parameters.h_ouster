#pragma once
// 파일이 한 번만 포함되도록 지시

#include <ros/ros.h>
// ROS (로봇 운영 체제) 핵심 헤더 파일 포함
#include <ros/package.h>
// ROS 패키지 관련 기능 포함

#include <std_msgs/Header.h>
// ROS 표준 메시지 중 Header 메시지 정의 포함
#include <std_msgs/Float64MultiArray.h>
// ROS 표준 메시지 중 double 배열 메시지 정의 포함
#include <std_msgs/Int64MultiArray.h>
// ROS 표준 메시지 중 int64 배열 메시지 정의 포함
#include <sensor_msgs/Image.h>
// 이미지 데이터를 위한 ROS 센서 메시지 정의 포함
#include <sensor_msgs/PointCloud.h>
// 포인트 클라우드 데이터를 위한 ROS 센서 메시지 정의 포함
#include <sensor_msgs/PointCloud2.h>
// 포인트 클라우드 데이터를 위한 개선된 ROS 센서 메시지 정의 포함
#include <sensor_msgs/image_encodings.h>
// 이미지 인코딩 관련 ROS 센서 메시지 정의 포함
#include <nav_msgs/Odometry.h>
// 오도메트리 데이터를 위한 ROS 내비게이션 메시지 정의 포함
#include <nav_msgs/Path.h>
// 경로 데이터를 위한 ROS 내비게이션 메시지 정의 포함
#include <visualization_msgs/Marker.h>
// 시각화 마커를 위한 ROS 메시지 정의 포함
#include <visualization_msgs/MarkerArray.h>
// 시각화 마커 배열을 위한 ROS 메시지 정의 포함

// #include <opencv/cv.h>
// OpenCV의 구 버전 헤더 파일 (현재 주석 처리됨)
#include <opencv2/opencv.hpp>
// OpenCV의 주요 기능 포함
#include <cv_bridge/cv_bridge.h>
// ROS와 OpenCV 간 이미지 변환을 위한 브릿지 기능 포함
#include <eigen3/Eigen/Dense>
// Eigen 라이브러리의 Dense (밀집 행렬 및 배열) 관련 기능 포함
#include <opencv2/core/eigen.hpp>
// OpenCV와 Eigen 간 변환 기능 포함

#include <pcl/point_cloud.h>
// PCL (Point Cloud Library)의 포인트 클라우드 기본 클래스 포함
#include <pcl/point_types.h>
// PCL의 다양한 포인트 타입 정의 포함
#include <pcl/range_image/range_image.h>
// PCL의 범위 이미지 관련 기능 포함
#include <pcl/kdtree/kdtree_flann.h>
// PCL의 KD-트리 및 FLANN (빠른 근사 최근접 이웃 검색) 기능 포함
#include <pcl/common/common.h>
// PCL의 공통 기능 포함
#include <pcl/common/centroid.h>
// PCL의 중심점 계산 관련 기능 포함
#include <pcl/common/transforms.h>
// PCL의 변환 (회전, 이동 등) 관련 기능 포함
#include <pcl/registration/icp.h>
// PCL의 ICP (Iterative Closest Point) 등록 알고리즘 포함
#include <pcl/io/pcd_io.h>
// PCL의 PCD (Point Cloud Data) 입출력 기능 포함
#include <pcl/filters/filter.h>
// PCL의 필터링 관련 기본 클래스 포함
#include <pcl/filters/voxel_grid.h>
// PCL의 복셀 그리드 필터링 기능 포함
#include <pcl/filters/crop_box.h> 
// PCL의 박스 영역으로 자르는 필터링 기능 포함
#include <pcl_conversions/pcl_conversions.h>
// PCL과 ROS 간 변환 기능 포함

#include <tf/LinearMath/Quaternion.h>
// ROS의 tf 라이브러리에서 Quaternion 계산 관련 기능 포함
#include <tf/transform_listener.h>
// ROS의 tf 변환 리스너 기능 포함
#include <tf/transform_datatypes.h>
// ROS의 tf 변환 데이터 타입 관련 기능 포함
#include <tf/transform_broadcaster.h>
// ROS의 tf 변환 브로드캐스터 기능 포함
 
#include <vector>
// C++ 표준 라이브러리의 벡터 컨테이너 포함
#include <cmath>
// C++ 표준 라이브러리의 수학 관련 함수 포함
#include <algorithm>
// C++ 표준 라이브러리의 알고리즘 관련 함수 포함
#include <queue>
// C++ 표준 라이브러리의 큐 컨테이너 포함
#include <deque>
// C++ 표준 라이브러리의 덱 (양방향 큐) 컨테이너 포함
#include <iostream>
// C++ 표준 라이브러리의 입출력 관련 기능 포함
#include <fstream>
// C++ 표준 라이브러리의 파일 입출력 관련 기능 포함
#include <ctime>
// C 표준 라이브러리의 시간 관련 기능 포함
#include <cfloat>
// C 표준 라이브러리의 부동 소수점 수 관련 제한 값 포함
#include <iterator>
// C++ 표준 라이브러리의 반복자 관련 기능 포함
#include <sstream>
// C++ 표준 라이브러리의 문자열 스트림 관련 기능 포함
#include <string>
// C++ 표준 라이브러리의 문자열 클래스 포함
#include <limits>
// C++ 표준 라이브러리의 타입 관련 제한 값 포함

#include <iomanip>
// C++ 표준 라이브러리의 입출력 형식 관련 기능 포함
#include <array>
// C++ 표준 라이브러리의 고정 크기 배열 컨테이너 포함
#include <thread>
// C++ 표준 라이브러리의 스레드 관련 기능 포함
#include <mutex>
// C++ 표준 라이브러리의 뮤텍스 (상호 배제) 관련 기능 포함
#include <cassert>
// C 표준 라이브러리의 assert 매크로 포함 (조건 검증을 위함)


#include "DBoW3/DBoW3.h"
// DBoW3 (Dense Bag of Words) 라이브러리 포함

typedef pcl::PointXYZI PointType;
// PCL의 XYZI (3차원 좌표 + 강도) 포인트 타입을 PointType으로 정의

typedef pcl::PointXYZ GroundPlanePointType;
// PCL의 XYZ (3차원 좌표) 포인트 타입을 GroundPlanePointType으로 정의

//pointXYZIR
struct PointXYZIR {
    PCL_ADD_POINT4D;
    // PCL의 4차원 포인트 표현 추가
    float intensity;
    // 강도 값
    std::uint8_t ring;
    // 레이저 스캐너의 링 인덱스
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Eigen 라이브러리와 메모리 정렬을 위한 매크로
}EIGEN_ALIGN16;
// Eigen 16바이트 정렬을 보장

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, ring, ring) 
)
// PointXYZIR 구조체를 PCL 포인트 구조체로 등록

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    // PCL의 4차원 포인트 표현 추가
    PCL_ADD_INTENSITY;
    // 강도 값 추가
    float roll;
    // 롤 (회전) 값
    float pitch;
    // 피치 (기울기) 값
    float yaw;
    // 요 (방향 전환) 값
    double time;
    // 시간 값
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Eigen 라이브러리와 메모리 정렬을 위한 매크로
} EIGEN_ALIGN16;
// Eigen 16바이트 정렬을 보장

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)
// PointXYZIRPYT 구조체를 PCL 포인트 구조체로 등록

typedef PointXYZIRPYT  PointTypePose;
// PointXYZIRPYT 타입을 PointTypePose로 정의

typedef pcl::PointXYZI PointOuster;
// pcl::PointXYZI 타입을 PointOuster로 정의
