// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <math.h> // 수학 라이브러리 포함
#include <vector> // 벡터 라이브러리 포함
#include "parameters.h" // 매개변수 관련 헤더 파일 포함
#include <nav_msgs/Odometry.h> // ROS 네비게이션 오도메트리 메시지 헤더 포함
#include <nav_msgs/Path.h> // ROS 네비게이션 경로 메시지 헤더 포함
#include <geometry_msgs/PoseStamped.h> // ROS 지오메트리 포즈 스탬프 메시지 헤더 포함
#include <pcl_conversions/pcl_conversions.h> // PCL 포인트 클라우드 변환 라이브러리 포함
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 라이브러리 포함
#include <pcl/point_types.h> // PCL 포인트 타입 라이브러리 포함
#include <pcl/filters/voxel_grid.h> // PCL 복셀 그리드 필터 라이브러리 포함
#include <pcl/kdtree/kdtree_flann.h> // PCL KD 트리 라이브러리 포함
#include <ros/ros.h> // ROS 라이브러리 포함
#include <sensor_msgs/Imu.h> // ROS 센서 IMU 메시지 헤더 포함
#include <sensor_msgs/PointCloud2.h> // ROS 센서 포인트 클라우드 메시지 헤더 포함
#include <tf/transform_datatypes.h> // ROS TF 변환 데이터 타입 라이브러리 포함
#include <tf/transform_broadcaster.h> // ROS TF 변환 브로드캐스터 라이브러리 포함
#include <eigen3/Eigen/Dense> // Eigen 라이브러리 포함
#include <ceres/ceres.h> // Ceres 최적화 라이브러리 포함
#include <mutex> // 뮤텍스 라이브러리 포함
#include <queue> // 큐 라이브러리 포함
#include <thread> // 스레드 라이브러리 포함
#include <iostream> // 입출력 라이브러리 포함
#include <string> // 문자열 라이브러리 포함
#include "lidarFeaturePointsFunction.hpp" // LiDAR 특징 포인트 함수 헤더 포함
#include "tic_toc.h" // 시간 측정 도우미 함수 헤더 포함

int frameCount = 0; // 프레임 카운트 변수

double timeLaserCloudCornerLast = 0; // 최근 코너 레이저 클라우드 시간
double timeLaserCloudSurfLast = 0; // 최근 서피스 레이저 클라우드 시간
double timeLaserCloudFullRes = 0; // 전체 해상도 레이저 클라우드 시간
double timeLaserOdometry = 0; // 레이저 오도메트리 시간

int laserCloudCenWidth = 10; // 중심 레이저 클라우드 너비
int laserCloudCenHeight = 10; // 중심 레이저 클라우드 높이
int laserCloudCenDepth = 5; // 중심 레이저 클라우드 깊이

const int laserCloudWidth = 21; // 레이저 클라우드 너비
const int laserCloudHeight = 21; // 레이저 클라우드 높이
const int laserCloudDepth = 11; // 레이저 클라우드 깊이

const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; // 레이저 클라우드 총 개수 (4851)

int laserCloudValidInd[125]; // 유효한 레이저 클라우드 인덱스 배열 (125개)
int laserCloudSurroundInd[125]; // 주변 레이저 클라우드 인덱스 배열 (125개)

// 입력: 오도메트리로부터
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>()); // 이전 코너 레이저 클라우드
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>()); // 이전 서피스 레이저 클라우드

// 출력: 모든 시각적으로 볼 수 있는 큐브 포인트
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>()); // 주변 레이저 클라우드

// 맵에서 트리를 구축하기 위한 주변 포인트
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>()); // 맵으로부터 가져온 코너 레이저 클라우드
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>()); // 맵으로부터 가져온 서피스 레이저 클라우드

// 입력 및 출력: 한 프레임의 포인트. 로컬에서 글로벌로
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>()); // 한 프레임의 전체 해상도 레이저 클라우드

// 각 큐브 내의 포인트
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum]; // 각 큐브의 코너 레이저 클라우드
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum]; // 각 큐브의 서피스 레이저 클라우드

// kd 트리
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()); // 코너 레이저 클라우드를 위한 kd 트리
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>()); // 서피스 레이저 클라우드를 위한 kd 트리

double parameters[7] = {0, 0, 0, 1, 0, 0, 0}; // 초기 위치 및 자세 파라미터
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters); // 현재 자세를 나타내는 Quaternion 매핑
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4); // 현재 위치를 나타내는 Vector3d 매핑

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// odom의 월드와 map의 월드 프레임 사이의 변환
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0); // 맵 월드에서 오도메트리 월드로의 회전을 나타내는 Quaternion
Eigen::Vector3d t_wmap_wodom(0, 0, 0); // 맵 월드에서 오도메트리 월드로의 이동을 나타내는 Vector3d

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0); // 오도메트리 월드에서 현재 위치로의 회전을 나타내는 Quaternion
Eigen::Vector3d t_wodom_curr(0, 0, 0); // 오도메트리 월드에서 현재 위치로의 이동을 나타내는 Vector3d

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf; // 코너 레이저 클라우드 버퍼 큐
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf; // 서피스 레이저 클라우드 버퍼 큐
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf; // 전체 해상도 레이저 클라우드 버퍼 큐
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf; // 오도메트리 버퍼 큐
std::mutex mBuf; // 뮤텍스

pcl::VoxelGrid<PointType> downSizeFilterCorner; // 코너 레이저 클라우드 다운샘플링 필터
pcl::VoxelGrid<PointType> downSizeFilterSurf; // 서피스 레이저 클라우드 다운샘플링 필터

std::vector<int> pointSearchInd; // 포인트 검색 인덱스 벡터
std::vector<float> pointSearchSqDis; // 포인트 검색 제곱 거리 벡터

PointType pointOri, pointSel; // 포인트 변수

ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath; // ROS 퍼블리셔

nav_msgs::Path laserAfterMappedPath; // 매핑된 레이저 경로



// 초기 추정값 설정
void transformAssociateToMap()
{
    q_w_curr = q_wmap_wodom * q_wodom_curr; // 현재 자세를 맵 월드 프레임에 연결
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; // 현재 위치를 맵 월드 프레임에 연결
}

// 변환 업데이트
void transformUpdate()
{
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse(); // 맵 월드 프레임과 오도메트리 월드 프레임 사이의 회전 업데이트
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr; // 맵 월드 프레임과 오도메트리 월드 프레임 사이의 이동 업데이트
}

// 포인트를 맵으로 변환
void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z); // 현재 포인트
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr; // 맵 월드 프레임으로 포인트 변환
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

// 포인트를 맵으로 연결
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_w(pi->x, pi->y, pi->z); // 월드 프레임에서의 포인트
    Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr); // 현재 자세 및 위치로 포인트 변환
    po->x = point_curr.x();
    po->y = point_curr.y();
    po->z = point_curr.z();
    po->intensity = pi->intensity;
}

// 이전 코너 레이저 클라우드 핸들러
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
    mBuf.lock(); // 버퍼에 접근하기 위한 뮤텍스 잠금
    cornerLastBuf.push(laserCloudCornerLast2); // 이전 코너 레이저 클라우드를 버퍼에 추가
    mBuf.unlock(); // 뮤텍스 잠금 해제
}

// 이전 서피스 레이저 클라우드 핸들러
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
    mBuf.lock(); // 버퍼에 접근하기 위한 뮤텍스 잠금
    surfLastBuf.push(laserCloudSurfLast2); // 이전 서피스 레이저 클라우드를 버퍼에 추가
    mBuf.unlock(); // 뮤텍스 잠금 해제
}

// 전체 해상도 레이저 클라우드 핸들러
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock(); // 버퍼에 접근하기 위한 뮤텍스 잠금
    fullResBuf.push(laserCloudFullRes2); // 전체 해상도 레이저 클라우드를 버퍼에 추가
    mBuf.unlock(); // 뮤텍스 잠금 해제
}

// 오도메트리 수신 핸들러
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    mBuf.lock(); // 버퍼에 접근하기 위한 뮤텍스 잠금
    odometryBuf.push(laserOdometry); // 오도메트리 데이터를 버퍼에 추가
    mBuf.unlock(); // 뮤텍스 잠금 해제

    // 고주파로 발행
    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;
    q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
    q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
    q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
    q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
    t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
    t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
    t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

    Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
    Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "/aft_mapped";
    odomAftMapped.header.stamp = laserOdometry->header.stamp;
    odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
    odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
    odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
    odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
    odomAftMapped.pose.pose.position.x = t_w_curr.x();
    odomAftMapped.pose.pose.position.y = t_w_curr.y();
    odomAftMapped.pose.pose.position.z = t_w_curr.z();
    pubOdomAftMappedHighFrec.publish(odomAftMapped); // 고주파로 오도메트리 발행
}

void process()
{
	while(1) // 무한 루프
	{
		while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
			!fullResBuf.empty() && !odometryBuf.empty())
		{
			mBuf.lock(); // 버퍼에 접근하기 위한 뮤텍스 잠금

			// 버퍼에서 타임스탬프를 비교하여 가장 오래된 데이터를 버림
			while (!cornerLastBuf.empty() && cornerLastBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
				cornerLastBuf.pop();

			if (cornerLastBuf.empty()) // 코너 레이저 클라우드 버퍼가 비어있으면
			{
				mBuf.unlock(); // 뮤텍스 잠금 해제
				break;
			}

			while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				odometryBuf.pop();

			if (odometryBuf.empty()) // 오도메트리 버퍼가 비어있으면
			{
				mBuf.unlock(); // 뮤텍스 잠금 해제
				break;
			}

			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
				surfLastBuf.pop();

			if (surfLastBuf.empty()) // 서피스 레이저 클라우드 버퍼가 비어있으면
			{
				mBuf.unlock(); // 뮤텍스 잠금 해제
				break;
			}

			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
				fullResBuf.pop();

			if (fullResBuf.empty()) // 전체 해상도 레이저 클라우드 버퍼가 비어있으면
			{
				mBuf.unlock(); // 뮤텍스 잠금 해제
				break;
			}

			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec(); // 코너 레이저 클라우드의 타임스탬프 저장
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec(); // 서피스 레이저 클라우드의 타임스탬프 저장
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec(); // 전체 해상도 레이저 클라우드의 타임스탬프 저장
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec(); // 오도메트리의 타임스탬프 저장
			
			// 데이터의 타임스탬프가 모두 동일하지 않으면
			if (timeLaserCloudCornerLast != timeLaserOdometry ||
			    timeLaserCloudSurfLast != timeLaserOdometry ||
			    timeLaserCloudFullRes != timeLaserOdometry)
			{
			    printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
			    printf("unsync messeage!"); // 비동기 메시지 출력
			    mBuf.unlock(); // 뮤텍스 잠금 해제
			    break; // 루프 종료
			}
			
			laserCloudCornerLast->clear(); // 코너 레이저 클라우드 초기화
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast); // ROS 메시지를 PCL 포인트 클라우드로 변환하여 저장
			cornerLastBuf.pop(); // 버퍼에서 데이터 제거
			
			laserCloudSurfLast->clear(); // 서피스 레이저 클라우드 초기화
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast); // ROS 메시지를 PCL 포인트 클라우드로 변환하여 저장
			surfLastBuf.pop(); // 버퍼에서 데이터 제거
			
			laserCloudFullRes->clear(); // 전체 해상도 레이저 클라우드 초기화
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes); // ROS 메시지를 PCL 포인트 클라우드로 변환하여 저장
			fullResBuf.pop(); // 버퍼에서 데이터 제거
			
			// 오도메트리 데이터를 읽어와 현재 자세와 위치 업데이트
			q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
			odometryBuf.pop(); // 버퍼에서 데이터 제거

			while (!cornerLastBuf.empty())
			{
			    cornerLastBuf.pop(); // 코너 레이저 클라우드 버퍼의 모든 데이터 삭제
			    printf("drop lidar frame in mapping for real-time performance \n"); // 실시간 성능을 위해 레이더 프레임 삭제 메시지 출력
			}
			
			mBuf.unlock(); // 뮤텍스 잠금 해제
			
			TicToc t_whole; // 전체 시간 측정 시작
			
			transformAssociateToMap(); // 현재 자세 및 위치를 맵에 연결
			
			TicToc t_shift; // 시프트 시간 측정 시작
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth; // 중심 큐브 I 인덱스 계산
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight; // 중심 큐브 J 인덱스 계산
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth; // 중심 큐브 K 인덱스 계산
			
			if (t_w_curr.x() + 25.0 < 0)
			    centerCubeI--; // X 좌표가 음수인 경우 중심 큐브 I 인덱스 수정
			if (t_w_curr.y() + 25.0 < 0)
			    centerCubeJ--; // Y 좌표가 음수인 경우 중심 큐브 J 인덱스 수정
			if (t_w_curr.z() + 25.0 < 0)
			    centerCubeK--; // Z 좌표가 음수인 경우 중심 큐브 K 인덱스 수정

			while (centerCubeI < 3)
			{
			    for (int j = 0; j < laserCloudHeight; j++)
			    {
				for (int k = 0; k < laserCloudDepth; k++)
				{
				    int i = laserCloudWidth - 1; // 큐브의 가장 오른쪽 열부터 시작
				    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
				    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			
				    // 큐브 배열의 각 열을 한 칸 오른쪽으로 이동
				    for (; i >= 1; i--)
				    {
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					    laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					    laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
				    }
			
				    // 가장 왼쪽 열에 이전 큐브 데이터 복사
				    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					laserCloudCubeCornerPointer;
				    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					laserCloudCubeSurfPointer;
			
				    // 이전 큐브 데이터 초기화
				    laserCloudCubeCornerPointer->clear();
				    laserCloudCubeSurfPointer->clear();
				}
			    }
			
			    centerCubeI++; // 중심 큐브 I 인덱스 증가
			    laserCloudCenWidth++; // 중심 큐브의 너비 인덱스 증가
			}
			



			while (centerCubeI >= laserCloudWidth - 3)
			{
			    for (int j = 0; j < laserCloudHeight; j++)
			    {
				for (int k = 0; k < laserCloudDepth; k++)
				{
				    int i = 0; // 큐브의 가장 왼쪽 열부터 시작
				    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
				    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			
				    // 큐브 배열의 각 열을 한 칸 왼쪽으로 이동
				    for (; i < laserCloudWidth - 1; i++)
				    {
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					    laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					    laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
				    }
			
				    // 가장 오른쪽 열에 이전 큐브 데이터 복사
				    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					laserCloudCubeCornerPointer;
				    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					laserCloudCubeSurfPointer;
			
				    // 이전 큐브 데이터 초기화
				    laserCloudCubeCornerPointer->clear();
				    laserCloudCubeSurfPointer->clear();
				}
			    }
			
			    centerCubeI--; // 중심 큐브 I 인덱스 감소
			    laserCloudCenWidth--; // 중심 큐브의 너비 인덱스 감소
			}


			while (centerCubeJ < 3)
			{
			    for (int i = 0; i < laserCloudWidth; i++)
			    {
			        for (int k = 0; k < laserCloudDepth; k++)
			        {
			            int j = laserCloudHeight - 1; // 큐브의 가장 위쪽 행부터 시작
			            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
			                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
			                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			
			            // 큐브 배열의 각 행을 한 칸 아래로 이동
			            for (; j >= 1; j--)
			            {
			                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                    laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
			                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                    laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
			            }
			
			            // 가장 아래쪽 행에 이전 큐브 데이터 복사
			            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                laserCloudCubeCornerPointer;
			            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                laserCloudCubeSurfPointer;
			
			            // 이전 큐브 데이터 초기화
			            laserCloudCubeCornerPointer->clear();
			            laserCloudCubeSurfPointer->clear();
			        }
			    }
			
			    centerCubeJ++; // 중심 큐브 J 인덱스 증가
			    laserCloudCenHeight++; // 중심 큐브의 높이 인덱스 증가
			}

			while (centerCubeJ >= laserCloudHeight - 3)
			{
			    for (int i = 0; i < laserCloudWidth; i++)
			    {
			        for (int k = 0; k < laserCloudDepth; k++)
			        {
			            int j = 0; // 큐브의 가장 아래쪽 행부터 시작
			            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
			                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
			                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			
			            // 큐브 배열의 각 행을 한 칸 위로 이동
			            for (; j < laserCloudHeight - 1; j++)
			            {
			                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                    laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
			                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                    laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
			            }
			
			            // 가장 위쪽 행에 이전 큐브 데이터 복사
			            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                laserCloudCubeCornerPointer;
			            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                laserCloudCubeSurfPointer;
			
			            // 이전 큐브 데이터 초기화
			            laserCloudCubeCornerPointer->clear();
			            laserCloudCubeSurfPointer->clear();
			        }
			    }
			
			    centerCubeJ--; // 중심 큐브 J 인덱스 감소
			    laserCloudCenHeight--; // 중심 큐브의 높이 인덱스 감소
			}

			while (centerCubeK < 3)
			{
			    for (int i = 0; i < laserCloudWidth; i++)
			    {
				for (int j = 0; j < laserCloudHeight; j++)
				{
				    int k = laserCloudDepth - 1; // 큐브의 가장 위쪽 층부터 시작
				    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
				    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			
				    // 큐브 배열의 각 층을 한 칸 위로 이동
				    for (; k >= 1; k--)
				    {
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
				    }
			
				    // 가장 위쪽 층에 이전 큐브 데이터 복사
				    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					laserCloudCubeCornerPointer;
				    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
					laserCloudCubeSurfPointer;
			
				    // 이전 큐브 데이터 초기화
				    laserCloudCubeCornerPointer->clear();
				    laserCloudCubeSurfPointer->clear();
				}
			    }
			
			    centerCubeK++; // 중심 큐브 K 인덱스 증가
			    laserCloudCenDepth++; // 중심 큐브의 깊이 인덱스 증가
			}

			while (centerCubeK >= laserCloudDepth - 3)
			{
			    for (int i = 0; i < laserCloudWidth; i++)
			    {
			        for (int j = 0; j < laserCloudHeight; j++)
			        {
			            int k = 0;
			            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
			                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
			                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
			
			            // 큐브 배열의 각 층을 한 칸 아래로 이동
			            for (; k < laserCloudDepth - 1; k++)
			            {
			                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
			                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
			            }
			
			            // 가장 아래쪽 층에 이전 큐브 데이터 복사
			            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                laserCloudCubeCornerPointer;
			            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
			                laserCloudCubeSurfPointer;
			
			            // 이전 큐브 데이터 초기화
			            laserCloudCubeCornerPointer->clear();
			            laserCloudCubeSurfPointer->clear();
			        }
			    }
			
			    centerCubeK--; // 중심 큐브 K 인덱스 감소
			    laserCloudCenDepth--; // 중심 큐브의 깊이 인덱스 감소
			}



			int laserCloudValidNum = 0;  // 유효한 포인트 수
			int laserCloudSurroundNum = 0;  // 주변 포인트 수
			
			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
			    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
			    {
			        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
			        {
			            if (i >= 0 && i < laserCloudWidth &&
			                j >= 0 && j < laserCloudHeight &&
			                k >= 0 && k < laserCloudDepth)
			            { 
			                // 유효한 포인트 인덱스를 저장
			                laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
			                laserCloudValidNum++;
			
			                // 주변 포인트 인덱스를 저장
			                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
			                laserCloudSurroundNum++;
			            }
			        }
			    }
			}


			laserCloudCornerFromMap->clear();  // 지도에서 가져온 코너 포인트 클리어
			laserCloudSurfFromMap->clear();    // 지도에서 가져온 서피스 포인트 클리어
			
			for (int i = 0; i < laserCloudValidNum; i++)
			{
			    // 지도에서 가져온 코너 포인트와 서피스 포인트를 병합
			    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
			    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}
			
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();  // 지도에서 가져온 코너 포인트 수
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();      // 지도에서 가져온 서피스 포인트 수
			
			pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
			downSizeFilterCorner.filter(*laserCloudCornerStack);
			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();  // 현재 프레임의 코너 포인트 수
			
			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
			downSizeFilterSurf.filter(*laserCloudSurfStack);
			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();      // 현재 프레임의 서피스 포인트 수
			
			printf("map prepare time %f ms\n", t_shift.toc());
			// 현재까지의 코드 실행 시간을 출력합니다.
			
			printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
			// 현재 지도에 있는 코너 포인트 수와 서피스 포인트 수를 출력합니다.
			
			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
			{
			    // 만약 현재 지도에 있는 코너 포인트 수가 10개보다 크고 서피스 포인트 수가 50개보다 크면 아래 코드를 실행합니다.
			
			    TicToc t_opt;
			    TicToc t_tree;
			
			    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
			    // Ceres 최적화에서 사용할 지도 상의 코너 포인트를 Kd 트리에 입력합니다.
			
			    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
			    // Ceres 최적화에서 사용할 지도 상의 서피스 포인트를 Kd 트리에 입력합니다.
			
			    printf("build tree time %f ms \n", t_tree.toc());
			    // Kd 트리 구축 시간을 출력합니다.
			
			    for (int iterCount = 0; iterCount < 2; iterCount++)
			    {
			        // 최적화를 2번 반복합니다.
			
			        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
			        // 최적화 중에 사용할 Huber Loss 함수를 설정하고, 0.1은 Huber Loss의 파라미터입니다.
			
			        ceres::LocalParameterization *q_parameterization =
			            new ceres::EigenQuaternionParameterization();
			        // 회전 변수 (Quaternion)에 대한 로컬 파라미터화 방법을 설정합니다.
			
			        ceres::Problem::Options problem_options;
			        ceres::Problem problem(problem_options);
			        // Ceres 최적화 문제를 설정합니다.
			
			        problem.AddParameterBlock(parameters, 4, q_parameterization);
			        // 회전 변수 (Quaternion)에 대한 최적화 변수를 추가합니다.
			
			        problem.AddParameterBlock(parameters + 4, 3);
			        // 변환 변수 (Translation)에 대한 최적화 변수를 추가합니다.
			
			        TicToc t_data;
			        int corner_num = 0;
			        // 데이터 처리 시간을 측정하기 위한 변수입니다.

				for (int i = 0; i < laserCloudCornerStackNum; i++)
				{
				    pointOri = laserCloudCornerStack->points[i];
				    // 현재 스택에 있는 코너 포인트를 가져옵니다.
				
				    pointAssociateToMap(&pointOri, &pointSel);
				    // 현재 코너 포인트를 지도 상의 좌표계로 변환합니다.
				
				    kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
				    // 지도 상의 코너 포인트에서 현재 코너 포인트에 가장 가까운 5개의 이웃 포인트를 찾습니다.
				
				    if (pointSearchSqDis[4] < 1.0)
				    {
				        // 만약 현재 코너 포인트와 가장 가까운 이웃 포인트 사이의 거리 제곱이 1.0보다 작다면 아래 코드를 실행합니다.
				        // 이 조건은 코너 포인트가 서로 가까운 포인트들과 선 모양을 이루고 있을 때를 나타냅니다.
				
				        std::vector<Eigen::Vector3d> nearCorners;
				        Eigen::Vector3d center(0, 0, 0);
				        for (int j = 0; j < 5; j++)
				        {
				            Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
				                                laserCloudCornerFromMap->points[pointSearchInd[j]].y,
				                                laserCloudCornerFromMap->points[pointSearchInd[j]].z);
				            center = center + tmp;
				            nearCorners.push_back(tmp);
				        }
				        center = center / 5.0;
				        // 가장 가까운 5개 포인트의 중심점과 각 포인트를 nearCorners 벡터에 저장합니다.
				
				        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
				        for (int j = 0; j < 5; j++)
				        {
				            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
				            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
				        }
				        // 가장 가까운 5개 포인트의 공분산 행렬을 계산합니다.
				
				        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
				
				        // 코너 포인트가 선 모양일 경우
				        // 주의: Eigen 라이브러리는 고유값을 증가 순서로 정렬합니다.
				        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
				        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
				        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
				        {
				            // 만약 가장 큰 고유값이 두 번째로 큰 고유값의 3배 이상이면 아래 코드를 실행합니다.
				            // 이 조건은 코너 포인트가 선 모양을 이루고 있을 때를 나타냅니다.
				
				            Eigen::Vector3d point_on_line = center;
				            Eigen::Vector3d point_a, point_b;
				            point_a = 0.1 * unit_direction + point_on_line;
				            point_b = -0.1 * unit_direction + point_on_line;
				
				            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
				            // 코스트 함수를 생성합니다. 이 함수는 코너 포인트와 라인의 두 점 사이의 거리를 최적화 대상으로 합니다.
				            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
				            // 최적화 문제에 코스트 함수를 추가합니다.
				            corner_num++;
				        }                            
				    }
				    /*
				    else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
				    {
				        Eigen::Vector3d center(0, 0, 0);
				        for (int j = 0; j < 5; j++)
				        {
				            Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
				                                laserCloudCornerFromMap->points[pointSearchInd[j]].y,
				                                laserCloudCornerFromMap->points[pointSearchInd[j]].z);
				            center = center + tmp;
				        }
				        center = center / 5.0;    
				        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
				        ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
				        problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
				    }
				    */
				}

				int surf_num = 0;
				for (int i = 0; i < laserCloudSurfStackNum; i++)
				{
				    pointOri = laserCloudSurfStack->points[i];
				    // 현재 스택에 있는 서피스 포인트를 가져옵니다.
				
				    pointAssociateToMap(&pointOri, &pointSel);
				    // 현재 서피스 포인트를 지도 상의 좌표계로 변환합니다.
				
				    kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
				    // 지도 상의 서피스 포인트에서 현재 서피스 포인트에 가장 가까운 5개의 이웃 포인트를 찾습니다.
				
				    Eigen::Matrix<double, 5, 3> matA0;
				    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
				    if (pointSearchSqDis[4] < 1.0)
				    {
					// 만약 현재 서피스 포인트와 가장 가까운 이웃 포인트 사이의 거리 제곱이 1.0보다 작다면 아래 코드를 실행합니다.
					// 이 조건은 서피스 포인트가 서로 가까운 포인트들과 평면 모양을 이루고 있을 때를 나타냅니다.
				
					for (int j = 0; j < 5; j++)
					{
					    matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
					    matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
					    matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
					}
					// 가장 가까운 5개 포인트의 좌표를 matA0 행렬에 저장합니다.
				
					Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
					double negative_OA_dot_norm = 1 / norm.norm();
					norm.normalize();
					// 평면의 법선 벡터와 negative_OA_dot_norm 값을 계산합니다.
				
					// 여기서 n(pa, pb, pc)는 평면의 단위 법선 벡터입니다.
					bool planeValid = true;
					for (int j = 0; j < 5; j++)
					{
					    // 만약 OX * n > 0.2이면 평면이 잘 맞지 않는 것으로 판단합니다.
					    if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
						     norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
						     norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
					    {
						planeValid = false;
						break;
					    }
					}
					Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
					if (planeValid)
					{
					    // 코스트 함수를 생성합니다. 이 함수는 서피스 포인트와 평면의 법선 벡터 및 negative_OA_dot_norm 값을 최적화 대상으로 합니다.
					    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
					    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
					    surf_num++;
					}
				    }



						/*
						// 포인트가 주어진 거리 내에 있으면
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
						    // 중심점을 (0, 0, 0)으로 초기화
						    Eigen::Vector3d center(0, 0, 0);
						    // 5개의 포인트를 더해서 중심점 계산
						    for (int j = 0; j < 5; j++)
						    {
						        Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
						                            laserCloudSurfFromMap->points[pointSearchInd[j]].y,
						                            laserCloudSurfFromMap->points[pointSearchInd[j]].z);
						        center = center + tmp;
						    }
						    // 중심점을 5로 나눔
						    center = center / 5.0;  
						    // 현재 포인트 위치
						    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
						    // 비용 함수 생성
						    ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
						    // 문제에 비용 함수 추가
						    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}
					// 코너 포인트 개수와 사용된 코너 포인트 개수 출력
					//printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
					// 서프 포인트 개수와 사용된 서프 포인트 개수 출력
					//printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);
					
					// 데이터 연관 시간 출력
					printf("mapping data assosiation time %f ms \n", t_data.toc());
					
					// 솔버 시간 측정 시작
					TicToc t_solver;
					ceres::Solver::Options options;
					// 선형 솔버 타입을 DENSE_QR로 설정
					options.linear_solver_type = ceres::DENSE_QR;
					// 최대 반복 횟수를 4로 설정
					options.max_num_iterations = 4;
					// 진행 상태를 표준 출력으로 보내지 않음
					options.minimizer_progress_to_stdout = false;
					// 기울기 검사를 하지 않음
					options.check_gradients = false;
					// 기울기 검사 상대 정밀도 설정
					options.gradient_check_relative_precision = 1e-4;
					ceres::Solver::Summary summary;
				    
					// 문제 해결
					ceres::Solve(options, &problem, &summary);
					// 솔버 시간 출력
					printf("mapping solver time %f ms \n", t_solver.toc());
					
					// 레이저 오도메트리 시간 출력
					//printf("time %f \n", timeLaserOdometry);
					// 코너 인자 개수와 서프 인자 개수 출력
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					// 결과 파라미터 출력
					//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
					//     parameters[4], parameters[5], parameters[6]);
					}
						// 최적화 시간 출력
						printf("mapping optimization time %f \n", t_opt.toc());
					}




			// 충분한 코너와 서프 포인트가 없는 경우 경고 메시지 출력
			else
			{
			    ROS_WARN("time Map corner and surf num are not enough");
			}
			// 변환 업데이트 함수 호출
			transformUpdate();
			
			// 포인트 추가 시간 측정 시작
			TicToc t_add;
			// 모든 코너 포인트에 대해 반복
			for (int i = 0; i < laserCloudCornerStackNum; i++)
			{
			    // 맵 좌표계로 포인트 변환
			    pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
			
			    // 큐브 인덱스 계산
			    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
			    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
			    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
			
			    // 포인트가 음수영역에 있는 경우 인덱스 조정
			    if (pointSel.x + 25.0 < 0)
			        cubeI--;
			    if (pointSel.y + 25.0 < 0)
			        cubeJ--;
			    if (pointSel.z + 25.0 < 0)
			        cubeK--;
			
			    // 큐브 인덱스가 유효한 범위 내에 있는 경우
			    if (cubeI >= 0 && cubeI < laserCloudWidth &&
			        cubeJ >= 0 && cubeJ < laserCloudHeight &&
			        cubeK >= 0 && cubeK < laserCloudDepth)
			    {
			        // 큐브 배열에서 해당 인덱스 계산
			        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
			        // 해당 큐브에 포인트 추가
			        laserCloudCornerArray[cubeInd]->push_back(pointSel);
			    }
			}


			// 모든 서프 포인트에 대해 반복
			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
			    // 맵 좌표계로 포인트 변환
			    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
			
			    // 큐브 인덱스 계산
			    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
			    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
			    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
			
			    // 포인트가 음수영역에 있는 경우 인덱스 조정
			    if (pointSel.x + 25.0 < 0)
			        cubeI--;
			    if (pointSel.y + 25.0 < 0)
			        cubeJ--;
			    if (pointSel.z + 25.0 < 0)
			        cubeK--;
			
			    // 큐브 인덱스가 유효한 범위 내에 있는 경우
			    if (cubeI >= 0 && cubeI < laserCloudWidth &&
			        cubeJ >= 0 && cubeJ < laserCloudHeight &&
			        cubeK >= 0 && cubeK < laserCloudDepth)
			    {
			        // 큐브 배열에서 해당 인덱스 계산
			        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
			        // 해당 큐브에 포인트 추가
			        laserCloudSurfArray[cubeInd]->push_back(pointSel);
			    }
			}
			// 포인트 추가 시간 출력
			printf("add points time %f ms\n", t_add.toc());
			
			
			// 필터링 시간 측정 시작
			TicToc t_filter;



			// 서프 포인트 배열에 대한 반복
			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
			    // 맵 좌표계로 포인트 변환
			    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
			
			    // 큐브 인덱스 계산
			    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
			    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
			    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
			
			    // 포인트가 음수 영역에 있는 경우 인덱스 조정
			    if (pointSel.x + 25.0 < 0)
			        cubeI--;
			    if (pointSel.y + 25.0 < 0)
			        cubeJ--;
			    if (pointSel.z + 25.0 < 0)
			        cubeK--;
			
			    // 큐브 인덱스가 유효한 범위 내에 있는 경우
			    if (cubeI >= 0 && cubeI < laserCloudWidth &&
			        cubeJ >= 0 && cubeJ < laserCloudHeight &&
			        cubeK >= 0 && cubeK < laserCloudDepth)
			    {
			        // 큐브 배열에서 해당 인덱스 계산
			        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
			        // 해당 큐브에 포인트 추가
			        laserCloudSurfArray[cubeInd]->push_back(pointSel);
			    }
			}
			// 포인트 추가 시간 출력
			printf("add points time %f ms\n", t_add.toc());
			
			// 필터링 시간 측정 시작
			TicToc t_filter;
			
			// 유효한 레이저 클라우드에 대한 반복
			for (int i = 0; i < laserCloudValidNum; i++)
			{
			    int ind = laserCloudValidInd[i];
			
			    // 코너 포인트 클라우드 필터링
			    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
			    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
			    downSizeFilterCorner.filter(*tmpCorner);
			    laserCloudCornerArray[ind] = tmpCorner;
			
			    // 서프 포인트 클라우드 필터링
			    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
			    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
			    downSizeFilterSurf.filter(*tmpSurf);
			    laserCloudSurfArray[ind] = tmpSurf;
			}
			// 필터링 시간 출력
			printf("filter time %f ms \n", t_filter.toc());
			
			// 주변 맵 발행 시간 측정 시작
			TicToc t_pub;
			// 매 5 프레임마다 주변 맵 발행
			if (frameCount % 5 == 0)
			{
			    // 주변 레이저 클라우드 초기화
			    laserCloudSurround->clear();
			    // 주변 레이저 클라우드에 포인트 추가
			    for (int i = 0; i < laserCloudSurroundNum; i++)
			    {
			        int ind = laserCloudSurroundInd[i];
			        *laserCloudSurround += *laserCloudCornerArray[ind];
			        *laserCloudSurround += *laserCloudSurfArray[ind];
			    }
			
			    // ROS 메시지로 변환 및 발행
			    sensor_msgs::PointCloud2 laserCloudSurround3;
			    pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
			    laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			    laserCloudSurround3.header.frame_id = "map";
			    pubLaserCloudSurround.publish(laserCloudSurround3);
			}

			// 매 20 프레임마다 수행
			if (frameCount % 20 == 0)
			{
			    // 레이저 클라우드 맵 초기화
			    pcl::PointCloud<PointType> laserCloudMap;
			    // 모든 큐브에 대한 반복
			    for (int i = 0; i < 4851; i++)
			    {
			        // 코너와 서프 포인트를 맵에 추가
			        laserCloudMap += *laserCloudCornerArray[i];
			        laserCloudMap += *laserCloudSurfArray[i];
			    }
			    // ROS 메시지로 변환
			    sensor_msgs::PointCloud2 laserCloudMsg;
			    pcl::toROSMsg(laserCloudMap, laserCloudMsg);
			    // 헤더 정보 설정 및 발행
			    laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			    laserCloudMsg.header.frame_id = "map";
			    pubLaserCloudMap.publish(laserCloudMsg);
			}
			
			// 전체 해상도 레이저 클라우드 포인트 수 계산
			int laserCloudFullResNum = laserCloudFullRes->points.size();
			// 모든 포인트에 대한 반복
			for (int i = 0; i < laserCloudFullResNum; i++)
			{
			    // 맵 좌표계로 포인트 변환
			    pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}
			
			// 전체 해상도 레이저 클라우드를 ROS 메시지로 변환 및 발행
			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			laserCloudFullRes3.header.frame_id = "map";
			pubLaserCloudFullRes.publish(laserCloudFullRes3);
			
			// 맵 발행 시간 출력
			printf("mapping pub time %f ms \n", t_pub.toc());
			
			// 전체 매핑 시간 출력
			printf("whole mapping time %f ms +++++\n", t_whole.toc());

			// 매핑 후 오도메트리 메시지 초기화
			nav_msgs::Odometry odomAftMapped;
			// 헤더의 프레임 ID 설정
			odomAftMapped.header.frame_id = "map";
			// 자식 프레임 ID 설정
			odomAftMapped.child_frame_id = "/aft_mapped";
			// 헤더의 시간 스탬프 설정
			odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			// 오리엔테이션(방향) 설정
			odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
			odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
			odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
			odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
			// 위치 설정
			odomAftMapped.pose.pose.position.x = t_w_curr.x();
			odomAftMapped.pose.pose.position.y = t_w_curr.y();
			odomAftMapped.pose.pose.position.z = t_w_curr.z();
			// 오도메트리 메시지 발행
			pubOdomAftMapped.publish(odomAftMapped);

			
			// 매핑 후 레이저 포즈 메시지 초기화
			geometry_msgs::PoseStamped laserAfterMappedPose;
			// 오도메트리 헤더 정보를 레이저 포즈 헤더에 복사
			laserAfterMappedPose.header = odomAftMapped.header;
			// 오도메트리 포즈 정보를 레이저 포즈에 복사
			laserAfterMappedPose.pose = odomAftMapped.pose.pose;
			// 매핑 후 레이저 경로의 헤더 설정
			laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
			laserAfterMappedPath.header.frame_id = "map";
			// 매핑 후 레이저 경로에 포즈 추가
			laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
			// 매핑 후 레이저 경로 메시지 발행
			pubLaserAfterMappedPath.publish(laserAfterMappedPath);
			
			// 매핑 후 포즈를 파일로 출력하기 위한 파일 스트림
			std::ofstream outPoses_;
			// 파일 열기
			outPoses_.open("/home/snow/polyWorkSpace/compare_ws/src/A-LOAM/resultsOutput/poses_aloam.txt");
			// 매핑 후 레이저 경로의 모든 포즈에 대해 반복
			for(size_t i=0; i< laserAfterMappedPath.poses.size(); i++ ){
			    auto odom_tmp = laserAfterMappedPath.poses[i];
			    // 파일에 타임스탬프와 포즈 정보 기록
			    outPoses_ << std::fixed << odom_tmp.header.stamp.toSec() << " "
			            << odom_tmp.pose.position.x << " "
			            << odom_tmp.pose.position.y << " "
			            << odom_tmp.pose.position.z << " "
			            << odom_tmp.pose.orientation.x << " "
			            << odom_tmp.pose.orientation.y << " "
			            << odom_tmp.pose.orientation.z << " "
			            << odom_tmp.pose.orientation.w << "\n";                 
			}
			// 파일 닫기
			outPoses_.close();

			
			// tf 변환 브로드캐스터 정적 객체 생성
			static tf::TransformBroadcaster br;
			// tf 변환 객체 생성
			tf::Transform transform;
			// tf 쿼터니언 객체 생성
			tf::Quaternion q;
			// 변환의 원점 설정
			transform.setOrigin(tf::Vector3(t_w_curr(0),
			                                t_w_curr(1),
			                                t_w_curr(2)));
			// 쿼터니언 값 설정
			q.setW(q_w_curr.w());
			q.setX(q_w_curr.x());
			q.setY(q_w_curr.y());
			q.setZ(q_w_curr.z());
			// 변환의 회전 설정
			transform.setRotation(q);
			// 변환 정보 전송
			br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "map", "/aft_mapped"));
			// 주석 처리된 부분: 포즈 파일 기록
			// if(outPoses_.is_open()){
			// 	outPoses_ << std::fixed << odomAftMapped.header.stamp.toSec() << " "
			// 			<< odomAftMapped.pose.pose.position.x << " "
			// 			<< odomAftMapped.pose.pose.position.y << " "
			// 			<< odomAftMapped.pose.pose.position.z << " "
			// 			<< odomAftMapped.pose.pose.orientation.x << " "
			// 			<< odomAftMapped.pose.pose.orientation.y << " "
			// 			<< odomAftMapped.pose.pose.orientation.z << " "
			// 			<< odomAftMapped.pose.pose.orientation.w << "\n"; 
			// }
			
			// 프레임 카운트 증가
			frameCount++;
			}
			// 2밀리초 동안 스레드 일시 정지
			std::chrono::milliseconds dura(2);
			std::this_thread::sleep_for(dura);
			}
			}


int main(int argc, char **argv)
{
	
	// ROS 초기화 및 노드 이름 설정
	ros::init(argc, argv, "laserMapping");
	// 노드 핸들러 생성
	ros::NodeHandle nh;
	
	// 선과 평면 해상도 파라미터 초기화
	float lineRes = 0;
	float planeRes = 0;
	// 선 해상도 설정
	nh.param<float>("mapping_line_resolution", lineRes, 0.4);
	// 평면 해상도 설정
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
	// 클라우드 토픽 이름 가져오기
	std::string CLOUD_TOPIC; 
	nh.getParam("/intensity_feature_tracker/cloud_topic", CLOUD_TOPIC); 
	// 해상도 값 출력
	printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
	// 필터 설정
	downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
	downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);
	
	// 레이저 클라우드 코너 및 서프 구독자 설정
	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudCornerLastHandler);
	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudSurfLastHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);
	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>(CLOUD_TOPIC, 100, laserCloudFullResHandler);
	
	// ROS 퍼블리셔 설정
	pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);
	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);
	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);
	
	// 레이저 클라우드 배열 초기화
	for (int i = 0; i < laserCloudNum; i++)
	{
	    laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
	    laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	}
	
	// 매핑 프로세스를 별도의 스레드에서 실행
	std::thread mapping_process{process};
	
	// ROS 이벤트 루프 시작
	ros::spin();
	
	return 0;
}

	
