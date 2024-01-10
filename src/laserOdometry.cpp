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

// 기본적인 수학 함수들을 사용하기 위한 표준 라이브러리
#include <cmath>
// ROS의 오도메트리 메시지를 사용하기 위한 헤더 파일
#include <nav_msgs/Odometry.h>
// ROS의 경로 메시지를 사용하기 위한 헤더 파일
#include <nav_msgs/Path.h>
// ROS의 포즈 스탬프 메시지를 사용하기 위한 헤더 파일
#include <geometry_msgs/PoseStamped.h>
// PCL(Point Cloud Library)의 포인트 클라우드와 관련된 헤더 파일
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL의 보켈 그리드 필터를 사용하기 위한 헤더 파일
#include <pcl/filters/voxel_grid.h>
// PCL의 KD-트리 탐색을 위한 헤더 파일
#include <pcl/kdtree/kdtree_flann.h>
// PCL과 ROS 간의 변환을 위한 헤더 파일
#include <pcl_conversions/pcl_conversions.h>
// ROS 기능을 사용하기 위한 헤더 파일
#include <ros/ros.h>
// ROS의 IMU(관성 측정 장치) 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/Imu.h>
// ROS의 포인트 클라우드 메시지를 사용하기 위한 헤더 파일
#include <sensor_msgs/PointCloud2.h>
// tf 변환 데이터 타입을 사용하기 위한 헤더 파일
#include <tf/transform_datatypes.h>
// tf 변환 브로드캐스터를 사용하기 위한 헤더 파일
#include <tf/transform_broadcaster.h>
// Eigen 라이브러리의 밀집 행렬과 배열을 사용하기 위한 헤더 파일
#include <eigen3/Eigen/Dense>
// 멀티스레딩 관련 기능을 위한 표준 뮤텍스 헤더
#include <mutex>
// 표준 큐를 사용하기 위한 헤더 파일
#include <queue>

// 사용자 정의 파라미터 헤더 파일
#include "parameters.h"
// 라이다 특징점 추출 함수를 위한 헤더 파일
#include "lidarFeaturePointsFunction.hpp"
// 시간 측정을 위한 사용자 정의 클래스
#include "tic_toc.h"
// 이미지 추적과 관련된 기능이 주석 처리됨
// #include "intensity_feature_tracker.h"
// #include "image_handler.h"

// 왜곡 플래그 정의, 현재는 사용하지 않음 (0)
#define DISTORTION 0

// 코너와 평면에 대한 대응 개수를 카운트하는 변수
int corner_correspondence = 0, plane_correspondence = 0;

// 스캔 주기, 거리 임계값, 인접 스캔 범위에 대한 상수 선언
constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

// 프레임을 건너뛰는 숫자 설정
int skipFrameNum = 5;
// 시스템 초기화 상태 표시 변수
bool systemInited = false;

// 다양한 포인트 클라우드 유형에 대한 시간 변수 선언
double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

// 코너와 서피스 포인트에 대한 KD-트리 선언
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

// 스킵 플래그 문자열 변수
std::string skip_flag; 
// 다양한 포인트 클라우드 유형에 대한 포인터 선언
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

// 마지막 스캔의 코너, 서피스, 전체 해상도 포인트 클라우드 포인터
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// 마지막 스캔의 코너 및 서피스 포인트 클라우드 개수 변수
int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// 현재 프레임에서 월드 프레임으로의 변환을 위한 쿼터니언과 벡터
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w)와 t_curr_last를 위한 배열
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

// 배열을 기반으로 한 Eigen 쿼터니언과 벡터의 맵핑
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

// 다양한 유형의 포인트 클라우드를 저장하기 위한 큐
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
// 큐에 대한 스레드 안전 접근을 위한 뮤텍스
std::mutex mBuf;

// 라이다 포인트를 이전 프레임에서 현재 프레임의 시작으로 변환하는 함수
void TransformToStart(PointType const *const pi, PointType *const po)
{
    // 보간 비율
    double s;
    // 왜곡이 있는 경우, 인텐시티를 기반으로 보간 비율 계산
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0; // 왜곡이 없는 경우, 보간 비율을 1로 설정
    //s = 1; // 주석 처리된 대체 보간 비율 설정

    // 보간 비율을 기반으로 한 쿼터니언 계산
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    // 보간 비율을 기반으로 한 변환 벡터 계산
    Eigen::Vector3d t_point_last = s * t_last_curr;
    // 입력 포인트를 벡터로 변환
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    // 변환된 포인트 계산
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    // 출력 포인트에 변환된 좌표와 인텐시티 할당
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// 모든 라이다 포인트를 다음 프레임의 시작으로 변환
// 라이다 포인트를 이전 프레임에서 현재 프레임의 끝으로 변환하는 함수
void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // 먼저 포인트 왜곡 제거
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    // 변환된 포인트를 벡터로 변환
    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    // 현재 프레임의 끝으로 변환된 포인트 계산
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    // 출력 포인트에 변환된 좌표 할당
    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    // 왜곡 시간 정보 제거
    po->intensity = int(pi->intensity);
}

// 라이다 클라우드의 '날카로운' 포인트에 대한 핸들러 함수
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    // 스레드 안전을 위해 뮤텍스 잠금
    mBuf.lock();
    // '날카로운' 포인트 큐에 추가
    cornerSharpBuf.push(cornerPointsSharp2);
    // 뮤텍스 잠금 해제
    mBuf.unlock();
}

// 라이다 클라우드의 '덜 날카로운' 포인트에 대한 핸들러 함수
void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    // 스레드 안전을 위해 뮤텍스 잠금
    mBuf.lock();
    // '덜 날카로운' 포인트 큐에 추가
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    // 뮤텍스 잠금 해제
    mBuf.unlock();
}


// 평평한 라이다 포인트 클라우드를 처리하는 핸들러 함수
void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    // 스레드 안전을 위해 뮤텍스 잠금
    mBuf.lock();
    // 평평한 포인트 클라우드를 버퍼에 추가
    surfFlatBuf.push(surfPointsFlat2);
    // 뮤텍스 잠금 해제
    mBuf.unlock();
}

// 덜 평평한 라이다 포인트 클라우드를 처리하는 핸들러 함수
void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    // 스레드 안전을 위해 뮤텍스 잠금
    mBuf.lock();
    // 덜 평평한 포인트 클라우드를 버퍼에 추가
    surfLessFlatBuf.push(surfPointsLessFlat2);
    // 뮤텍스 잠금 해제
    mBuf.unlock();
}

// 모든 라이다 포인트 클라우드를 받는 핸들러 함수
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    // 스레드 안전을 위해 뮤텍스 잠금
    mBuf.lock();
    // 전체 포인트 클라우드를 버퍼에 추가
    fullPointsBuf.push(laserCloudFullRes2);
    // 뮤텍스 잠금 해제
    mBuf.unlock();
}




// argc: 명령행 매개변수의 개수, argv: 명령행 매개변수의 배열
int main(int argc, char **argv)
{
    // ROS 노드를 초기화합니다.
    ros::init(argc, argv, "laserOdometry");
    
    // ROS 노드 핸들을 생성합니다.
    ros::NodeHandle nh;

    // "mapping_skip_frame" 매개변수를 읽고 skipFrameNum 변수에 저장합니다.
    nh.param<int>("mapping_skip_frame", skipFrameNum, 1);

    // ROS 토픽 "/laser_cloud_sharp"을 구독하는 subscriber를 생성합니다.
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);

    // ROS 토픽 "/laser_cloud_less_sharp"을 구독하는 subscriber를 생성합니다.
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);

    // ROS 토픽 "/laser_cloud_flat"을 구독하는 subscriber를 생성합니다.
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);

    // ROS 토픽 "/laser_cloud_less_flat"을 구독하는 subscriber를 생성합니다.
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);

    // ROS 토픽 "/velodyne_cloud_2"을 구독하는 subscriber를 생성합니다.
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    // ROS 토픽 "/laser_cloud_corner_last"에 데이터를 발행하는 publisher를 생성합니다.
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);

    // ROS 토픽 "/laser_cloud_surf_last"에 데이터를 발행하는 publisher를 생성합니다.
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);

    // ROS 토픽 "/velodyne_cloud_3"에 데이터를 발행하는 publisher를 생성합니다.
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);

    // ROS 토픽 "/laser_odom_to_init_aloam"에 데이터를 발행하는 publisher를 생성합니다.
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init_aloam", 100);

    // ROS 토픽 "/laser_odom_path"에 데이터를 발행하는 publisher를 생성합니다.
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    // Laser 경로를 저장하기 위한 Path 메시지를 생성합니다.
    nav_msgs::Path laserPath;

    // 프레임 카운트를 초기화합니다.
    int frameCount = 0;

    // 주기적으로 실행할 레이트(주행 속도)를 설정합니다.
    ros::Rate rate(100);

    // ROS가 정상적으로 동작 중인 동안 반복합니다.
    while (ros::ok())
    {
        // ROS 스핀을 실행하여 콜백 함수가 호출되도록 합니다.
        ros::spinOnce();
    
        // 버퍼들이 비어있지 않고 모든 센서 데이터가 존재할 경우 아래 코드를 실행합니다.
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            // 각 센서 데이터의 타임스탬프를 가져옵니다.
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
    
            // 모든 센서 데이터의 타임스탬프가 같은지 확인합니다.
            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                // 만약 타임스탬프가 동기화되지 않았다면 에러 메시지를 출력하고 프로그램을 종료합니다.
                printf("타임스탬프가 동기화되지 않았습니다!");
                ROS_BREAK();
            }

            // 버퍼를 잠금 상태로 설정합니다.
            mBuf.lock();
            // sharp 모드에서 사용할 cornerPointsSharp 버퍼를 비웁니다.
            cornerPointsSharp->clear();
            // ROS 메시지 형태의 데이터를 pcl 포인트 클라우드로 변환합니다.
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            // skip_flag 변수에 현재 cornerSharpBuf의 프레임 ID를 저장합니다.
            skip_flag = cornerSharpBuf.front()->header.frame_id;
            // 로그로 skip_flag 값을 출력합니다.
            ROS_INFO("skip_flag: %s", skip_flag.c_str());
            // cornerSharpBuf의 첫 번째 메시지를 삭제합니다.
            cornerSharpBuf.pop();
            
            // less sharp 모드에서 사용할 cornerPointsLessSharp 버퍼를 비웁니다.
            cornerPointsLessSharp->clear();
            // ROS 메시지 형태의 데이터를 pcl 포인트 클라우드로 변환합니다.
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            // cornerLessSharpBuf의 첫 번째 메시지를 삭제합니다.
            cornerLessSharpBuf.pop();
            
            // flat 모드에서 사용할 surfPointsFlat 버퍼를 비웁니다.
            surfPointsFlat->clear();
            // ROS 메시지 형태의 데이터를 pcl 포인트 클라우드로 변환합니다.
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            // surfFlatBuf의 첫 번째 메시지를 삭제합니다.
            surfFlatBuf.pop();
            
            // less flat 모드에서 사용할 surfPointsLessFlat 버퍼를 비웁니다.
            surfPointsLessFlat->clear();
            // ROS 메시지 형태의 데이터를 pcl 포인트 클라우드로 변환합니다.
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            // surfLessFlatBuf의 첫 번째 메시지를 삭제합니다.
            surfLessFlatBuf.pop();
            
            // full resolution 모드에서 사용할 laserCloudFullRes 버퍼를 비웁니다.
            laserCloudFullRes->clear();
            // ROS 메시지 형태의 데이터를 pcl 포인트 클라우드로 변환합니다.
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            // fullPointsBuf의 첫 번째 메시지를 삭제합니다.
            fullPointsBuf.pop();
            
            // 버퍼의 잠금을 해제합니다.
            mBuf.unlock();
            // TicToc 객체를 생성하여 시간 측정을 시작합니다.
            TicToc t_whole;
            
            // 시스템이 초기화되지 않은 경우 초기화를 수행합니다.
            if (!systemInited)
            {
                // 시스템이 처음 초기화되었음을 표시합니다.
                systemInited = true;
            
                // "Initialization finished" 메시지를 출력합니다.
                std::cout << "초기화가 완료되었습니다." << std::endl;
            }
                
            else
            {
                // cornerPointsSharp 버퍼의 포인트 수를 계산합니다.
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
            
                // surfPointsFlat 버퍼의 포인트 수를 계산합니다.
                int surfPointsFlatNum = surfPointsFlat->points.size();
            
                // TicToc 객체를 생성하여 최적화 시간 측정을 시작합니다.
                TicToc t_opt;
            
                // use_aloam 변수를 선언하고 초기화합니다.
                bool use_aloam;
            
                // skip_flag가 "skip_intensity"인 경우 use_aloam을 true로 설정합니다.
                if (skip_flag == "skip_intensity")
                {
                    use_aloam = true;
                }
                else
                {
                    // 그렇지 않으면 use_aloam을 false로 설정합니다.
                    use_aloam = false;
                }

                // 최적화 반복을 2번 수행하고, use_aloam이 true인 경우에만 실행합니다.
                for (size_t opti_counter = 0; opti_counter < 2 && use_aloam; ++opti_counter)
                {
                    // corner_correspondence과 plane_correspondence 변수를 초기화합니다.
                    corner_correspondence = 0;
                    plane_correspondence = 0;
                
                    // ceres 라이브러리의 HuberLoss를 사용하여 손실 함수를 설정합니다.
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                
                    // EigenQuaternionParameterization을 사용하여 quaternion 파라미터화를 설정합니다.
                    ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization();
                
                    // ceres::Problem::Options 객체를 생성합니다.
                    ceres::Problem::Options problem_options;
                
                    // ceres::Problem 객체를 생성하고 파라미터 블록을 추가합니다.
                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(para_q, 4, q_parameterization); // quaternion 파라미터 블록 추가
                    problem.AddParameterBlock(para_t, 3); // translation 파라미터 블록 추가
                
                    // PCL(Point Cloud Library)을 사용하여 포인트 클라우드와 관련된 변수를 초기화합니다.
                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;
                
                    // 데이터 처리 시간을 측정하기 위해 TicToc 객체를 생성합니다.
                    TicToc t_data;
                    // corner 포인트들에 대한 대응점을 찾습니다.
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        // 현재 corner 포인트를 시작 포인트로 변환합니다.
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                    
                        // Kd-Tree를 사용하여 가장 가까운 포인트를 검색합니다.
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                    
                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            // 가장 가까운 포인트의 인덱스를 가져옵니다.
                            closestPointInd = pointSearchInd[0];
                    
                            // 가장 가까운 포인트의 스캔 ID를 추출합니다.
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);
                    
                            // 두 번째 가장 가까운 포인트까지의 거리를 초기화합니다.
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                    
                            // 스캔 라인 증가 방향으로 검색합니다.
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // 같은 스캔 라인에 있는 경우, 계속합니다.
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;
                    
                                // 인접한 스캔 라인보다 멀리 떨어진 경우, 루프를 종료합니다.
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                    
                                // 포인트 간의 거리를 계산합니다.
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);
                    
                                if (pointSqDis < minPointSqDis2)
                                {
                                    // 더 가까운 포인트를 찾습니다.
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }



                        // 스캔 라인 감소 방향으로 검색합니다.
                    for (int j = closestPointInd - 1; j >= 0; --j)
                    {
                        // 같은 스캔 라인에 있는 경우, 계속합니다.
                        if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                            continue;
                    
                        // 인접한 스캔 라인보다 멀리 떨어진 경우, 루프를 종료합니다.
                        if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                            break;
                    
                        // 포인트 간의 거리를 계산합니다.
                        double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                (laserCloudCornerLast->points[j].x - pointSel.x) +
                                            (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                (laserCloudCornerLast->points[j].y - pointSel.y) +
                                            (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                (laserCloudCornerLast->points[j].z - pointSel.z);
                    
                        if (pointSqDis < minPointSqDis2)
                        {
                            // 더 가까운 포인트를 찾습니다.
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }
                }








                // minPointInd2가 유효한 경우에 실행합니다.
                if (minPointInd2 >= 0) // closestPointInd와 minPointInd2 모두 유효한 경우
                {
                    // 현재 corner 포인트의 좌표를 Eigen 벡터로 저장합니다.
                    Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                               cornerPointsSharp->points[i].y,
                                               cornerPointsSharp->points[i].z);
                
                    // 가장 가까운 포인트의 좌표를 Eigen 벡터로 저장합니다.
                    Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                 laserCloudCornerLast->points[closestPointInd].y,
                                                 laserCloudCornerLast->points[closestPointInd].z);
                
                    // 두 번째로 가까운 포인트의 좌표를 Eigen 벡터로 저장합니다.
                    Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                 laserCloudCornerLast->points[minPointInd2].y,
                                                 laserCloudCornerLast->points[minPointInd2].z);
                
                    // DISTORTION 옵션이 활성화되어 있다면, s 값을 계산합니다.
                    double s;
                    if (DISTORTION)
                        s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                    else
                        s = 1.0;
                
                    // LidarEdgeFactor를 사용하여 비용 함수를 생성합니다.
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                
                    // 문제에 비용 함수를 추가합니다.
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                
                    // corner 포인트 대응 개수를 증가시킵니다.
                    corner_correspondence++;
                }

            }

        // 평면 특징에 대한 대응점을 찾습니다.
            for (int i = 0; i < surfPointsFlatNum; ++i)
            {
                // 현재 surf 포인트를 시작 포인트로 변환합니다.
                TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
            
                // Kd-Tree를 사용하여 가장 가까운 포인트를 검색합니다.
                kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            
                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                {
                    // 가장 가까운 포인트의 인덱스를 가져옵니다.
                    closestPointInd = pointSearchInd[0];
            
                    // 가장 가까운 포인트의 스캔 ID를 추출합니다.
                    int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
            
                    // 두 번째와 세 번째 가장 가까운 포인트까지의 거리를 초기화합니다.
                    double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
            
                    // 스캔 라인 증가 방향으로 검색합니다.
                    for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                    {
                        // 인접한 스캔 라인보다 멀리 떨어진 경우, 루프를 종료합니다.
                        if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                            break;
            
                        // 포인트 간의 거리를 계산합니다.
                        double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                (laserCloudSurfLast->points[j].x - pointSel.x) +
                                            (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                (laserCloudSurfLast->points[j].y - pointSel.y) +
                                            (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                (laserCloudSurfLast->points[j].z - pointSel.z);


                        // 동일한 또는 낮은 스캔 라인에 있는 경우, 가장 가까운 포인트를 찾습니다.
                        if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        // 높은 스캔 라인에 있는 경우, 두 번째로 가까운 포인트를 찾습니다.
                        else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                        {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                            }

                    // 스캔 라인 감소 방향으로 검색합니다.
                    for (int j = closestPointInd - 1; j >= 0; --j)
                    {
                        // 인접한 스캔 라인보다 멀리 떨어진 경우, 루프를 종료합니다.
                        if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                            break;
                    
                        // 포인트 간의 거리를 계산합니다.
                        double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                (laserCloudSurfLast->points[j].x - pointSel.x) +
                                            (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                (laserCloudSurfLast->points[j].y - pointSel.y) +
                                            (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                (laserCloudSurfLast->points[j].z - pointSel.z);
                    
                        // 동일한 또는 높은 스캔 라인에 있는 경우, 가장 가까운 포인트를 찾습니다.
                        if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        // 낮은 스캔 라인에 있는 경우, 두 번째로 가까운 포인트를 찾습니다.
                        else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                        {
                            // 더 가까운 포인트를 찾습니다.
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }

                    // minPointInd2와 minPointInd3가 모두 유효한 경우에 실행합니다.
                    if (minPointInd2 >= 0 && minPointInd3 >= 0)
                    {
                        // 현재 surf 포인트의 좌표를 Eigen 벡터로 저장합니다.
                        Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                    surfPointsFlat->points[i].y,
                                                    surfPointsFlat->points[i].z);
                    
                        // 가장 가까운 포인트의 좌표를 Eigen 벡터로 저장합니다.
                        Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                    laserCloudSurfLast->points[closestPointInd].y,
                                                    laserCloudSurfLast->points[closestPointInd].z);
                    
                        // 두 번째로 가까운 포인트의 좌표를 Eigen 벡터로 저장합니다.
                        Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                    laserCloudSurfLast->points[minPointInd2].y,
                                                    laserCloudSurfLast->points[minPointInd2].z);
                    
                        // 세 번째로 가까운 포인트의 좌표를 Eigen 벡터로 저장합니다.
                        Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                    laserCloudSurfLast->points[minPointInd3].y,
                                                    laserCloudSurfLast->points[minPointInd3].z);
                    
                        // DISTORTION 옵션이 활성화되어 있다면, s 값을 계산합니다.
                        double s;
                        if (DISTORTION)
                            s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                        else
                            s = 1.0;
                    
                        // LidarPlaneFactor를 사용하여 비용 함수를 생성합니다.
                        ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    
                        // 문제에 비용 함수를 추가합니다.
                        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                    
                        // plane 포인트 대응 개수를 증가시킵니다.
                        plane_correspondence++;
                    }

                        }
                    }

                    // 주석 처리된 부분은 출력문으로 주석처리되어 있으므로 주석을 해제하면 출력 가능합니다.
                    /*
                    printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());
                    */
                    
                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        // 대응 포인트 개수가 일정 임계값 미만인 경우, 이를 출력합니다.
                        // printf("less correspondence! *************************************************\n");
                    }
                    
                    // 최적화 문제를 해결하는 데 사용되는 Ceres Solver의 설정을 정의합니다.
                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR; // 선형 솔버 유형을 설정합니다.
                    options.max_num_iterations = 4; // 최대 반복 횟수를 설정합니다.
                    options.minimizer_progress_to_stdout = false; // 최적화 진행 상황을 표시하지 않도록 설정합니다.
                    ceres::Solver::Summary summary; // 최적화 결과 요약 정보를 저장할 변수를 선언합니다.
                    ceres::Solve(options, &problem, &summary); // 최적화 문제를 해결합니다.
                    printf("geometry features solver time %f ms \n", t_solver.toc()); // 최적화 시간을 출력합니다.

                }
                // printf("optimization twice time %f \n", t_opt.toc());

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;
            }


            // 시간 측정을 위한 TicToc 객체를 생성합니다.
            TicToc t_pub;
            
            // 오도메트리 정보를 발행합니다.
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "camera_init";
            laserOdometry.child_frame_id = "/laser_odom";
            
            // 오도메트리 메시지의 타임스탬프를 설정합니다.
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            
            // 오도메트리 메시지의 자세 정보(orientation)를 설정합니다.
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            
            // 오도메트리 메시지의 위치 정보(position)를 설정합니다.
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            
            // 오도메트리 메시지를 발행합니다.
            pubLaserOdometry.publish(laserOdometry);
            
            // geometry_msgs::PoseStamped 형태의 laserPose 메시지를 생성하고 오도메트리 정보를 설정합니다.
            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            
            // laserPose 메시지를 laserPath에 추가합니다.
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "camera_init";
            
            // laserPath 메시지를 발행합니다.
            pubLaserPath.publish(laserPath);



            // corner features와 plane features를 스캔의 끝점(end point)으로 변환합니다.
            if (0)
            {
                // cornerPointsLessSharp의 포인트 수를 가져옵니다.
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
            
                // cornerPointsLessSharp의 모든 포인트를 스캔의 끝점으로 변환합니다.
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }
            
                // surfPointsLessFlat의 포인트 수를 가져옵니다.
                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
            
                // surfPointsLessFlat의 모든 포인트를 스캔의 끝점으로 변환합니다.
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }
            
                // laserCloudFullRes의 포인트 수를 가져옵니다.
                int laserCloudFullResNum = laserCloudFullRes->points.size();
            
                // laserCloudFullRes의 모든 포인트를 스캔의 끝점으로 변환합니다.
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }
            
            // 포인트 클라우드 교체를 위한 임시 포인트 클라우드(laserCloudTemp)를 생성하고, cornerPointsLessSharp와 교체합니다.
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;
            
            // 포인트 클라우드 교체를 위한 임시 포인트 클라우드(laserCloudTemp)를 생성하고, surfPointsLessFlat와 교체합니다.
            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;
            
            // 교체된 포인트 클라우드의 포인트 개수를 업데이트합니다.
            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();
            
            // k-d 트리(kdtree)를 업데이트하여 교체된 포인트 클라우드에 대한 새로운 입력을 설정합니다.
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);



            // 프레임 카운트(frameCount)가 일정 간격(skipFrameNum)마다 실행됩니다.
            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;
            
                // laserCloudCornerLast 포인트 클라우드를 ROS 메시지로 변환하고 발행합니다.
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);
            
                // laserCloudSurfLast 포인트 클라우드를 ROS 메시지로 변환하고 발행합니다.
                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
            
                // laserCloudFullRes 포인트 클라우드를 ROS 메시지로 변환하고 발행합니다.
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            
            // 프레임 카운트를 증가시킵니다.
            frameCount++;
        }
        rate.sleep();
    }
    return 0;
}
