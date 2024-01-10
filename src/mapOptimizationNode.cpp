// ROS에서 포인트 클라우드 데이터를 처리하기 위한 메시지 타입
#include <sensor_msgs/PointCloud2.h>

// ROS에서 오도메트리 데이터를 처리하기 위한 메시지 타입
#include <nav_msgs/Odometry.h>

// ROS에서 경로 데이터를 처리하기 위한 메시지 타입
#include <nav_msgs/Path.h>

// ROS에서 메시지 필터링을 위한 구독자를 만들기 위한 라이브러리
#include <message_filters/subscriber.h>

// ROS에서 다양한 메시지를 동기화하기 위한 라이브러리
#include <message_filters/synchronizer.h>

// ROS에서 시간 기반의 메시지 동기화를 위한 라이브러리
#include <message_filters/time_synchronizer.h>

// ROS에서 대략적인 시간 동기화 정책을 사용하기 위한 라이브러리
#include <message_filters/sync_policies/approximate_time.h>

// ROS에서 정확한 시간 동기화 정책을 사용하기 위한 라이브러리
#include <message_filters/sync_policies/exact_time.h>

// ROS 기본 헤더 파일
#include <ros/ros.h>

// Ceres 솔버 라이브러리, 비선형 최적화 문제 해결을 위한 라이브러리
#include <ceres/ceres.h>

// LiDAR 특징점 처리 함수를 위한 사용자 정의 헤더 파일
#include "lidarFeaturePointsFunction.hpp"

// 맵 최적화를 위한 사용자 정의 헤더 파일
#include "mapOptimization.hpp"

// 표준 입출력 스트림을 위한 라이브러리
#include <iostream>

// OpenMP 라이브러리, 병렬 처리를 위한 라이브러리
#include <omp.h>

// 메인 함수 정의
int main(int argc, char **argv)
{
    // ROS 초기화 및 노드 이름 설정
    ros::init(argc, argv, "map_optimization");
    // ROS 노드 핸들 생성
    ros::NodeHandle nh;

    // 전체 클라우드 토픽 이름을 저장할 문자열 변수 선언
    std::string CLOUD_TOPIC; // full cloud topic name
    // ROS 파라미터 서버에서 클라우드 토픽 이름 가져오기
    nh.getParam("/intensity_feature_tracker/cloud_topic", CLOUD_TOPIC);

    // 맵 최적화 객체 생성
    mapOptimization mapOptimization_;

    // 클라우드, 오도메트리, 평면 포인트 클라우드에 대한 구독자 설정
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, CLOUD_TOPIC, 20);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/laser_odom_to_init", 20);
    message_filters::Subscriber<sensor_msgs::PointCloud2> plane_sub(nh, "/laser_cloud_less_flat", 20);
    message_filters::Subscriber<sensor_msgs::PointCloud2> plane_search_sub(nh, "/laser_cloud_less_sharp", 20);

    // 동기화 정책을 위한 typedef 설정
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    // 다양한 메시지를 동기화하기 위한 Synchronizer 객체 생성
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, odom_sub, plane_sub, plane_search_sub);

    // 동기화된 메시지에 대한 콜백 함수 등록
    sync.registerCallback(boost::bind(&mapOptimization::mapOptimizationCallback, &mapOptimization_, _1, _2, _3, _4));

    // 레이저 오도메트리 데이터에 대한 구독자 설정 및 콜백 함수 연결
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 20, &mapOptimization::laserOdometryHandler, &mapOptimization_);

    // 최종 매핑된 데이터를 발행하기 위한 퍼블리셔 설정
    ros::Publisher pub_aft_mapped_to_init = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

    // 멀티스레드 스피너 생성 및 실행
    ros::MultiThreadedSpinner spinner(8);
    spinner.spin();

    return 0;
}
