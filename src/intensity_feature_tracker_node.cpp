#include "intensity_feature_tracker.h"
#include "image_handler.h"
#include "tic_toc.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>

// ImageHandler 및 feature_tracker 객체 선언
intensity_slam::ImageHandler *image_handler;
intensity_slam::feature_tracker *feature_tracker;

// 맵 오도메트리 콜백 함수
std::mutex mapOdomMutex;
void map_odom_callback(const nav_msgs::Odometry::ConstPtr &mapOdometry)
{
    mapOdomMutex.lock();
    feature_tracker->mapOdomQueue_.push(mapOdometry);
    mapOdomMutex.unlock();
}

// 포인트 클라우드 콜백 함수
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    TicToc covert_pointcloud2Intensity_time;

    // 병렬 처리 섹션 설정 (4개 스레드)
    #pragma omp parallel sections num_threads(4)
    {
        #pragma omp section
        image_handler->cloud_handler(cloud_msg); // 이미지 핸들링 (5ms)
    }
    ros::Time cloud_time = cloud_msg->header.stamp;
    TicToc detectFeatures_time;
    feature_tracker->detectfeatures(cloud_time, image_handler->image_intensity, image_handler->cloud_track, image_handler->GroundPointOut); // 특징 추출
}

int main(int argc, char **argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "intensity_feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // feature_tracker 객체 생성
    feature_tracker = new intensity_slam::feature_tracker(n);

    // 파라미터 로드
    std::cout << "read parameters" << std::endl;
    feature_tracker->readParameters();

    // ImageHandler 객체 생성
    image_handler = new intensity_slam::ImageHandler(feature_tracker->IMAGE_HEIGHT, feature_tracker->IMAGE_WIDTH, feature_tracker->NUM_THREADS);

    if (true)
    {
        std::cout << "start subscribe topic: " << feature_tracker->CLOUD_TOPIC << std::endl;

        // 포인트 클라우드 및 맵 오도메트리 콜백 함수 구독
        ros::Subscriber sub_point = n.subscribe(feature_tracker->CLOUD_TOPIC, 1, cloud_callback);
        ros::Subscriber sub_map_path = n.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100, map_odom_callback);

        // 멀티 스레드 스피너 시작
        ros::MultiThreadedSpinner spinner(8);
        spinner.spin();
    }

    return 0;
}
