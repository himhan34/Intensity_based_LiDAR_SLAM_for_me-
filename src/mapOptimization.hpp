// 중복 포함을 방지하기 위한 전처리기 지시문
#pragma once

// ROS 메시지 타입들을 위한 헤더 파일 포함

// 센서 메시지의 포인트 클라우드 데이터 처리를 위한 헤더 파일
#include <sensor_msgs/PointCloud2.h>  

// 오도메트리 데이터 처리를 위한 헤더 파일
#include <nav_msgs/Odometry.h>      

// 경로 데이터 처리를 위한 헤더 파일
#include <nav_msgs/Path.h>            

// Eigen 라이브러리 포함
// Eigen 라이브러리 주석 처리된 구버전 포함문
// #include <Eigen/Dense>       

// Eigen 라이브러리 최신 버전 포함문
#include <eigen3/Eigen/Dense>        

// OpenMP 라이브러리 포함
// 병렬 처리를 위한 OpenMP 라이브러리
#include <omp.h>                     

// 사용자 정의 헤더 파일 포함

 // 이미지 핸들러 클래스
#include "image_handler.h"    

// 시간 측정을 위한 TicToc 클래스
#include "tic_toc.h"   

// 키 프레임 처리를 위한 클래스
#include "keyframe.h"        

 // ROS 기하 메시지의 포인트 타입
#include <geometry_msgs/Point.h>     

// 표준 라이브러리 데크 컨테이너
#include <deque>                      

// 사용자 정의 ikd-Tree 라이브러리
#include "ikd-Tree/ikd_Tree.h"        

// PCL 라이브러리의 사전 컴파일 방지 정의
#define PCL_NO_PRECOMPILE

template <typename Derived>
static void reduceVector(std::vector<Derived> &v, std::vector<uchar> status)
{
    int j = 0; // 새로운 벡터의 인덱스를 추적하는 변수 j를 초기화합니다.
    for (int i = 0; i < int(v.size()); i++) // 기존 벡터 v의 모든 원소를 확인합니다.
        if (status[i]) // 만약 status 벡터에서 i번째 원소가 참(1)이라면 다음을 수행합니다.
            v[j++] = v[i]; // 새로운 벡터 v의 j번째 위치에 현재 원소를 복사하고, j를 증가시킵니다.
    v.resize(j); // 새로운 크기로 벡터 v를 재조정하여 불필요한 원소를 제거합니다.
}

class mapOptimization
{
public:
    mapOptimization(); // 생성자
    ~mapOptimization(); // 소멸자
    void mapOptimizationCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& plane_cloud_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_corner_msg); // 맵 최적화 콜백 함수
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry); // 레이저 오도메트리 핸들러
    cv::Mat setMask(int height, int width, int crop); // 마스크 설정 함수
    void keypoint2uv(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& uv); // 키포인트를 이미지 좌표로 변환하는 함수
    void extractPointsAndFilterZeroValue(const std::vector<cv::Point2f>& cur_orb_point_2d_uv_,  const pcl::PointCloud<PointType>::Ptr& cloudTrack_, std::vector<cv::Point3f>& cur_out_point3d_, std::vector<uchar>& status_); // 포인트 추출 및 값이 0인 포인트 필터링 함수
    void extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> &cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d, std_msgs::Header header); // 매칭된 포인트 추출 함수
    void extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, std::vector<cv::Point3f> &cur_matched_points3d,  std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d); // 매칭된 포인트 추출 함수
    void transformUpdate(); // 변환 업데이트 함수
    void transformAssociateToMap(); // 맵에 연관된 변환 함수
    void calculateAverageDistance(double &avg_distance, std::vector<cv::Point3f> prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t); // 평균 거리 계산 함수
    void appendLines(visualization_msgs::Marker &line_list, geometry_msgs::Point p1, geometry_msgs::Point p2); // 라인 추가 함수
    void initialLineList(visualization_msgs::Marker &line_list, std_msgs::Header header); // 초기 라인 목록 설정 함수
    void image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_); // 이미지 표시 함수
    void filterNaNPoints(); // NaN 포인트 필터링 함수
    intensity_slam::ImageHandler *image_handler_; // 이미지 핸들러 포인터
    std::map<size_t, mapProcessor::Keyframe> keyframe_pool_; // 키프레임 풀 맵
    std::deque<std::shared_ptr<mapProcessor::SlideWindowKeyframe>> keyframe_sliding_window_; // 슬라이딩 윈도우 키프레임 덱
    mapProcessor::Keyframe prev_keyframe, cur_keyframe; // 이전 키프레임과 현재 키프레임

private:
    size_t keyframeId_; // 키프레임 ID
    int IMAGE_WIDTH; // 이미지 폭
    int IMAGE_HEIGHT; // 이미지 높이
    int IMAGE_CROP; // 이미지 자르기 크기
    int NUM_THREADS; // 스레드 개수
    int NUM_ORB_FEATURES; // ORB 특징점 개수
    bool HAND_HELD_FLAG; // 핸드헬드 플래그
    int SLIDING_WINDOW_SIZE; // 슬라이딩 윈도우 크기
    int GROUND_PLANE_WINDOW_SIZE; // 지면 평면 윈도우 크기
    Eigen::Quaterniond q_wmap_wodom; // 월드 맵에서 월드 오도메트리로의 회전
    Eigen::Vector3d t_wmap_wodom; // 월드 맵에서 월드 오도메트리로의 이동
    Eigen::Quaterniond q_wodom_curr; // 월드 오도메트리에서 현재 위치로의 회전
    Eigen::Vector3d t_wodom_curr; // 월드 오도메트리에서 현재 위치로의 이동
    ros::Publisher pubOdomAftMappedHighFrec; // 매핑 후 고주파 오도메트리 발행자
    ros::Publisher robot_marker_pub; // 로봇 마커 발행자
    ros::Publisher matched_points_pub; // 매칭된 포인트 발행자
    std::vector<cv::KeyPoint> prev_keypoints_, cur_keypoints_, cur_keypoints_tmp_; // 이전 및 현재 키포인트
    std::vector<cv::Point2f>  prev_orb_point_2d_uv_, cur_orb_point_2d_uv_, cur_orb_point_2d_uv_tmp_; // 이전 및 현재 ORB 포인트 2D 좌표
    std::vector<cv::Point3f>  prev_out_point3d_, cur_out_point3d_, cur_out_point3d_tmp_; // 이전 및 현재 포인트 3D 좌표
    std::vector<uchar> status_; // 상태 정보
    cv::Mat MASK; // 마스크 이미지
    cv::Mat prev_descriptors_, cur_descriptors_, cur_descriptors_tmp_; // 이전 및 현재 디스크립터
    cv::Mat prev_keyframe_img, cur_keyframe_img; // 이전 및 현재 키프레임 이미지
    cv::BFMatcher matcher_; // Brute-Force 매처
    bool keyframe_flag_; // 키프레임 플래그
    double parameters_[7] = {0, 0, 0, 1, 0, 0, 0}; // 매개변수 배열
    std::vector<cv::Point3f> prev_matched_points3d_, cur_matched_points3d_; // 이전 및 현재 매칭된 포인트 3D 좌표
    visualization_msgs::Marker line_list; // 라인 목록
    ros::Publisher matched_lines_pub; // 매칭된 라인 발행자
    ros::Publisher matched_keypoints_img_pub, ground_points_pub; // 매칭된 키포인트 이미지 발행자, 지면 포인트 발행자
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_, corner_voxel_grid_; // 복셀 그리드
    pcl::PointXYZ ground_point_sensor_, ground_point_world_; // 지면 포인트 (센서 및 월드 좌표계)
    std::mutex map_mutex_; // 맵 뮤텍스
    KD_TREE<GroundPlanePointType>::Ptr ikdtree, corner_ikdtree_; // KD 트리 포인터 (지면 및 코너)
    pcl::PointCloud<pcl::PointXYZ> ground_plane_cloud_; // 지면 평면 클라우드
};



