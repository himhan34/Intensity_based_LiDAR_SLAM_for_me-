#pragma once // 이 헤더 파일을 중복 포함 방지하기 위한 전처리기 지시문

#include "parameters.h" // "parameters.h" 헤더 파일 포함

#include <ros/ros.h> // ROS 라이브러리 포함
#include <opencv2/opencv.hpp> // OpenCV 라이브러리 포함
#include <iostream> // 입력 및 출력 스트림 지원 라이브러리 포함
#include "tic_toc.h" // 시간 측정 도구 관련 헤더 파일 포함
#include <omp.h> // OpenMP(병렬 프로그래밍) 지원 라이브러리 포함
#include <pcl/visualization/cloud_viewer.h> // PCL(점군 처리 라이브러리) 시각화 관련 헤더 파일 포함
#include "keyframe.h" // "keyframe.h" 헤더 파일 포함
#include "loop_closure_handler.h" // 루프 클로저 처리 관련 헤더 파일 포함

#include <gtsam/geometry/Rot3.h> // GTSAM 라이브러리의 Rot3 클래스 관련 헤더 파일 포함
#include <gtsam/geometry/Pose3.h> // GTSAM 라이브러리의 Pose3 클래스 관련 헤더 파일 포함
#include <gtsam/slam/PriorFactor.h> // GTSAM 라이브러리의 PriorFactor 클래스 관련 헤더 파일 포함
#include <gtsam/slam/BetweenFactor.h> // GTSAM 라이브러리의 BetweenFactor 클래스 관련 헤더 파일 포함
#include <gtsam/nonlinear/NonlinearFactorGraph.h> // GTSAM 라이브러리의 NonlinearFactorGraph 클래스 관련 헤더 파일 포함
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h> // GTSAM 라이브러리의 LevenbergMarquardtOptimizer 클래스 관련 헤더 파일 포함
#include <gtsam/nonlinear/Marginals.h> // GTSAM 라이브러리의 Marginals 클래스 관련 헤더 파일 포함
#include <gtsam/nonlinear/Values.h> // GTSAM 라이브러리의 Values 클래스 관련 헤더 파일 포함

#include <gtsam/nonlinear/ISAM2.h> // GTSAM 라이브러리의 ISAM2 클래스 관련 헤더 파일 포함

#include <deque> // 덱(Deque) 컨테이너 관련 헤더 파일 포함
#include <queue> // 큐(Queue) 컨테이너 관련 헤더 파일 포함

#include <iostream> // 입력 및 출력 스트림 지원 라이브러리 포함
#include <fstream> // 파일 입출력 관련 헤더 파일 포함
#include <stdlib.h> // 표준 라이브러리 관련 헤더 파일 포함

#include <tf2_ros/transform_listener.h> // TF2 ROS 패키지의 Transform Listener 관련 헤더 파일 포함
#include <geometry_msgs/TransformStamped.h> // ROS의 TransformStamped 메시지 관련 헤더 파일 포함
#include <geometry_msgs/Twist.h> // ROS의 Twist 메시지 관련 헤더 파일 포함

namespace intensity_slam {

// feature_tracker 클래스 선언
class feature_tracker {
public:
    // 생성자: ROS 노드 핸들을 인자로 받음
    feature_tracker(ros::NodeHandle& n);

    // 소멸자
    ~feature_tracker();

    // 매개변수를 읽어오는 함수
    void readParameters();

    // 특징점을 감지하고 처리하는 함수
    void detectfeatures(ros::Time &time, 
                        const cv::Mat &image_intensity, 
                        const pcl::PointCloud<PointType>::Ptr cloud_track,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoint_);

    // 코너 특징점을 감지하고 처리하는 함수
    void detectcorners(ros::Time &time, 
                        const cv::Mat &image_intensity, 
                        const pcl::PointCloud<PointType>::Ptr cloud_track);

    // 마스크를 설정하는 함수
    void setMask();

    // 키포인트를 이미지 좌표로 변환하는 함수
    void keypoint2uv();

    // 키포인트를 이미지 좌표로 변환하는 함수
    std::vector<cv::Point2f> keypoint2uv(std::vector<cv::KeyPoint> cur_keypoints);

    // 포인트를 추출하고 값이 0인 포인트를 필터링하는 함수
    void extractPointsAndFilterZeroValue();

    // 포인트를 추출하고 값이 0인 포인트를 필터링하는 함수
    void extractPointsAndFilterZeroValue(std::vector<cv::Point2f> cur_orb_point_2d_uv, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudTrack, std::vector<cv::Point3f> &cur_out_point3d, std::vector<uchar> &status);

    // 이미지와 탐지 시간을 사용하여 이미지를 표시하는 함수
    void image_show(std::vector<cv::DMatch> &matches, std::string& detectTime);
    
    // 이미지, 탐지 시간, 이전 이미지, 현재 이미지 및 키포인트 좌표를 사용하여 이미지를 표시하는 함수
    void image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img_, cv::Mat cur_img_, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_);
    
    // 매칭된 포인트와 이전 포인트를 사용하여 키포인트를 추출하는 함수
    void extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_points, std::vector<cv::Point3f> &cur_points, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d);
    
    // 소스 클라우드와 대상 클라우드를 사용하여 포인트 간의 변환 행렬을 계산하는 함수
    Eigen::Matrix4d p2p_calculateRandT(std::vector<cv::Point3f> &src_cloud, std::vector<cv::Point3f> &dst_cloud);
    
    // 변환 정보를 브로드캐스트하는 함수
    void tfBroadcast();
    
    // 키프레임을 가져오고 특징점을 저장하는 함수
    void getKeyframe(double time, std::vector<cv::Point3f> &featurMatchedpoints);
    
    // 루프 클로저 처리를 위한 스레드 함수
    void loopClosureThread();
    
    // 맵과 오도메트리를 처리하는 함수
    void mapOdomHandle();
    
    // 팩터 그래프 처리를 위한 스레드 함수
    void factorGraphThread();
    
    // ICP(Iterative Closest Point) 처리를 위한 스레드 함수
    void icpThread(loopClosureProcessor::Keyframe keyframeNew);
    
    // 포즈를 업데이트하는 함수
    void updatePoses();
    
    // 탐지 시간을 기반으로 TF2 정보를 파일에 작성하는 함수
    void writeTF2file(ros::Time &time);
    
    // 과거 데이터에서 하위 맵(submap)을 추출하는 함수
    void getSubmapOfhistory(pcl::PointCloud<PointType>::Ptr cloud_submap);
    
    // 현재 스캔을 지도에 매핑하는 함수
    void tranformCurrentScanToMap(pcl::PointCloud<PointType>::Ptr cloud_current_scan, size_t index, loopClosureProcessor::Keyframe keyframe_new);
    
    // 변환 행렬을 가져오는 함수
    Eigen::Matrix4d getTransformMatrix(size_t index);

    // 프로젝트 이름
    std::string PROJECT_NAME;
    
    // 클라우드 토픽 이름
    std::string CLOUD_TOPIC;
    
    // 이미지의 너비
    int IMAGE_WIDTH;
    
    // 이미지의 높이
    int IMAGE_HEIGHT;
    
    // 이미지 자르기 크기
    int IMAGE_CROP;
    
    // ORB(Oriented FAST and Rotated BRIEF) 특징 사용 여부
    int USE_ORB;
    
    // ORB 특징의 수
    int NUM_ORB_FEATURES;
    
    // 시간 스킵 간격
    double SKIP_TIME;
    
    // 병렬 스레드 수
    int NUM_THREADS;
    
    // 핸드헬드 모드 사용 여부
    bool HAND_HELD_FLAG;
    
    // TEASER (Robust Registration) 사용 여부
    bool USE_TEASER;
    
    // PnP-RANSAC (Perspective-n-Point with RANSAC) 사용 여부
    bool USE_PNPRANSAC;
    
    // ICP (Iterative Closest Point) 사용 여부
    bool USE_ICP;
    
    // 이미지 자르기 사용 여부
    bool USE_CROP;
    
    // 다운샘플링 사용 여부
    bool USE_DOWNSAMPLE;
    
    // 자르기 크기
    double CROP_SIZE;
    
    // 복셀 크기
    double VOXEL_SIZE;
    
    // ICP 적합성 점수
    double FITNESS_SCORE;
    
    // 키프레임 간 시간 간격
    double KEYFRAME_TIME_INTERVAL;
    
    // 키프레임 간 거리 간격
    double KEYFRAME_DISTANCE_INTERVAL;

    // 마스크 이미지
    cv::Mat MASK;
    
    // 현재 프레임에서 ORB 특징의 2D 좌표
    std::vector<cv::Point2f> cur_orb_point_2d_uv_;
    
    // 이전 프레임에서 ORB 특징의 2D 좌표
    std::vector<cv::Point2f> prev_orb_point_2d_uv_;
    
    // 포인트 클라우드 추적용 포인트 클라우드
    pcl::PointCloud<PointType>::Ptr cloudTrack_;
    
    // 현재 프레임의 포인트 클라우드
    pcl::PointCloud<PointType> cur_cloud_;
    
    // 이전 프레임의 포인트 클라우드
    pcl::PointCloud<PointType> prev_cloud_;
    
    // 지면 포인트 클라우드
    pcl::PointCloud<pcl::PointXYZ> ground_cloud_;
    
    // ORB 특징 추출 상태
    std::vector<uchar> status_;
    
    // 현재 프레임에서 추출된 3D 포인트
    std::vector<cv::Point3f> cur_out_point3d_;
    
    // 이전 프레임에서 추출된 3D 포인트
    std::vector<cv::Point3f> prev_out_point3d_;
    
    // 첫 번째 프레임의 시간
    double first_frame_time_;
    
    // 현재 프레임의 시간
    ros::Time time_stamp_;
    
    // 주파수
    int frequency_;
    
    // 키프레임 ID
    size_t keyframeId_;
    
    // 루프 클로저 처리기
    loopClosureHandler loopClosureProcessor_;
    
    // ICP 프로세스 스레드
    std::thread icpProcessThread_;
    
    // 루프 클로저 활성화 여부
    bool getLoop_;
    
    // 키프레임 위치 3D 포인트 클라우드
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D_;
    
    // 키프레임 위치 6D 포인트 클라우드
    pcl::PointCloud<PointXYZIRPYT>::Ptr cloudKeyPoses6D_;
    
    // 현재 키프레임
    loopClosureProcessor::Keyframe keyframe_;
    
    // 팩터 그래프 노드 큐
    std::deque<std::shared_ptr<loopClosureProcessor::FactorGraphNode>> factorGraphNode_;
    
    // 키프레임 큐
    std::deque<std::shared_ptr<loopClosureProcessor::Keyframe>> keyframesQueue_;
    
    // 지도 오도메트리 큐
    std::queue<nav_msgs::Odometry::ConstPtr> mapOdomQueue_;
    
    // 라이다 오도메트리 발행자
    ros::Publisher pubLaserOdometry_;
    
    // 매칭된 키포인트 이미지 발행자 (프런트 엔드)
    ros::Publisher matched_keypoints_img_pub_front_end;
    
    // 현재 스캔 포인트 클라우드
    pcl::PointCloud<PointType>::Ptr current_scan_;
    
    // 루프 클로저 스캔 포인트 클라우드
    pcl::PointCloud<PointType>::Ptr loop_scan_;
    
    // 강도 정보 스킵 여부
    bool skip_intensity_;
    
    // 특징 추출 시간
    double feature_extraction_time_;
    
    // 스캔 매칭 시간
    double scan_matching_time_;


    // ROS 노드 핸들
ros::NodeHandle nh_;

// 키포즈 발행자 (키프레임 위치)
ros::Publisher pubKeyPoses_;

// 루프 클로저 스캔 포인트 클라우드 발행자
ros::Publisher pubLoopScan_;

// 현재 스캔 포인트 클라우드 발행자
ros::Publisher pubCurrentScan_;

// 이전 강도 이미지
cv::Mat prev_img_;

// 현재 강도 이미지
cv::Mat cur_img_;

// 이전 프레임의 키포인트
std::vector<cv::KeyPoint> prev_keypoints_;

// 현재 프레임의 키포인트
std::vector<cv::KeyPoint> cur_keypoints_;

// 이전 프레임의 디스크립터
cv::Mat prev_descriptors_;

// 현재 프레임의 디스크립터
cv::Mat cur_descriptors_;

// PCL 시각화용 뷰어
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

// 로컬 스캔 간 스캔 트랜스폼 (R+T)
Eigen::Matrix4d T_s2s_;

// 로컬 스캔에서 맵으로의 전역 변환 행렬
Eigen::Matrix4d T_s2m_;

// 로컬 스캔에서 PGO 후의 전역 변환 행렬
Eigen::Matrix4d T_s2pgo_;

// 현재 위치
Eigen::Vector3d cur_position_;

// 이전 위치
Eigen::Vector3d prev_position_;

// 현재 회전
Eigen::Matrix3d cur_rotation_;

// 이전 회전
Eigen::Matrix3d prev_rotation_;

// 대응 관계 (키포인트 인덱스 쌍)
std::vector<std::pair<int, int>> corres_;

// gtsam 라이브러리를 사용한 그래프 최적화를 위한 요소
gtsam::NonlinearFactorGraph gtSAMgraph_;

// 최적화의 초기 추정치
gtsam::Values initialEstimate_;

// 최적화 결과 추정치
gtsam::Values optimizedEstimate_;

// ISAM2 최적화기
gtsam::ISAM2 *isam_;

// 현재 ISAM2 추정치
gtsam::Values isamCurrentEstimate_;

// Prior 요소의 노이즈 모델
gtsam::noiseModel::Diagonal::shared_ptr priorNoise_;

// 오도메트리 요소의 노이즈 모델
gtsam::noiseModel::Diagonal::shared_ptr odometryNoise_;

// 제약 조건 요소의 노이즈 모델
gtsam::noiseModel::Diagonal::shared_ptr constraintNoise_;

// 로버스트 노이즈 모델
gtsam::noiseModel::Base::shared_ptr robustNoiseModel_;

// 팩터 뮤텍스
std::mutex factorMutex;

// 그래프 최적화 이전 회전 (Quaternion)
Eigen::Quaterniond graph_prev_rotation_;

// 그래프 최적화 이전 위치
Eigen::Vector3d graph_prev_translation_;

// 루프 클로저 인덱스를 위한 KD 트리
int loop_index_kdtree_;

// 특징 추출 스킵 플래그
bool skip_flag_;

// PGO 수행 시간
double pgo_time_;

// 루프 클로저 수행 시간
double loop_time_;
};
}







