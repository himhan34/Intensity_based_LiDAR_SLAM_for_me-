// 'nav_msgs/Odometry.h' 라이브러리를 포함합니다. 이 라이브러리는 오도메트리 데이터를 다룹니다.
#include <nav_msgs/Odometry.h>

// 'nav_msgs/Path.h' 라이브러리를 포함합니다. 이 라이브러리는 경로 데이터를 다룹니다.
#include <nav_msgs/Path.h> 

// 'message_filters/subscriber.h' 라이브러리를 포함합니다. 이 라이브러리는 메시지 필터링을 위한 구독자 기능을 제공합니다.
#include <message_filters/subscriber.h>

// 'message_filters/synchronizer.h' 라이브러리를 포함합니다. 이 라이브러리는 메시지 동기화를 위한 기능을 제공합니다.
#include <message_filters/synchronizer.h>

// 'message_filters/time_synchronizer.h' 라이브러리를 포함합니다. 이 라이브러리는 시간 기반 메시지 동기화 기능을 제공합니다.
#include <message_filters/time_synchronizer.h>

// 'message_filters/sync_policies/approximate_time.h' 라이브러리를 포함합니다. 대략적인 시간 동기화 정책을 제공합니다.
#include <message_filters/sync_policies/approximate_time.h>

// 'message_filters/sync_policies/exact_time.h' 라이브러리를 포함합니다. 정확한 시간 동기화 정책을 제공합니다.
#include <message_filters/sync_policies/exact_time.h>

// 'eigen3/Eigen/Dense' 라이브러리를 포함합니다. 이 라이브러리는 고밀도 행렬과 벡터 연산을 위한 기능을 제공합니다.
#include <eigen3/Eigen/Dense>

// ROS 퍼블리셔 객체 'pubMergedOdometry'를 선언합니다. 이 객체는 병합된 오도메트리 데이터를 발행합니다.
ros::Publisher pubMergedOdometry;

// Eigen 라이브러리를 사용한 쿼터니언 변수들을 선언합니다. 이들은 각각의 오도메트리 데이터에 사용됩니다.
Eigen::Quaterniond aloam_q_wodom_prev, intensity_q_wodom_prev, q_odom;

// Eigen 라이브러리를 사용한 3차원 벡터 변수들을 선언합니다. 이들은 위치 데이터에 사용됩니다.
Eigen::Vector3d aloam_t_wodom_prev, intensity_t_wodom_prev, t_odom;

// Eigen 라이브러리를 사용한 4x4 행렬 변수들을 선언합니다. 이들은 오도메트리 변환을 위해 사용됩니다.
Eigen::Matrix4d odom_cur, aloam_odom_cur, intensity_odom_cur, aloam_prev, intensity_prev;

// 'aloam_odom_init'과 'intensity_odom_init' 불리언 변수들을 선언하고, 초기값을 false로 설정합니다. 이 변수들은 오도메트리 초기화 상태를 나타냅니다.
bool aloam_odom_init = false, intensity_odom_init = false;

// 문자열 변수 'skip_flag'를 선언합니다. 이 변수는 특정 조건을 건너뛰기 위해 사용될 수 있습니다.
std::string skip_flag; 

// 두 개의 오도메트리 메시지를 입력으로 받는 콜백 함수 정의
void callback(const nav_msgs::Odometry::ConstPtr& aloam_odom, const nav_msgs::Odometry::ConstPtr& intensity_odom)
{
    // Eigen 라이브러리를 사용하여 쿼터니언과 벡터 변수 선언
    Eigen::Quaterniond aloam_q_wodom_curr, intensity_q_wodom_curr;
    Eigen::Vector3d aloam_t_wodom_curr, intensity_t_wodom_curr;

    // aloam 오도메트리 데이터에서 쿼터니언 값 추출
    aloam_q_wodom_curr.x() = aloam_odom->pose.pose.orientation.x;
    aloam_q_wodom_curr.y() = aloam_odom->pose.pose.orientation.y;
    aloam_q_wodom_curr.z() = aloam_odom->pose.pose.orientation.z;
    aloam_q_wodom_curr.w() = aloam_odom->pose.pose.orientation.w;

    // aloam 오도메트리 데이터에서 위치 값 추출
    aloam_t_wodom_curr.x() = aloam_odom->pose.pose.position.x;
    aloam_t_wodom_curr.y() = aloam_odom->pose.pose.position.y;
    aloam_t_wodom_curr.z() = aloam_odom->pose.pose.position.z;

    // aloam 오도메트리 변환 행렬 생성
    aloam_odom_cur.block<3,3>(0,0) = aloam_q_wodom_curr.toRotationMatrix();
    aloam_odom_cur.block<3,1>(0,3) = aloam_t_wodom_curr;
    aloam_odom_cur.block(3,0,1,4) << 0, 0, 0, 1;

    // intensity 오도메트리 데이터에서 쿼터니언 값 추출
    intensity_q_wodom_curr.x() = intensity_odom->pose.pose.orientation.x;
    intensity_q_wodom_curr.y() = intensity_odom->pose.pose.orientation.y;
    intensity_q_wodom_curr.z() = intensity_odom->pose.pose.orientation.z;
    intensity_q_wodom_curr.w() = intensity_odom->pose.pose.orientation.w;

    // intensity 오도메트리 데이터에서 위치 값 추출
    intensity_t_wodom_curr.x() = intensity_odom->pose.pose.position.x;
    intensity_t_wodom_curr.y() = intensity_odom->pose.pose.position.y;
    intensity_t_wodom_curr.z() = intensity_odom->pose.pose.position.z;

    // 특정 조건을 건너뛰기 위해 사용되는 플래그 설정
    skip_flag = intensity_odom->child_frame_id;

    // intensity 오도메트리 변환 행렬 생성
    intensity_odom_cur.block<3,3>(0,0) = intensity_q_wodom_curr.toRotationMatrix();
    intensity_odom_cur.block<3,1>(0,3) = intensity_t_wodom_curr;
    intensity_odom_cur.block(3,0,1,4) << 0, 0, 0, 1;

    // 오도메트리 초기화 확인
    if(!aloam_odom_init && !intensity_odom_init){
        // 처음 받은 오도메트리 데이터로 초기화
        aloam_prev = aloam_odom_cur;
        intensity_prev = intensity_odom_cur;
        odom_cur = intensity_odom_cur;
        aloam_odom_init = true;
        intensity_odom_init = true;
    }
    else{
        // 이전과 현재 오도메트리 데이터 비교를 통한 차이 계산
        Eigen::Matrix4d intensity_odom_diff = intensity_prev.inverse() * intensity_odom_cur;
        Eigen::Matrix4d aloam_odom_diff = aloam_prev.inverse() * aloam_odom_cur; 

        // skip_flag에 따른 조건부 처리
        if( skip_flag == "/odom_skip"){
            std::cout << "intensity_odom_diff: " << intensity_odom_diff << std::endl;
            odom_cur = odom_cur * aloam_odom_diff;
        }
        else{
            odom_cur = odom_cur * intensity_odom_diff;
        } 
        // 이전 오도메트리 데이터 업데이트
        aloam_prev = aloam_odom_cur;
        intensity_prev = intensity_odom_cur;
        
    }
    // 최종 오도메트리 데이터에서 회전 행렬 및 위치 추출
    Eigen::Matrix3d rot_cur = odom_cur.block(0,0,3,3);
    Eigen::Vector3d t_cur = odom_cur.block(0,3,3,1);
    Eigen::Quaterniond q_cur(rot_cur); 

    // 새로운 오도메트리 메시지 생성 및 값 설정
    nav_msgs::Odometry odom;
    odom.header.stamp = aloam_odom->header.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "laser_odom";
    odom.pose.pose.orientation.x = q_cur.x();
    odom.pose.pose.orientation.y = q_cur.y();
    odom.pose.pose.orientation.z = q_cur.z();
    odom.pose.pose.orientation.w = q_cur.w();
    odom.pose.pose.position.x = t_cur.x();
    odom.pose.pose.position.y = t_cur.y();
    odom.pose.pose.position.z = t_cur.z();

    // 병합된 오도메트리 데이터 발행
    pubMergedOdometry.publish(odom); 
}

// 메인 함수 정의
int main(int argc, char **argv)
{
    // ROS 시스템을 초기화하고 'map_optimization' 노드 이름으로 설정
    ros::init(argc, argv, "map_optimization");
    // ROS 노드 핸들 생성
    ros::NodeHandle nh;

    // '/laser_odom_to_init' 토픽에 대한 퍼블리셔 설정, 큐 사이즈는 100
    pubMergedOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

    // '/laser_odom_to_init_aloam' 토픽을 위한 구독자 설정, 큐 사이즈는 10
    message_filters::Subscriber<nav_msgs::Odometry> aloam_odom_sub(nh, "/laser_odom_to_init_aloam", 10);

    // '/laser_odom_to_init_intensity' 토픽을 위한 구독자 설정, 큐 사이즈는 10
    message_filters::Subscriber<nav_msgs::Odometry> intensity_odom_sub(nh, "/laser_odom_to_init_intensity", 10);

    // 동기화 정책을 위한 typedef 설정. ApproximateTime 정책 사용
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;

    // 두 오도메트리 구독자를 동기화하기 위한 Synchronizer 설정
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), aloam_odom_sub, intensity_odom_sub);

    // 동기화된 메시지에 대한 콜백 함수 등록
    sync.registerCallback(boost::bind(&callback, _1, _2)); 

    // ROS 이벤트 루프 시작
    ros::spin();
    return 0;  
}
