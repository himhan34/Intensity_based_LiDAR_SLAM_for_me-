// 중복 포함을 방지하기 위한 전처리기 지시문
#pragma once

// ROS 관련 헤더 파일 포함
#include <ros/ros.h>
#include <ros/package.h>

// ROS에서 이미지 메시지를 처리하기 위한 헤더 파일
#include <sensor_msgs/Image.h>

// DBoW3 라이브러리 포함 (데이터베이스 기반의 시각적 단어 및 이미지 매칭을 위한 라이브러리)
#include <DBoW3/DBoW3.h>

// OpenCV 관련 헤더 파일 포함
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 사용자 정의 헤더 파일 포함
#include "keyframe.h"            // 키 프레임 처리를 위한 클래스
#include "parameters.h"          // 파라미터 설정을 위한 클래스
#include "Scancontext.h"         // 스캔 컨텍스트 관련 클래스

// 주석 처리된 OpenCV 및 기타 라이브러리 포함
// #include <opencv2/features2d.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/eigen.hpp>
// #include <opencv/cv.h>
// #include "ouster_ros/ros.h"

// 루프 클로저 핸들러 클래스 정의
class loopClosureHandler
{
private:
    // 프라이빗 멤버 데이터

public:
    // 생성자
    loopClosureHandler(/* args */);
    // 소멸자
    ~loopClosureHandler();

    // 키 프레임 처리 함수
    void keyframeProcessor(loopClosureProcessor::Keyframe keyframe);
    // 키 프레임과 포인트 클라우드를 처리하는 함수
    void keyframeProcessor(loopClosureProcessor::Keyframe keyframe, pcl::PointCloud<PointType>::Ptr cloud_keyPose3D);
    // 루프 클로저 검출 함수
    int detectLoop(cv::Mat & image, cv::Mat& descriptors, int frame_index);
    // OpenCV 이미지를 ROS 이미지 메시지로 변환하는 함수
    sensor_msgs::ImagePtr cvMat2Image(std_msgs::Header header, cv::Mat & image);
    // 스캔 컨텍스트 매니저 객체
    SCManager scManager;

public:
    // DBoW3 데이터베이스 및 단어장
    DBoW3::Database db_;
    DBoW3::Vocabulary* voc_;
    // BoW 디스크립터 벡터
    std::vector<cv::Mat> bow_descriptors_;
    // 이미지 풀
    std::map<int, cv::Mat> image_pool_;
    // 키 프레임 풀
    std::map<size_t, loopClosureProcessor::Keyframe> keyframe_pool_;
    // 루프 인덱스
    int loop_index_;
    // KD-트리 포인터 및 뮤텍스
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyframeNearSearch_;
    std::mutex kdtreeKeyframeNearSearch_mutex_;
    // 이미지 마스크 (현재 주석 처리됨)
    // cv::Mat MASK;
};
