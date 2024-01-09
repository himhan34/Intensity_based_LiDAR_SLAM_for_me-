#pragma once
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>
#include "parameters.h"

namespace mapProcessor {

// 키프레임(Keyframe) 정보를 저장하는 구조체
struct Keyframe
{
    size_t keyframeId;                 // 키프레임의 고유 ID
    pcl::PointCloud<PointType> cloud_track_;   // 키프레임에 대한 포인트 클라우드 데이터 (트랙)
    pcl::PointCloud<pcl::PointXYZ> ground_cloud_; // 키프레임에 대한 지면 포인트 클라우드 데이터
    Eigen::Quaterniond q_map_cur_k;   // 지도 기준에서 현재 키프레임의 회전 정보 (쿼터니언)
    Eigen::Vector3d t_map_cur_k;      // 지도 기준에서 현재 키프레임의 위치 정보 (3차원 벡터)

    // 기본 생성자
    Keyframe() {}

    // 초기화 생성자
    Keyframe(
        size_t keyframeIdTmp,                     // 키프레임의 고유 ID
        pcl::PointCloud<PointType> cloud,         // 포인트 클라우드 데이터 (트랙)
        pcl::PointCloud<pcl::PointXYZ> cloud_ground, // 지면 포인트 클라우드 데이터
        Eigen::Quaterniond q_map_cur_k,           // 지도 기준에서 현재 키프레임의 회전 정보 (쿼터니언)
        Eigen::Vector3d t_map_cur_k) :            // 지도 기준에서 현재 키프레임의 위치 정보 (3차원 벡터)
        keyframeId{ keyframeIdTmp },
        cloud_track_{ cloud },
        ground_cloud_{ cloud_ground },
        q_map_cur_k_{ q_map_cur_k },
        t_map_cur_k_{ t_map_cur_k }
    {}
};
// namespace mapProcessor

struct SlideWindowKeyframe
{
    cv::Mat descriptors;                        // 두 키프레임 간 매치를 검출하기 위한 디스크립터
    cv::Mat image_intensity;                    // 시각화를 위한 이미지 강도 정보
    std::vector<cv::Point2f> orb_point_2d_uv;  // 시각화를 위한 ORB 키포인트 2D 좌표
    Eigen::Quaterniond q_map_cur_tk;            // 포인트 클라우드를 지도 프레임으로 변환하기 위한 회전 정보 (쿼터니언)
    Eigen::Vector3d t_map_cur_tk;               // 포인트 클라우드를 지도 프레임으로 변환하기 위한 위치 정보 (3차원 벡터)
    std::vector<cv::Point3f> cur_point3d_wrt_orb_features;  // ORB 키포인트에 대한 3D 포인트 클라우드 (Lidar 프레임 기준)

    // 기본 생성자
    SlideWindowKeyframe() {}

    // 초기화 생성자
    SlideWindowKeyframe(
        cv::Mat descriptorsTmp,                                   // 매치 검출을 위한 디스크립터
        cv::Mat image_intensityTmp,                               // 이미지 강도 정보
        std::vector<cv::Point2f> orb_point_2d_uvTmp,             // ORB 키포인트 2D 좌표
        Eigen::Quaterniond q_map_cur_tkTmp,                       // 포인트 클라우드 변환을 위한 회전 정보 (쿼터니언)
        Eigen::Vector3d t_map_cur_tkTmp,                          // 포인트 클라우드 변환을 위한 위치 정보 (3차원 벡터)
        std::vector<cv::Point3f> cur_point3d_wrt_orb_featuresTmp  // ORB 키포인트에 대한 3D 포인트 클라우드
    ) :
        descriptors{ descriptorsTmp },
        image_intensity{ image_intensityTmp },
        orb_point_2d_uv{ orb_point_2d_uvTmp },
        q_map_cur_tk{ q_map_cur_tkTmp },
        t_map_cur_tk{ t_map_cur_tkTmp },
        cur_point3d_wrt_orb_features{ cur_point3d_wrt_orb_featuresTmp }
    {}
};
}

namespace loopClosureProcessor {
    // 키프레임 데이터를 저장하는 구조체
    struct Keyframe {
        double time;                         // 시간 정보
        size_t keyframeId;                   // 키프레임의 고유 ID
        cv::Mat image;                       // 이미지 데이터
        std::vector<cv::KeyPoint> keypoints; // 키포인트 정보
        cv::Mat descriptors;                 // 키포인트 디스크립터
        std::vector<cv::Point3f> featureMatched3DPoints; // 매치된 3D 포인트 정보
        Eigen::Vector3d cur_position_;        // 현재 위치 (3D 벡터)
        Eigen::Vector3d prev_position_;       // 이전 위치 (3D 벡터)
        Eigen::Matrix3d cur_rotation_;        // 현재 회전 정보 (3x3 행렬)
        Eigen::Matrix3d prev_rotation_;       // 이전 회전 정보 (3x3 행렬)
        pcl::PointCloud<PointType> cloud_track_; // 포인트 클라우드 데이터

        // 기본 생성자
        Keyframe() {}

        // 초기화 생성자
        Keyframe(
            double timeTmp,                                // 시간 정보
            size_t keyframeIdTmp,                          // 고유 키프레임 ID
            cv::Mat imageTmp,                              // 이미지 데이터
            std::vector<cv::KeyPoint> keypointsTmp,        // 키포인트 정보
            cv::Mat descriptorsTmp,                        // 키포인트 디스크립터
            std::vector<cv::Point3f> featureMatched3DPointsTmp,  // 매치된 3D 포인트 정보
            Eigen::Vector3d cur_position_Tmp,              // 현재 위치 (3D 벡터)
            Eigen::Vector3d prev_position_Tmp,             // 이전 위치 (3D 벡터)
            Eigen::Matrix3d cur_rotation_Tmp,              // 현재 회전 정보 (3x3 행렬)
            Eigen::Matrix3d prev_rotation_Tmp,             // 이전 회전 정보 (3x3 행렬)
            pcl::PointCloud<PointType> cloud               // 포인트 클라우드 데이터
        ) :
            time{ timeTmp },
            keyframeId{ keyframeIdTmp },
            image{ imageTmp },
            keypoints{ keypointsTmp },
            descriptors{ descriptorsTmp },
            featureMatched3DPoints{ featureMatched3DPointsTmp },
            cur_position_{ cur_position_Tmp },
            prev_position_{ prev_position_Tmp },
            cur_rotation_{ cur_rotation_Tmp },
            prev_rotation_{ prev_rotation_Tmp },
            cloud_track_{ cloud }
        {}
    };

     struct FactorGraphNode {
            double time;                // 시간 정보
            size_t keyframeId;          // 키프레임의 고유 ID
            Eigen::Vector3d cur_position_;  // 현재 위치 (3D 벡터)
            Eigen::Vector3d prev_position_; // 이전 위치 (3D 벡터)
            Eigen::Matrix3d cur_rotation_;  // 현재 회전 정보 (3x3 행렬)
            Eigen::Matrix3d prev_rotation_; // 이전 회전 정보 (3x3 행렬)
    
            // 초기화 생성자
            FactorGraphNode(
                double timeTmp,                     // 시간 정보
                size_t keyframeIdTmp,               // 고유 키프레임 ID
                Eigen::Vector3d cur_position_Tmp,   // 현재 위치 (3D 벡터)
                Eigen::Vector3d prev_position_Tmp,  // 이전 위치 (3D 벡터)
                Eigen::Matrix3d cur_rotation_Tmp,   // 현재 회전 정보 (3x3 행렬)
                Eigen::Matrix3d prev_rotation_Tmp   // 이전 회전 정보 (3x3 행렬)
            ) :
                time{ timeTmp },
                keyframeId{ keyframeIdTmp },
                cur_position_{ cur_position_Tmp },
                prev_position_{ prev_position_Tmp },
                cur_rotation_{ cur_rotation_Tmp },
                prev_rotation_{ prev_rotation_Tmp }
            {}
    
            // 기본 생성자
            FactorGraphNode() {}
        };
    }
}







