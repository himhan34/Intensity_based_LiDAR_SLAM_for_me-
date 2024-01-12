// 주석: https://github.com/TixiaoShan/imaging_lidar_place_recognition에서 수정된 코드입니다.
#pragma once // 주석: 헤더 파일이 중복으로 포함되는 것을 방지하기 위한 전처리 지시문

#include "parameters.h" // 주석: 사용자 정의 매개변수 및 설정이 포함된 헤더 파일
#include <omp.h> // 주석: OpenMP 라이브러리를 사용하여 병렬 처리를 지원하는 헤더 파일
#include <math.h> // 주석: 수학 함수 및 상수를 사용하는 헤더 파일

#include <pcl/segmentation/sac_segmentation.h> // 주석: PCL(Point Cloud Library) 라이브러리의 점군 세그먼테이션 관련 헤더 파일
#include <pcl/sample_consensus/method_types.h> // 주석: PCL 샘플 콘센서스 메소드 관련 헤더 파일
#include <pcl/sample_consensus/model_types.h> // 주석: PCL 샘플 콘센서스 모델 관련 헤더 파일
#include <pcl/ModelCoefficients.h> // 주석: PCL 모델 계수 관련 헤더 파일

namespace intensity_slam {
class ImageHandler
{
public:

    ros::NodeHandle nh; // 주석: ROS(Node.js 기반 로봇 운영체제) 노드 핸들을 생성합니다.

    ros::Publisher pub_image; // 주석: ROS 이미지 퍼블리셔를 생성합니다.
    ros::Publisher groundPoint_pub; // 주석: ROS 포인트 클라우드 퍼블리셔를 생성합니다.

    cv::Mat image_range; // 주석: OpenCV(Mat 클래스를 사용한) 이미지 객체(image_range)를 생성합니다.
    cv::Mat image_ambient; // 주석: OpenCV(Mat 클래스를 사용한) 이미지 객체(image_ambient)를 생성합니다.
    cv::Mat image_intensity; // 주석: OpenCV(Mat 클래스를 사용한) 이미지 객체(image_intensity)를 생성합니다.
    int IMAGE_HEIGHT, IMAGE_WIDTH, NUM_THREADS; // 주석: 이미지 높이, 너비 및 스레드 수를 정의합니다.

    pcl::PointCloud<PointType>::Ptr cloud_track; // 주석: PCL(Point Cloud Library) 점군(cloud_track)을 나타내는 포인터를 생성합니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPointOut; // 주석: PCL의 포인트 클라우드(pcl::PointXYZ)를 나타내는 포인터를 생성합니다.
    ImageHandler(int height = 128, int width = 1024, int threadNum = 6) // 주석: 이미지 핸들러(ImageHandler) 클래스의 생성자를 정의합니다.
    {
        IMAGE_HEIGHT = height; // 주석: 이미지 높이 초기화
        IMAGE_WIDTH = width; // 주석: 이미지 너비 초기화
        NUM_THREADS = threadNum; // 주석: 스레드 수 초기화
        cloud_track.reset(new pcl::PointCloud<PointType>()); // 주석: PCL 포인트 클라우드 객체를 초기화합니다.
        cloud_track->resize(IMAGE_HEIGHT * IMAGE_WIDTH); // 주석: PCL 포인트 클라우드의 크기를 설정합니다.
        GroundPointOut.reset(new pcl::PointCloud<pcl::PointXYZ>); // 주석: PCL 포인트 클라우드 객체를 초기화합니다.
        groundPoint_pub = nh.advertise<sensor_msgs::PointCloud2>("/ransac_groundPoint", 1000); // 주석: 포인트 클라우드 퍼블리셔를 초기화합니다.
    }

void groundPlaneExtraction(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    GroundPointOut->points.clear(); // 주석: 추출된 지면 포인트 클라우드를 초기화합니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_original_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *point_cloud_original_ptr); // 주석: ROS 메시지로부터 포인트 클라우드로 변환합니다.

    assert((int)point_cloud_original_ptr->size() % IMAGE_HEIGHT * IMAGE_WIDTH == 0); // 주석: 포인트 클라우드 크기가 이미지 크기의 배수임을 확인합니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_screening_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    #pragma omp parallel for num_threads(NUM_THREADS)
    for (long unsigned int i = 0; i < point_cloud_original_ptr->points.size(); i++) {
        if (point_cloud_original_ptr->points[i].z >= -2.0 && point_cloud_original_ptr->points[i].z <= -0.45) {
            point_cloud_screening_ptr->points.push_back(point_cloud_original_ptr->points[i]);
        }
    }
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(15 * M_PI / 180);
    seg.setNumberOfThreads(2 * NUM_THREADS);
    seg.setInputCloud(point_cloud_screening_ptr);
    seg.segment(*inliers, *coefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPoint_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];

    Eigen::Vector3f plane_normal(A, B, C);
    Eigen::Vector3f z_normal(0, 0, 1);
    
    if (plane_normal.dot(z_normal) > cos(15 * M_PI / 180)) {
        #pragma omp parallel for num_threads(NUM_THREADS)
        for (long unsigned int i = 0; i < point_cloud_original_ptr->points.size(); i++) {
            double X = point_cloud_original_ptr->points[i].x;
            double Y = point_cloud_original_ptr->points[i].y;
            double Z = point_cloud_original_ptr->points[i].z;
            double height = abs(A * X + B * Y + C * Z + D) / sqrt(A * A + B * B + C * C); // 주석: 점에서 평면까지의 높이를 계산합니다.

            if ((height <= 0.03) && (point_cloud_original_ptr->points[i].z < -0.0)) {
                GroundPoint_ptr->points.push_back(point_cloud_original_ptr->points[i]);
            }
        }

        GroundPointOut = GroundPoint_ptr; // 주석: 추출된 지면 포인트 클라우드를 저장합니다.
        
        if (groundPoint_pub.getNumSubscribers() != 0) {
            sensor_msgs::PointCloud2 groundPoint_cloud_msg;
            pcl::toROSMsg(*GroundPoint_ptr, groundPoint_cloud_msg); // 주석: PCL 포인트 클라우드를 ROS 메시지로 변환합니다.
            groundPoint_cloud_msg.header = cloud_msg->header;
            groundPoint_pub.publish(groundPoint_cloud_msg); // 주석: 지면 포인트 클라우드를 퍼블리시합니다.
        }
    }
}


void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{       
    pcl::PointCloud<PointOuster>::Ptr laser_cloud(new pcl::PointCloud<PointOuster>());
    pcl::fromROSMsg(*cloud_msg, *laser_cloud); // ROS 메시지에서 포인트 클라우드로 변환합니다.
    assert((int)laser_cloud->size() % IMAGE_HEIGHT * IMAGE_WIDTH == 0); // 포인트 클라우드 크기가 이미지 크기의 배수임을 확인합니다.
    image_range = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0)); // 이미지를 초기화합니다.
    image_ambient = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0)); // 이미지를 초기화합니다.
    image_intensity = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0)); // 이미지를 초기화합니다.

    #pragma omp parallel for num_threads(NUM_THREADS)
    for (int u = 0; u < IMAGE_HEIGHT; u++) 
    {
        for (int v = 0; v < IMAGE_WIDTH; v++) 
        {
            const auto& pt = laser_cloud->points[u * IMAGE_WIDTH + v]; // 현재 이미지 픽셀에 해당하는 포인트를 가져옵니다.
            float range = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z); // 주석: 포인트의 거리를 계산합니다.
            float intensity = pt.intensity; // 포인트의 강도 값을 가져옵니다.

            intensity = std::min(intensity, 255.0f); // 강도 값을 최대 255로 제한합니다.
            image_range.at<uint8_t>(u, v) = std::min(range * 20, 255.0f); // 이미지의 거리 정보를 설정합니다.
            image_intensity.at<uint8_t>(u, v) = intensity; // 이미지의 강도 정보를 설정합니다.
            PointType* p = &cloud_track->points[u * IMAGE_WIDTH + v]; // 주석: 추적용 포인트 클라우드의 현재 위치를 가져옵니다.

            if (range >= 0.1)
            {
                p->x = pt.x;
                p->y = pt.y;
                p->z = pt.z;
                p->intensity = intensity; // 주석: 추적용 포인트 클라우드에 현재 포인트 정보를 설정합니다.
            }
            else
            {
                p->x = p->y = p->z =  0;
                p->intensity = 0; // 주석: 거리가 너무 가까운 포인트는 0으로 설정하여 필터링합니다.
            }
        }
    }
}


