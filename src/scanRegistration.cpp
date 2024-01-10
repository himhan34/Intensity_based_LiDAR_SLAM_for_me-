// Merge intensitySLAM framework with ALOAM and use ALOAM's odometry when intensity features are not enough, usually 2 or 3 frames
// Modified by: Wenqiang Du         snowdwq@gmail.com


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

// 기본적인 수학 함수들을 위한 라이브러리
#include <cmath>
// 벡터 컨테이너 사용을 위한 라이브러리
#include <vector>
// 문자열 사용을 위한 라이브러리
#include <string>
// 시간 측정을 위한 사용자 정의 헤더 파일
#include "tic_toc.h"
// ROS 오도메트리 메시지 사용을 위한 라이브러리
#include <nav_msgs/Odometry.h>
// OpenCV 이미지 처리를 위한 라이브러리
#include <opencv2/imgproc.hpp>
// PCL(포인트 클라우드 라이브러리) 변환을 위한 라이브러리
#include <pcl_conversions/pcl_conversions.h>
// PCL 포인트 클라우드 사용을 위한 라이브러리
#include <pcl/point_cloud.h>
// PCL 포인트 타입 사용을 위한 라이브러리
#include <pcl/point_types.h>
// PCL 보셀 그리드 필터 사용을 위한 라이브러리
#include <pcl/filters/voxel_grid.h>
// PCL KD-트리 검색을 위한 라이브러리
#include <pcl/kdtree/kdtree_flann.h>
// ROS 기본 헤더 파일
#include <ros/ros.h>
// ROS IMU 센서 메시지 사용을 위한 라이브러리
#include <sensor_msgs/Imu.h>
// ROS 포인트 클라우드 메시지 사용을 위한 라이브러리
#include <sensor_msgs/PointCloud2.h>
// ROS 변환 데이터 타입 사용을 위한 라이브러리
#include <tf/transform_datatypes.h>
// ROS 변환 브로드캐스터 사용을 위한 라이브러리
#include <tf/transform_broadcaster.h>
// 사용자 정의 파라미터 헤더 파일
#include "parameters.h"
// 강도 기반 특징 추적을 위한 사용자 정의 헤더 파일
#include "intensity_feature_tracker.h"
// 이미지 처리를 위한 사용자 정의 헤더 파일
#include "image_handler.h"

// 표준 수학 함수 사용을 위한 이름공간 선언
using std::atan2;
using std::cos;
using std::sin;

// 건너뛸 프레임 수를 저장하는 변수 선언
uint32_t skipFrameNum = 0;

// 스캔 주기를 저장하는 상수 변수 선언
const double scanPeriod = 0.1;

// 시스템 지연을 나타내는 상수 변수 선언
const int systemDelay = 0; 

// 시스템 초기화 카운트를 저장하는 변수 선언
int systemInitCount = 0;

// 시스템 초기화 여부를 나타내는 불린 변수 선언
bool systemInited = false;

// 스캔 라인 수를 저장하는 변수 선언
int N_SCANS = 0;

// 클라우드 곡률 배열 선언
float cloudCurvature[400000];

// 클라우드 정렬 인덱스 배열 선언
int cloudSortInd[400000];

// 클라우드 이웃 선택 배열 선언
int cloudNeighborPicked[400000];

// 클라우드 라벨 배열 선언
int cloudLabel[400000];

// 이미지 핸들러 객체 포인터 선언
intensity_slam::ImageHandler *image_handler;

// 특징 추적 객체 포인터 선언
intensity_slam::feature_tracker *feature_tracker;

// 클라우드 곡률 값을 비교하는 함수 정의. 두 인덱스의 곡률 값을 비교하여 작은 것을 반환.
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

// 레이저 포인트 클라우드 데이터를 발행하기 위한 퍼블리셔
ros::Publisher pubLaserCloud;

// 예리한 모서리 포인트들을 발행하기 위한 퍼블리셔
ros::Publisher pubCornerPointsSharp;

// 덜 예리한 모서리 포인트들을 발행하기 위한 퍼블리셔
ros::Publisher pubCornerPointsLessSharp;

// 평평한 표면 포인트들을 발행하기 위한 퍼블리셔
ros::Publisher pubSurfPointsFlat;

// 덜 평평한 표면 포인트들을 발행하기 위한 퍼블리셔
ros::Publisher pubSurfPointsLessFlat;

// 제거할 포인트들을 발행하기 위한 퍼블리셔
ros::Publisher pubRemovePoints;

// 각 스캔별로 데이터를 발행하기 위한 퍼블리셔 벡터
std::vector<ros::Publisher> pubEachScan;

// 각 라인별로 데이터를 발행할지 여부를 나타내는 불린 변수 선언
bool PUB_EACH_LINE = false;

// 포인트 클라우드에서 고려할 최소 거리 값
double MINIMUM_RANGE = 0.1; 

// PointT 타입의 포인트 클라우드에서 특정 거리보다 가까운 포인트를 제거하는 템플릿 함수
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    // 입력 클라우드와 출력 클라우드가 다르면, 출력 클라우드의 헤더와 포인트 크기를 설정
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    // 출력 클라우드에 저장될 포인트의 인덱스
    size_t j = 0;

    // 입력 클라우드의 모든 포인트를 순회
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        // 포인트가 설정한 거리보다 가깝다면 건너뛰기
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        // 가까운 포인트가 아니라면 출력 클라우드에 추가
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    // 모든 포인트를 처리한 후, 출력 클라우드의 실제 크기 조정
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    // 출력 클라우드의 높이, 너비 및 밀도 설정
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

// 레이저 포인트 클라우드 메시지를 처리하는 함수
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    // 전체 처리 시간을 측정하기 위한 TicToc 객체 생성
    TicToc t_whole;

    // 이미지 핸들러를 사용하여 클라우드 데이터 처리
    image_handler->cloud_handler(laserCloudMsg);

    // 클라우드 메시지의 타임스탬프 저장
    ros::Time cloud_time = laserCloudMsg->header.stamp;

    // 특징 검출에 걸리는 시간을 측정하기 위한 TicToc 객체 생성
    TicToc detectFeatures_time;

    // 클라우드 핸들링에 걸린 시간 계산
    double cloudHandler_time = t_whole.toc();

    // 특징 추적기를 사용하여 특징 검출
    feature_tracker->detectfeatures(cloud_time, image_handler->image_intensity, image_handler->cloud_track, image_handler->GroundPointOut);

    // 시스템이 초기화되지 않았다면
    if (!systemInited)
    {
        // 초기화 카운트 증가
        systemInitCount++;

        // 초기화 카운트가 지연 시간보다 크거나 같으면 시스템 초기화
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return; // 아니면 함수 반환
    }


        
    // 데이터 준비를 위한 시간 측정 객체 생성
    TicToc t_prepare;
    
    // 각 스캔의 시작과 끝 인덱스를 저장할 벡터 초기화
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    
    // ROS 메시지를 PCL 포인트 클라우드로 변환
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    
    // 가까운 포인트를 제거하기 위한 인덱스 벡터
    std::vector<int> indices;
    
    // 가까운 포인트 제거 함수 호출
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);
    
    // 처리된 포인트 클라우드의 크기 계산
    int cloudSize = laserCloudIn.points.size();
    
    // 포인트 클라우드의 시작 방향 계산
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    
    // 포인트 클라우드의 끝 방향 계산
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;
    
    // 방향 범위 조정
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    
    // 반을 지났는지 나타내는 불린 변수
    bool halfPassed = false;
    
    // 포인트 클라우드의 포인트 수
    int count = cloudSize;
    
    // 포인트 타입 변수 선언
    PointType point;
    
    // 각 스캔별 포인트 클라우드를 저장할 벡터
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
  
  // 포인트 클라우드의 모든 포인트에 대해 반복
  for (int i = 0; i < cloudSize; i++)
  {
      // 현재 포인트의 좌표 설정
      point.x = laserCloudIn.points[i].x;
      point.y = laserCloudIn.points[i].y;
      point.z = laserCloudIn.points[i].z;
  
      // 포인트의 수직 각도 계산
      float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
      // 스캔 ID 초기화
      int scanID = 0;
  
      // N_SCANS의 값에 따라 스캔 ID 계산
      if (N_SCANS == 16)
      {
          scanID = int((angle + 15) / 2 + 0.5);
          if (scanID > (N_SCANS - 1) || scanID < 0)
          {
              count--;
              continue;
          }
      }
      else if (N_SCANS == 32)
      {
          scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
          if (scanID > (N_SCANS - 1) || scanID < 0)
          {
              count--;
              continue;
          }
      }
      else if (N_SCANS == 64)
      {
          scanID = int((angle + 22.5) * 1.41 + 0.5)-1;
          if (scanID > (N_SCANS - 1) || scanID < 0)
          {
              count--;
              continue;
          }
      }
      else if (N_SCANS == 128)
      {
          scanID = int((angle + 22.5) * 2.83 + 0.5)-1;
          if (scanID > (N_SCANS - 1) || scanID < 0)
          {
              count--;
              continue;
          }
      }
      else
      {
          // 스캔 수가 잘못되었을 경우 오류 메시지 출력 및 ROS 중단
          printf("wrong scan number\n");
          ROS_BREAK();
      }
  
      // 포인트의 방향 계산
      float ori = -atan2(point.y, point.x);
      // 반을 지났는지 확인
      if (!halfPassed)
      { 
          // 방향 조정
          if (ori < startOri - M_PI / 2)
          {
              ori += 2 * M_PI;
          }
          else if (ori > startOri + M_PI * 3 / 2)
          {
              ori -= 2 * M_PI;
          }
  
          // 절반을 지났는지 확인
          if (ori - startOri > M_PI)
          {
              halfPassed = true;
          }
      }
      else
      {
          // 방향 조정
          ori += 2 * M_PI;
          if (ori < endOri - M_PI * 3 / 2)
          {
              ori += 2 * M_PI;
          }
          else if (ori > endOri + M_PI / 2)
          {
              ori -= 2 * M_PI;
          }
      }
  
      // 상대적 시간 계산
      float relTime = (ori - startOri) / (endOri - startOri);
      // 강도에 스캔 ID와 상대적 시간을 저장
      point.intensity = scanID + scanPeriod * relTime;
      // 해당 스캔 ID에 포인트 추가
      laserCloudScans[scanID].push_back(point); 
  }

    
  // 처리된 포인트 클라우드의 크기를 업데이트
  cloudSize = count;
  
  // 새로운 포인트 클라우드 객체 생성
  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  
  // 모든 스캔에 대해 반복
  for (int i = 0; i < N_SCANS; i++)
  { 
      // 각 스캔의 시작 인덱스 설정
      scanStartInd[i] = laserCloud->size() + 5;
  
      // laserCloud에 각 스캔의 포인트 추가
      *laserCloud += laserCloudScans[i];
  
      // 각 스캔의 끝 인덱스 설정
      scanEndInd[i] = laserCloud->size() - 6;
  }
  
  // 포인트 클라우드의 모든 포인트에 대해 반복
  for (int i = 5; i < cloudSize - 5; i++)
  { 
      // 인접한 포인트들과의 차이를 계산하여 곡률 계산
      float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
      float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
      float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
  
      // 곡률 값을 cloudCurvature 배열에 저장
      cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
      // 정렬을 위한 인덱스 초기화
      cloudSortInd[i] = i;
      // 이웃 포인트 선택 플래그 초기화
      cloudNeighborPicked[i] = 0;
      // 라벨 초기화
      cloudLabel[i] = 0;
  }
  
  // 포인트 처리 시간 측정을 위한 TicToc 객체 생성
  TicToc t_pts;
  
  // 각 카테고리별 포인트 클라우드 객체 생성
  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;
  
  // 정렬 시간 측정을 위한 변수 초기화
  float t_q_sort = 0;

 // 모든 스캔에 대해 반복
for (int i = 0; i < N_SCANS; i++)
{
    // 스캔의 포인트 수가 최소 기준 미만이면 건너뛰기
    if( scanEndInd[i] - scanStartInd[i] < 6)
        continue;

    // 덜 평평한 표면 포인트 클라우드 초기화
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);

    // 스캔을 6개의 세그먼트로 나누어 처리
    for (int j = 0; j < 6; j++)
    {
        // 세그먼트의 시작 및 끝 인덱스 계산
        int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
        int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

        // 정렬 시간 측정
        TicToc t_tmp;
        std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
        t_q_sort += t_tmp.toc();

        // 가장 큰 곡률을 가진 포인트 선택
        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--)
        {
            int ind = cloudSortInd[k];

            // 선택되지 않았고, 곡률이 임계값 이상인 경우
            if (cloudNeighborPicked[ind] == 0 &&
                cloudCurvature[ind] > 0.1)
            {
                largestPickedNum++;
                if (largestPickedNum <= 2)
                {                        
                    // 가장 큰 곡률을 가진 포인트에 라벨 설정 및 추가
                    cloudLabel[ind] = 2;
                    cornerPointsSharp.push_back(laserCloud->points[ind]);
                    cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                }
                else if (largestPickedNum <= 20)
                {                        
                    // 다음으로 큰 곡률을 가진 포인트에 라벨 설정 및 추가
                    cloudLabel[ind] = 1;
                    cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                }
                else
                {
                    break;
                }

                // 선택된 포인트의 이웃 포인트를 선택됨으로 표시
                cloudNeighborPicked[ind] = 1;

                // 선택된 포인트 주변의 이웃 포인트 처리
                for (int l = 1; l <= 5; l++)
                {
                    float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                    float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                    float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind + l] = 1;
                }
                for (int l = -1; l >= -5; l--)
                {
                    float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                    float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                    float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind + l] = 1;
                }
            }
        }
    


      // 가장 작은 곡률을 가진 포인트 선택을 위한 카운터
      int smallestPickedNum = 0;
      
      // 설정된 세그먼트 내의 모든 포인트에 대해 반복
      for (int k = sp; k <= ep; k++)
      {
          // 현재 포인트의 인덱스
          int ind = cloudSortInd[k];
      
          // 이웃 포인트가 선택되지 않았고, 곡률이 임계값보다 작은 경우
          if (cloudNeighborPicked[ind] == 0 &&
              cloudCurvature[ind] < 0.1)
          {
              // 포인트에 라벨 설정 및 평평한 표면 포인트 클라우드에 추가
              cloudLabel[ind] = -1;
              surfPointsFlat.push_back(laserCloud->points[ind]);
      
              // 선택된 포인트 수 증가
              smallestPickedNum++;
              // 최소 선택된 포인트 수에 도달하면 중단
              if (smallestPickedNum >= 4)
              { 
                  break;
              }
      
              // 선택된 포인트의 이웃을 선택됨으로 표시
              cloudNeighborPicked[ind] = 1;
      
              // 선택된 포인트 주변의 이웃 포인트 처리
              for (int l = 1; l <= 5; l++)
              { 
                  float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                  float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                  float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                  // 변화량이 임계값 이상이면 중단
                  if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                  {
                      break;
                  }
      
                  // 이웃 포인트를 선택됨으로 표시
                  cloudNeighborPicked[ind + l] = 1;
              }
              for (int l = -1; l >= -5; l--)
              {
                  float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                  float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                  float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                  // 변화량이 임계값 이상이면 중단
                  if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                  {
                      break;
                  }
      
                  // 이웃 포인트를 선택됨으로 표시
                  cloudNeighborPicked[ind + l] = 1;
              }
          }
      }
      // 설정된 세그먼트 내의 모든 포인트에 대해 반복
      for (int k = sp; k <= ep; k++)
      {
          // 라벨이 0 이하인 포인트를 surfPointsLessFlatScan에 추가
          if (cloudLabel[k] <= 0)
          {
              surfPointsLessFlatScan->push_back(laserCloud->points[k]);
          }
      }
      
      // surfPointsLessFlatScan의 다운샘플링을 위한 pcl::PointCloud 객체 생성
      pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
      
      // 다운샘플링 필터 설정
      pcl::VoxelGrid<PointType> downSizeFilter;
      downSizeFilter.setInputCloud(surfPointsLessFlatScan);
      downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
      downSizeFilter.filter(surfPointsLessFlatScanDS);
      
      // 다운샘플링된 포인트를 surfPointsLessFlat에 추가
      surfPointsLessFlat += surfPointsLessFlatScanDS;
      
      // 각 포인트 클라우드에 대해 ROS 메시지로 변환 및 발행
      sensor_msgs::PointCloud2 laserCloudOutMsg;
      pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
      laserCloudOutMsg.header.frame_id = "os_sensor";
      pubLaserCloud.publish(laserCloudOutMsg);
      
      sensor_msgs::PointCloud2 cornerPointsSharpMsg;
      pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
      cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
      
      // 강도 이미지가 매칭하기 어려울 경우에 대한 처리
      if(feature_tracker->skip_intensity_){
          cornerPointsSharpMsg.header.frame_id = "skip_intensity";
          skipFrameNum ++; 
          std::cout << "skipFrameNum: " << skipFrameNum << std::endl;
          ROS_ERROR("intensity image is hard to match, skipped!")
          feature_tracker->skip_intensity_ = false; 
      }

    // skip_intensity_ 플래그가 false일 경우
    else{
        // cornerPointsSharp 메시지의 헤더 프레임 ID 설정
        cornerPointsSharpMsg.header.frame_id = "os_sensor";
    }
    
    // 처리된 sharp 코너 포인트 클라우드를 ROS 메시지로 발행
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);
    
    // 처리된 less sharp 코너 포인트 클라우드를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "os_sensor";
    // less sharp 코너 포인트 클라우드 메시지 발행
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);
    
    // 처리된 평평한 표면 포인트 클라우드를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "os_sensor";
    // 평평한 표면 포인트 클라우드 메시지 발행
    pubSurfPointsFlat.publish(surfPointsFlat2);
    
    // 처리된 덜 평평한 표면 포인트 클라우드를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "os_sensor";
    // 덜 평평한 표면 포인트 클라우드 메시지 발행
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
    
    // 각 라인별로 데이터를 발행하는 경우
    if(PUB_EACH_LINE)
    {
        // 모든 스캔에 대해 반복
        for(int i = 0; i< N_SCANS; i++)
        {
            // 각 스캔을 ROS 메시지로 변환
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "os_sensor";
            // 각 스캔 메시지 발행
            pubEachScan[i].publish(scanMsg);
        }
    }
    
    // map_odom 콜백에 대한 뮤텍스 선언
    std::mutex mapOdomMutex;
    
    // 맵 오도메트리 데이터 콜백 함수
    void map_odom_callback(const nav_msgs::Odometry::ConstPtr &mapOdometry){
        mapOdomMutex.lock();
        // 맵 오도메트리 큐에 데이터 추가
        feature_tracker->mapOdomQueue_.push(mapOdometry); 
        mapOdomMutex.unlock(); 
    }

// 메인 함수 정의
int main(int argc, char **argv)
{
    // ROS 초기화 및 노드 이름 설정
    ros::init(argc, argv, "scanRegistration");
    // ROS 노드 핸들 생성
    ros::NodeHandle nh;
    // 로그 레벨 설정
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    // 특징 추적기 객체 생성
    feature_tracker = new intensity_slam::feature_tracker(nh);
    // 파라미터 읽기
    std::cout << "read parameters" << std::endl;
    feature_tracker->readParameters();

    // 이미지 핸들러 객체 생성
    image_handler = new intensity_slam::ImageHandler(feature_tracker->IMAGE_HEIGHT, 
                                                     feature_tracker->IMAGE_WIDTH, 
                                                     feature_tracker->NUM_THREADS);

    // ROS 파라미터 서버에서 스캔 라인 수 설정
    nh.param<int>("/intensity_feature_tracker/image_height", N_SCANS, 64);

    // ROS 파라미터 서버에서 최소 범위 설정
    nh.param<double>("/map_optimization_parameters/remove_radius", MINIMUM_RANGE, 0.3);

    // 스캔 라인 수 출력
    printf("scan line number %d \n", N_SCANS);

    // 지원되는 스캔 라인 수 확인
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64 && N_SCANS != 128)
    {
        printf("only support velodyne with 16, 32, 64, 128 scan line!");
        return 0;
    }

    // 클라우드 토픽 이름 설정
    std::string CLOUD_TOPIC;
    nh.getParam("/intensity_feature_tracker/cloud_topic", CLOUD_TOPIC);

    // 클라우드 데이터와 맵 오도메트리 데이터에 대한 구독자 설정
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(CLOUD_TOPIC, 100, laserCloudHandler);
    ros::Subscriber sub_map_path = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100, map_odom_callback);

    // 다양한 종류의 포인트 클라우드에 대한 퍼블리셔 설정
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);
    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    // 각 라인별로 데이터를 발행하는 경우
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }

    // 루프 클로저와 팩터 그래프 처리를 위한 쓰레드 생성 및 실행
    std::thread loopClosureThread_(&intensity_slam::feature_tracker::loopClosureThread, feature_tracker);
    std::thread factorGraphThread_(&intensity_slam::feature_tracker::factorGraphThread, feature_tracker);

    // 멀티스레드 스피너 생성 및 실행
    ros::MultiThreadedSpinner spinner(8);
    spinner.spin();

    // 쓰레드 조인
    factorGraphThread_.join();
    loopClosureThread_.join();

    return 0;
}




