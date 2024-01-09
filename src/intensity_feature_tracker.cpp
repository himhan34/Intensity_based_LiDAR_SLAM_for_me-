#include "intensity_feature_tracker.h" // Intensity Feature Tracker 클래스 헤더 파일을 포함합니다.
#include "tic_toc.h" // Tic-Toc 라이브러리 헤더 파일을 포함합니다.
#include <tf2_ros/transform_broadcaster.h> // TF2 ROS의 Transform Broadcaster 헤더 파일을 포함합니다.
#include <ceres/ceres.h> // Ceres 라이브러리 헤더 파일을 포함합니다.
#include "lidarFeaturePointsFunction.hpp" // Lidar 특징 포인트 함수 헤더 파일을 포함합니다.

using namespace intensity_slam; // intensity_slam 네임스페이스를 사용합니다.


template <typename Derived>
static void reduceVector(std::vector<Derived> &v, std::vector<uchar> status)
{
    int j = 0; // 새로운 벡터의 인덱스를 추적하기 위한 변수 j를 초기화합니다.
    for (int i = 0; i < int(v.size()); i++) // 입력 벡터 v의 모든 요소를 반복하며 확인합니다.
    {
        if (status[i]) // status 벡터의 i번째 요소가 참(1)인 경우에만 실행합니다.
        {
            v[j++] = v[i]; // 입력 벡터 v에서 i번째 요소를 새로운 벡터에 복사하고 j를 증가시킵니다.
        }
    }
    v.resize(j); // 벡터 v의 크기를 j로 조정하여 불필요한 요소를 제거합니다.
}

feature_tracker::feature_tracker(ros::NodeHandle &n)
                    :nh_(n)                
{   
    pubKeyPoses_ = nh_.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2); // "key_pose_origin" 토픽을 퍼블리시하는 ROS 노드 핸들을 초기화합니다.
    pubLoopScan_ = nh_.advertise<sensor_msgs::PointCloud2>("/loop_scan", 2); // "loop_scan" 토픽을 퍼블리시하는 ROS 노드 핸들을 초기화합니다.
    pubCurrentScan_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_scan", 2); // "current_scan" 토픽을 퍼블리시하는 ROS 노드 핸들을 초기화합니다.
    T_s2s_ = Eigen::Matrix4d::Identity(); // 동일한 좌표계 사이의 변환 행렬을 단위 행렬로 초기화합니다.
    T_s2m_ = Eigen::Matrix4d::Identity(); // 스캔에서 지도로의 변환 행렬을 단위 행렬로 초기화합니다.
    T_s2pgo_ = Eigen::Matrix4d::Identity(); // 스캔에서 PGO(그래프 최적화)로의 변환 행렬을 단위 행렬로 초기화합니다.
    prev_position_ << 0.0, 0.0, 0.0; // 이전 위치를 0으로 초기화합니다.
    prev_rotation_ <<   0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0; // 이전 회전을 0으로 초기화합니다.
    keyframeId_ = 0; // 키프레임 ID를 0으로 초기화합니다.
    cloudKeyPoses3D_.reset(new pcl::PointCloud<PointType>()); // 3D 포인트 클라우드를 초기화합니다.
    cloudKeyPoses6D_.reset(new pcl::PointCloud<PointTypePose>()); // 6D 포인트 클라우드를 초기화합니다.

    gtsam::Vector Vector6(6); // 크기가 6인 벡터를 생성합니다.
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6; // 벡터의 값을 설정합니다.
    priorNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6); // 사전 노이즈 모델을 초기화합니다.
    odometryNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6); // 오도메트리 노이즈 모델을 초기화합니다.
    gtsam::Vector Vector6_loop(6); // 크기가 6인 벡터를 생성합니다.
    Vector6_loop << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4; // 벡터의 값을 설정합니다.
    constraintNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6_loop); // 제약 노이즈 모델을 초기화합니다.

    gtsam::ISAM2Params parameters; // ISAM2 파라미터를 생성합니다.
    parameters.relinearizeThreshold = 0.01; // 재선형화 임계값을 설정합니다.
    parameters.relinearizeSkip = 1; // 재선형화 스킵 수를 설정합니다.
    isam_ = new gtsam::ISAM2(parameters); // ISAM2 객체를 초기화합니다.
    getLoop_ = false; // 루프 감지 상태를 초기화합니다.
    pubLaserOdometry_ = nh_.advertise<nav_msgs::Odometry>("/laser_odom_to_init_intensity", 100); // 초기화된 레이저 오도메트리를 게시하기 위한 ROS 퍼블리셔를 초기화합니다.
    matched_keypoints_img_pub_front_end = nh_.advertise<sensor_msgs::Image>("/front_matched_keypoints_img", 10); // 일치하는 키포인트 이미지를 게시하기 위한 ROS 퍼블리셔를 초기화합니다.
    current_scan_.reset(new pcl::PointCloud<PointType>()); // 현재 스캔 포인트 클라우드를 초기화합니다.
    loop_scan_.reset(new pcl::PointCloud<PointType>()); // 루프 스캔 포인트 클라우드를 초기화합니다.

    std::string PROJECT_NAME("intensity_feature_tracker"); // 프로젝트 이름을 설정합니다.
    std::string pkg_path = ros::package::getPath(PROJECT_NAME); // 패키지 경로를 가져옵니다.
    skip_flag_ = false; // 스킵 플래그를 초기화합니다.
    skip_intensity_ = false; // 스킵 강도를 초기화합니다.
    
}

feature_tracker::~feature_tracker(){} // feature_tracker 클래스의 소멸자입니다.


void feature_tracker::detectcorners( ros::Time &time, 
                        const cv::Mat &image_intensity, 
                        const pcl::PointCloud<PointType>::Ptr cloud_track)
{
    // 이미지 강도와 포인트 클라우드를 이용하여 코너를 검출하는 메서드입니다.
    // 구현 내용이 주석으로 제공되지 않았습니다.
}

void feature_tracker::writeTF2file(ros::Time &time){
    // TF2 버퍼를 생성합니다.
    // tf2_ros::Buffer tfBuffer;
    // TF2 리스너를 생성하고 TF2 버퍼와 연결합니다.
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // TF 정보를 저장할 메시지를 생성합니다.
    // geometry_msgs::TransformStamped transformStamped;
    // try 블록을 사용하여 TF 정보를 조회합니다.
    // try{
    //     // "map"에서 "base_link"로의 TF 정보를 조회하고 시간(time)을 기준으로 가져옵니다.
    //     transformStamped = tfBuffer.lookupTransform("map","base_link",time);
    //     // 파일이 열려 있는지 확인한 후 TF 정보를 파일에 기록합니다.
    //     if(out_gtposes_.is_open()){
    //         // 시간, 위치(x, y, z), 회전(x, y, z, w) 정보를 파일에 기록합니다.
    //         out_gtposes_    << std::fixed << time.toSec() << " " 
    //                         << transformStamped.transform.translation.x << " " 
    //                         << transformStamped.transform.translation.y << " " 
    //                         << transformStamped.transform.translation.z << " " 
    //                         << transformStamped.transform.rotation.x << " " 
    //                         << transformStamped.transform.rotation.y << " " 
    //                         << transformStamped.transform.rotation.z << " " 
    //                         << transformStamped.transform.rotation.w << "\n"; 
    //     }
    //     // transformStamped.transform.rotation.w; 
    //     // time.toSec();
    // }
    // catch(tf2::TransformException &ex){
    //     // TF 정보를 조회하는 중에 오류가 발생하면 오류 메시지를 출력합니다.
    //     ROS_ERROR("%s", ex.what());
    //     //return false;
    // }
}

void feature_tracker::updatePoses(){
    // 임시 Pose3 객체를 생성합니다.
    gtsam::Pose3 tempEstimate;
    
    // isamCurrentEstimate_에 저장된 Pose3 개수만큼 반복합니다.
    for (size_t i = 0; i < isamCurrentEstimate_.size(); ++i){
        // factorMutex 뮤텍스를 사용하여 공유 데이터에 대한 접근을 보호합니다.
        const std::lock_guard<std::mutex> lock(factorMutex);
        
        // isamCurrentEstimate_에서 Pose3 객체를 가져와 임시 변수에 저장합니다.
        tempEstimate = isamCurrentEstimate_.at<gtsam::Pose3>(i);

        // 3D 포인트 클라우드의 해당 인덱스 위치에 Pose3의 x, y, z 값을 업데이트합니다.
        cloudKeyPoses3D_->points[i].x = tempEstimate.translation().x(); 
        cloudKeyPoses3D_->points[i].y = tempEstimate.translation().y();
        cloudKeyPoses3D_->points[i].z = tempEstimate.translation().z();

        // 6D 포인트 클라우드의 해당 인덱스 위치에 Pose3의 x, y, z, roll, pitch, yaw 값을 업데이트합니다.
        cloudKeyPoses6D_->points[i].x = tempEstimate.translation().x();
        cloudKeyPoses6D_->points[i].y = tempEstimate.translation().y();
        cloudKeyPoses6D_->points[i].z = tempEstimate.translation().z();
        cloudKeyPoses6D_->points[i].roll = tempEstimate.rotation().roll();
        cloudKeyPoses6D_->points[i].pitch = tempEstimate.rotation().pitch();
        cloudKeyPoses6D_->points[i].yaw = tempEstimate.rotation().yaw();
    }

    // 디버그 메시지를 출력합니다.
    ROS_DEBUG("updating poses: %li", isamCurrentEstimate_.size());

    {
        // factorMutex 뮤텍스를 사용하여 공유 데이터에 대한 접근을 보호합니다.
        const std::lock_guard<std::mutex> lock(factorMutex);
        // getLoop_ 변수를 false로 설정하여 루프 상태를 업데이트합니다.
        getLoop_ = false; 
    }  
}

void feature_tracker::icpThread(loopClosureProcessor::Keyframe keyframeNew){
    // ICP 스레드를 시작하고 현재 루프 클로저 프로세서의 상태를 출력합니다.
    std::cout << "\n\nicp thread," << loopClosureProcessor_.loop_index_ << ", keyframe id:" << keyframeNew.keyframeId << std::endl; 
}

Eigen::Matrix4d feature_tracker::getTransformMatrix(size_t index){
    // 특정 인덱스에 해당하는 변환 행렬을 계산하고 반환합니다.
    std::lock_guard<std::mutex> lock(factorMutex);
    ROS_INFO("getTransformMatrix, index: %li, size: %li", index, cloudKeyPoses6D_->points.size());
    auto p_tmp = cloudKeyPoses6D_->points[index];
    Eigen::Matrix4d T_tmp =  Eigen::Matrix4d::Identity();
    gtsam::Rot3 rot_temp = gtsam::Rot3::RzRyRx(p_tmp.yaw, p_tmp.pitch, p_tmp.roll);
    Eigen::Matrix3d R = rot_temp.toQuaternion().normalized().toRotationMatrix();// 3*3
    Eigen::Vector3d T(p_tmp.x, p_tmp.y, p_tmp.z);// 3*1
    T_tmp.block(0,0,3,3) = R; 
    T_tmp.block(0,3,3,1) = T; 
    T_tmp.block(3,0,1,4) << 0, 0, 0, 1;
    return T_tmp;
}

void feature_tracker::tranformCurrentScanToMap(pcl::PointCloud<PointType>::Ptr cloud_current_scan, size_t index, loopClosureProcessor::Keyframe keyframe_new){
    // 현재 스캔을 지도 좌표계로 변환합니다.
    Eigen::Matrix4d T_tmp = getTransformMatrix(index);
    std::cout << "T_tmp:"<< T_tmp << std::endl;
    pcl::transformPointCloud(keyframe_new.cloud_track_, *cloud_current_scan, T_tmp);
}

void feature_tracker::getSubmapOfhistory(pcl::PointCloud<PointType>::Ptr cloud_submap){
    size_t submap_window_size = 1; // 서브맵 윈도우 크기를 설정합니다.
    size_t i; 

    if(loopClosureProcessor_.loop_index_ < (int)submap_window_size){
        i = 0; // 루프 클로저 인덱스가 윈도우 크기보다 작으면 인덱스를 0으로 설정합니다.
    }
    else{
        i = loopClosureProcessor_.loop_index_ - submap_window_size;
    }
    for(; i < loopClosureProcessor_.loop_index_ + submap_window_size + 1; i++){
        if(i < loopClosureProcessor_.keyframe_pool_.size()){
            // 특정 인덱스에 해당하는 변환 행렬을 가져와서 클라우드를 변환하고 서브맵에 추가합니다.
            Eigen::Matrix4d T_tmp =  getTransformMatrix(i);
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(loopClosureProcessor_.keyframe_pool_[i].cloud_track_, *cloudOut, T_tmp);
            *cloud_submap += *cloudOut;
        }
    }
}

void feature_tracker::loopClosureThread(){
    ros::Rate rate(10); // ROS 루프의 속도를 초당 10회로 설정
    while (ros::ok()){  // ROS가 정상적으로 작동하는 동안 루프 실행
        size_t dequeSize = 0; // 덱(deque)의 크기를 저장할 변수 선언
        TicToc loop_tic;  // 타이머 시작 (TicToc은 사용자 정의 타이머 클래스로 추정)
        {
            const std::lock_guard<std::mutex> lock(factorMutex); // 뮤텍스로 스레드 동기화
            dequeSize = keyframesQueue_.size(); // 키 프레임 큐의 크기를 가져옴
        }

        if(dequeSize > 0){ // 큐에 키 프레임이 있으면
            std::shared_ptr<loopClosureProcessor::Keyframe> keyframeNewArriveTmp;
            loopClosureProcessor::Keyframe keyframeNewArrive;
            {
                const std::lock_guard<std::mutex> lock(factorMutex); // 뮤텍스로 스레드 동기화
                keyframeNewArriveTmp = keyframesQueue_.front(); // 큐의 첫 번째 키 프레임을 가져옴
                keyframesQueue_.pop_front(); // 큐에서 해당 키 프레임을 제거
            }
            keyframeNewArrive = *keyframeNewArriveTmp; // 키 프레임을 복사
            loopClosureProcessor_.keyframeProcessor(keyframeNewArrive); // 키 프레임 처리

            if(loopClosureProcessor_.loop_index_ >= 0){ // 루프 폐쇄가 발견되면             
                if(USE_ICP){ // ICP(Iterative Closest Point) 사용 여부 확인
                    pcl::IterativeClosestPoint<PointType, PointType> icp; // ICP 객체 생성
                    // ICP 설정
                    icp.setMaxCorrespondenceDistance(100);
                    icp.setMaximumIterations(100);
                    icp.setTransformationEpsilon(1e-6);
                    icp.setEuclideanFitnessEpsilon(1e-6);
                    icp.setRANSACIterations(0);
                    
                    pcl::CropBox<PointType> crop; // 포인트 클라우드를 자르기 위한 CropBox 객체 생성
                    pcl::VoxelGrid<PointType> vf_scan; // 다운샘플링을 위한 VoxelGrid 객체 생성
                    crop.setNegative(false); 
                    crop.setMin(Eigen::Vector4f(-CROP_SIZE, -CROP_SIZE, -CROP_SIZE, 1.0));
                    crop.setMax(Eigen::Vector4f(CROP_SIZE, CROP_SIZE, CROP_SIZE, 1.0));

                    vf_scan.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE); // Voxel 크기 설정
                    current_scan_->clear(); // 현재 스캔 데이터 초기화
                    loop_scan_->clear(); // 루프 스캔 데이터 초기화

                    {
                        ROS_INFO("current id: %li, size of keypose3d: %li", keyframeNewArrive.keyframeId, cloudKeyPoses3D_->points.size());
                        for(int i = 0; i < 40 && keyframeNewArrive.keyframeId >= cloudKeyPoses3D_->points.size(); i++){
                            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 대기
                        }
                        if(keyframeNewArrive.keyframeId >= cloudKeyPoses3D_->points.size()) {
                            ROS_ERROR("factor node is not updated in the end! return!");
                            continue; // 업데이트가 되지 않으면 계속
                        }
                        auto t_display = getTransformMatrix(keyframeNewArrive.keyframeId); // 변환 행렬 가져오기
                        std::cout << "original pose:" << t_display << std::endl;
                    }
                    // 현재 스캔을 맵에 변환
                    tranformCurrentScanToMap(current_scan_, keyframeNewArrive.keyframeId, keyframeNewArrive);     
                    getSubmapOfhistory(loop_scan_); // 역사적 서브맵 가져오기
                    if(loop_scan_->size() == 0){
                        ROS_ERROR("loop_scan_ is empty");
                        continue; // 루프 스캔이 비어있으면 계속
                    }
                    ROS_INFO("loop_scan_ size: %li", loop_scan_->size());
                    ROS_INFO("current_scan_ size: %li", current_scan_->size());

                    // NaN 제거
                    std::vector<int> idx;
                    current_scan_->is_dense = false;
                    pcl::removeNaNFromPointCloud(*current_scan_, *current_scan_, idx);
                    
                    loop_scan_->is_dense = false;
                    pcl::removeNaNFromPointCloud(*loop_scan_, *loop_scan_, idx);

                    if (USE_CROP) {
                        crop.setInputCloud(current_scan_);
                        crop.filter(*current_scan_); // 크롭 필터 적용
                    }
                    if (USE_DOWNSAMPLE) {
                        vf_scan.setInputCloud(current_scan_);
                        vf_scan.filter(*current_scan_); // 다운샘플링 적용
                    }

                    // 루프 클라우드에 대한 Crop Box 필터
                    if (USE_CROP) {
                        crop.setInputCloud(loop_scan_);
                        crop.filter(*loop_scan_); // 크롭 필터 적용
                    }
                    if (USE_DOWNSAMPLE) {
                        vf_scan.setInputCloud(loop_scan_);
                        vf_scan.filter(*loop_scan_); // 다운샘플링 적용
                    }

                    // 루프 스캔과 현재 스캔을 발행
                    sensor_msgs::PointCloud2 cloudMsgTemp;
                    pcl::toROSMsg(*loop_scan_, cloudMsgTemp);
                    cloudMsgTemp.header.stamp = ros::Time().now();
                    cloudMsgTemp.header.frame_id = "map";
                    pubLoopScan_.publish(cloudMsgTemp); // 루프 스캔을 ROS 메시지로 변환하여 발행

                    pcl::toROSMsg(*current_scan_, cloudMsgTemp);
                    cloudMsgTemp.header.stamp = ros::Time().now();
                    cloudMsgTemp.header.frame_id = "map";
                    pubCurrentScan_.publish(cloudMsgTemp); // 현재 스캔을 ROS 메시지로 변환하여 발행

                    ROS_INFO("loop_scan_DS size: %li", loop_scan_->size());
                    ROS_INFO("current_scan_DS size: %li", current_scan_->size());

                    // 클라우드 정렬
                    if(loop_scan_->points.size()>10 && current_scan_->points.size()>10){
                        icp.setInputSource(current_scan_); // ICP의 소스 클라우드로 현재 스캔 설정
                        icp.setInputTarget(loop_scan_); // ICP의 타겟 클라우드로 루프 스캔 설정
                        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
                        icp.align(*unused_result); // ICP를 사용하여 클라우드 정렬
                        
                        if (icp.hasConverged() != false){
                            std::cout << "\n icp is converged and fitnessScore is: " << icp.getFitnessScore(); // ICP가 수렴되었으면 피트니스 점수 출력
                        }
                        else{
                            std::cout << "\n icp is not converged"; // ICP가 수렴하지 않았으면 메시지 출력
                        }

                        if (icp.hasConverged() != false && icp.getFitnessScore() <= FITNESS_SCORE){
                        // ICP가 수렴하고 피트니스 점수가 임계값 이하인 경우
                        Eigen::Matrix4d T_before, T_after, T_temp, T_cur2map, T_loop2map, T_cur2map_gt;
                        
                        auto T_icp_cur2loop_bias = icp.getFinalTransformation(); // ICP의 최종 변환 행렬을 가져옴
                        std::cout << "\n icp result:\n" << T_icp_cur2loop_bias << std::endl; // ICP 결과 출력
                        T_temp = T_icp_cur2loop_bias.cast<double>(); // 변환 행렬을 double 타입으로 변환
                        
                        T_cur2map = getTransformMatrix(keyframeNewArrive.keyframeId); // 현재 키 프레임의 변환 행렬을 가져옴
                        T_loop2map = getTransformMatrix(loopClosureProcessor_.loop_index_); // 루프 폐쇄 키 프레임의 변환 행렬을 가져옴
                        T_cur2map_gt = T_temp * T_cur2map; // 현재 키 프레임에서 맵까지의 변환 행렬 계산
                    
                        Eigen::Matrix3d rot_cur = T_cur2map_gt.block(0,0,3,3); // 회전 부분만 추출
                        Eigen::Quaterniond q_cur(rot_cur); // 회전 행렬을 쿼터니언으로 변환
                        Eigen::Vector3d t_cur(T_cur2map_gt.block(0,3,3,1)); // 변환 행렬에서 이동 벡터 추출                           
                        Eigen::Vector3d cur_euler = q_cur.toRotationMatrix().eulerAngles(2, 1, 0); // 쿼터니언을 오일러 각으로 변환
                        
                        gtsam::Pose3 pose_from = gtsam::Pose3(gtsam::Rot3::RzRyRx(cur_euler[0], cur_euler[1], cur_euler[2]), gtsam::Point3(t_cur[0], t_cur[1], t_cur[2]));
                        // GTSAM 라이브러리를 사용하여 현재 키 프레임의 포즈를 생성
                        Eigen::Matrix3d after_rotation; // 후속 처리를 위한 회전 행렬 선언
                    
                        Eigen::Matrix3d rot_loop = T_loop2map.block(0,0,3,3); // 루프 폐쇄 키 프레임의 회전 행렬 추출
                        Eigen::Quaterniond q_loop(rot_loop); // 회전 행렬을 쿼터니언으로 변환        
                        Eigen::Vector3d t_loop(T_loop2map.block(0,3,3,1)); // 루프 폐쇄 키 프레임의 이동 벡터 추출                      
                        Eigen::Vector3d loop_euler = q_loop.toRotationMatrix().eulerAngles(2, 1, 0); // 쿼터니언을 오일러 각으로 변환

                        gtsam::Pose3 pose_to = gtsam::Pose3(gtsam::Rot3::RzRyRx(loop_euler[0], loop_euler[1], loop_euler[2]), gtsam::Point3(t_loop[0], t_loop[1], t_loop[2]));

                        // GTSAM 라이브러리를 사용하여 루프 폐쇄 키 프레임의 포즈 생성
                        
                        {
                            gtsam::Vector Vector6_loop(6); // 6차원 벡터 생성
                            float noiseScore = icp.getFitnessScore(); // ICP의 피트니스 점수를 가져옴
                            Vector6_loop << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore; // 벡터에 피트니스 점수 할당
                            constraintNoise_ = gtsam::noiseModel::Diagonal::Variances(Vector6_loop); // 대각선 노이즈 모델 생성
                        
                            robustNoiseModel_ = gtsam::noiseModel::Robust::Create(
                                gtsam::noiseModel::mEstimator::Cauchy::Create(1), 
                                gtsam::noiseModel::Diagonal::Variances(Vector6_loop)
                            ); // 로버스트 노이즈 모델 생성
                        
                            const std::lock_guard<std::mutex> lock(factorMutex); // 뮤텍스로 스레드 동기화                                
                            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(                            
                                    keyframeNewArrive.keyframeId,   
                                    loopClosureProcessor_.loop_index_,                         
                                    pose_from.between(pose_to),
                                    constraintNoise_));// 포즈 그래프에 두 포즈 간의 관계 추가
                            ROS_INFO("icp method: graph between factor of loop added"); // 로그 메시지 출력                                
                            getLoop_ = true; // 루프 감지 플래그 설정                                
                        }

    
                            }
                        }
                        ROS_INFO("time of ICP: %i ms", (int)loop_tic.toc()); // ICP 처리 시간을 밀리초 단위로 출력
                        
                                        } // ICP 사용 블록 끝
                        
                                    } // 루프 폐쇄 처리 블록 끝
                        
                                    if(getLoop_ == true){
                                        updatePoses(); // 포즈 업데이트 함수 호출
                                        sensor_msgs::PointCloud2 cloudMsgTemp;
                                        pcl::toROSMsg(*cloudKeyPoses3D_, cloudMsgTemp); // 3D 키 포즈들을 ROS 메시지로 변환
                                        cloudMsgTemp.header.stamp = ros::Time().now(); // 현재 시간으로 타임스탬프 설정
                                        cloudMsgTemp.header.frame_id = "map"; // 프레임 ID를 'map'으로 설정
                                        pubKeyPoses_.publish(cloudMsgTemp); // 키 포즈 메시지 발행
                                    }
                        
                                } // 메인 루프 블록 끝
                                ROS_INFO("loop time: %lf ms", loop_tic.toc()); // 전체 루프 처리 시간을 밀리초 단위로 출력
                                loop_time_ = loop_tic.toc(); // 전체 루프 처리 시간을 변수에 저장
                                
                                rate.sleep(); // 설정된 레이트에 따라 대기
                        
                                // 루프 종료 부분
                                
                            } // ROS OK 체크 루프 끝
                        
                        } // 함수 종료

    void feature_tracker::mapOdomHandle(){
    
        // mapOdomQueue_가 비어있지 않고 factorGraphNode_도 비어있지 않으며,
        // mapOdomQueue_의 가장 최근 메시지의 타임스탬프가 factorGraphNode_의 첫 번째 타임스탬프보다 큰 경우 반복
        while (!mapOdomQueue_.empty() && !factorGraphNode_.empty() && mapOdomQueue_.back()->header.stamp.toSec() >= factorGraphNode_.front()->time)
        {
            factorMutex.lock(); // 팩터 처리를 위한 뮤텍스 락
            // mapOdomQueue_가 비어있지 않고, 가장 오래된 메시지의 타임스탬프가 factorGraphNode_의 첫 번째 타임스탬프보다 작은 경우
            while(!mapOdomQueue_.empty() && mapOdomQueue_.front()->header.stamp.toSec() < factorGraphNode_.front()->time){  
                mapOdomQueue_.pop(); // mapOdomQueue_에서 메시지를 제거
            }
            factorMutex.unlock(); // 뮤텍스 언락
            
            if (mapOdomQueue_.empty())
            {
                continue; // mapOdomQueue_가 비어있으면 다음 반복으로 이동
            }
            // adding factor
            loopClosureProcessor::FactorGraphNode factorNewArrive;
            std::shared_ptr<loopClosureProcessor::FactorGraphNode> factorNewArriveTmp;
            // lock
            {
                const std::lock_guard<std::mutex> lock(factorMutex); // 팩터 처리를 위한 뮤텍스 락
                factorNewArriveTmp = factorGraphNode_.front(); // factorGraphNode_의 첫 번째 요소를 가져옴
                factorGraphNode_.pop_front(); // factorGraphNode_에서 첫 번째 요소를 제거
            }
            factorNewArrive = *factorNewArriveTmp; // factorNewArrive에 가져온 요소를 할당
    
            factorMutex.lock(); // 팩터 처리를 위한 뮤텍스 락
            auto odom_curr = mapOdomQueue_.front(); // mapOdomQueue_의 가장 오래된 메시지를 가져옴
            mapOdomQueue_.pop(); // mapOdomQueue_에서 메시지 제거
            factorMutex.unlock(); // 뮤텍스 언락
    
            Eigen::Quaterniond q_tmp;
            q_tmp.x() = odom_curr->pose.pose.orientation.x;
            q_tmp.y() = odom_curr->pose.pose.orientation.y;
            q_tmp.z() = odom_curr->pose.pose.orientation.z;
            q_tmp.w() = odom_curr->pose.pose.orientation.w;
            Eigen::Vector3d euler = q_tmp.toRotationMatrix().eulerAngles(2, 1, 0); 
            // 현재 오도메트리 메시지에서 쿼터니언을 추출하고, 쿼터니언을 오일러 각도로 변환
    
            Eigen::Vector3d t_wodom_curr;
            t_wodom_curr.x() = odom_curr->pose.pose.position.x;
            t_wodom_curr.y() = odom_curr->pose.pose.position.y;
            t_wodom_curr.z() = odom_curr->pose.pose.position.z;
            // 현재 오도메트리 메시지에서 위치 벡터를 추출
    
            if(cloudKeyPoses3D_->points.empty()){ // 만약 cloudKeyPoses3D_가 비어 있다면 (처음에 실행)
            const std::lock_guard<std::mutex> lock(factorMutex); // 뮤텍스 락을 획득
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>( // gtSAM 그래프에 Prior Factor 추가
                factorNewArrive.keyframeId, // 현재 키프레임의 ID
                gtsam::Pose3( // 로봇의 위치와 자세를 나타내는 Pose3 객체 생성
                    gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]), // 로봇의 자세(오일러 각도)
                    gtsam::Point3( // 로봇의 위치를 나타내는 3D 포인트
                        t_wodom_curr.x(), // x 좌표
                        t_wodom_curr.y(), // y 좌표
                        t_wodom_curr.z()  // z 좌표
                    )
                ),
                priorNoise_ // Prior Factor의 노이즈 모델
            ));
            initialEstimate_.insert(factorNewArrive.keyframeId, // 초기 추정값에 현재 키프레임의 ID와 Pose3 추가
                gtsam::Pose3( // 로봇의 위치와 자세를 나타내는 Pose3 객체 생성
                    gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]), // 로봇의 자세(오일러 각도)
                    gtsam::Point3( // 로봇의 위치를 나타내는 3D 포인트
                        t_wodom_curr.x(), // x 좌표
                        t_wodom_curr.y(), // y 좌표
                        t_wodom_curr.z()  // z 좌표)));
                }
    
                else { // 그 외 경우 (처음이 아닌 경우, 두 번째 이후 실행)
        // 이전 로봇 자세의 쿼터니언(q)을 사용하여 이전 로봇의 오일러 각도를 계산
        Eigen::Quaterniond q;
        q = graph_prev_rotation_;
        Eigen::Vector3d prev_euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // 이전 로봇의 오일러 각도 계산
    
        // 이전 키프레임에서 현재 키프레임으로의 Pose3 객체를 생성
        gtsam::Pose3 pose_from = gtsam::Pose3(
            gtsam::Rot3::RzRyRx(prev_euler[0], prev_euler[1], prev_euler[2]), // 이전 로봇의 자세(오일러 각도)
            gtsam::Point3( // 이전 로봇의 위치를 나타내는 3D 포인트
                graph_prev_translation_.x(), // x 좌표
                graph_prev_translation_.y(), // y 좌표
                graph_prev_translation_.z()  // z 좌표
            )
        );
    
        // 현재 키프레임에서의 Pose3 객체를 생성
        gtsam::Pose3 pose_to = gtsam::Pose3(
            gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]), // 현재 로봇의 자세(오일러 각도)
            gtsam::Point3( // 현재 로봇의 위치를 나타내는 3D 포인트
                t_wodom_curr.x(), // x 좌표
                t_wodom_curr.y(), // y 좌표
                t_wodom_curr.z()  // z 좌표
            )
        );
    
        const std::lock_guard<std::mutex> lock(factorMutex); // 뮤텍스 락을 획득
        gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>( // gtSAM 그래프에 Between Factor 추가
            factorNewArrive.keyframeId - 1, // 이전 키프레임 ID
            factorNewArrive.keyframeId, // 현재 키프레임 ID
            pose_from.between(pose_to), // 두 Pose3 사이의 관계 설정
            odometryNoise_ // Between Factor의 노이즈 모델
        ));
    
        // 초기 추정값에 현재 키프레임의 ID와 Pose3 추가
        initialEstimate_.insert(factorNewArrive.keyframeId,
            gtsam::Pose3(
                gtsam::Rot3::RzRyRx(euler[0], euler[1], euler[2]), // 현재 로봇의 자세(오일러 각도)
                gtsam::Point3( // 현재 로봇의 위치를 나타내는 3D 포인트
                    t_wodom_curr.x(), // x 좌표
                    t_wodom_curr.y(), // y 좌표
                    t_wodom_curr.z()  // z 좌표
                )
            )
        );
    }
    
    // 다음 루프를 위해 현재 로봇의 회전 및 이동 정보를 저장
    graph_prev_rotation_ = q_tmp;
    graph_prev_translation_ = t_wodom_curr;

{
    const std::lock_guard<std::mutex> lock(factorMutex); // 뮤텍스 락을 획득하여 동기화
    isam_->update(gtSAMgraph_, initialEstimate_); // ISAM 업데이트를 수행하고 그래프 및 초기 추정값을 사용
    isam_->update(); // ISAM 업데이트를 추가로 수행하여 최종 추정값 계산

    gtSAMgraph_.resize(0); // gtSAM 그래프를 초기화
    initialEstimate_.clear(); // 초기 추정값을 지움

    PointType thisPose3D; // 3D 포인트를 저장하는 변수
    PointTypePose thisPose6D; // 6D 포인트를 저장하는 변수

    gtsam::Pose3 latestEstimate; // 최신 추정값을 저장하는 변수
    isamCurrentEstimate_ = isam_->calculateEstimate(); // 현재 추정값 계산
    latestEstimate = isamCurrentEstimate_.at<gtsam::Pose3>(isamCurrentEstimate_.size() - 1); // 최신 추정값을 가져옴

    // 3D 포인트에 위치 및 강도 정보 추가
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D_->points.size(); // 클라우드의 포인트 수를 강도로 사용
    cloudKeyPoses3D_->push_back(thisPose3D); // 3D 클라우드에 포인트 추가

    // 6D 포인트에 위치, 강도, 회전 정보 추가
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = factorNewArrive.time; // 시간 정보 추가
    cloudKeyPoses6D_->push_back(thisPose6D); // 6D 클라우드에 포인트 추가
}

if (getLoop_ == true) { // 루프 클로저가 있는 경우
    updatePoses(); // 자세 업데이트 수행
}

// tf broadcast
gtsam::Pose3 latestEstimate; // 최신 추정값을 저장하는 변수
{
    const std::lock_guard<std::mutex> lock(factorMutex); // 뮤텍스 락을 획득하여 동기화
    isamCurrentEstimate_ = isam_->calculateEstimate(); // 현재 추정값 계산
    latestEstimate = isamCurrentEstimate_.at<gtsam::Pose3>(isamCurrentEstimate_.size() - 1); // 최신 추정값을 가져옴
}

auto q = latestEstimate.rotation().toQuaternion(); // 추정된 자세로부터 쿼터니언 회전 정보 가져오기
static tf2_ros::TransformBroadcaster br; // TF2의 TransformBroadcaster 생성
geometry_msgs::TransformStamped transformStamped; // ROS의 TransformStamped 메시지 생성
transformStamped.header.stamp = ros::Time().fromSec(factorNewArrive.time); // 타임스탬프 설정
transformStamped.header.frame_id = "map"; // 현재 프레임 설정
transformStamped.child_frame_id = "pgo_odom"; // 자식 프레임 설정
transformStamped.transform.translation.x = latestEstimate.translation().x(); // 변환 정보 (X) 설정
transformStamped.transform.translation.y = latestEstimate.translation().y(); // 변환 정보 (Y) 설정
transformStamped.transform.translation.z = latestEstimate.translation().z(); // 변환 정보 (Z) 설정
transformStamped.transform.rotation.w = q.w(); // 회전 정보 (w) 설정
transformStamped.transform.rotation.x = q.x(); // 회전 정보 (x) 설정
transformStamped.transform.rotation.y = q.y(); // 회전 정보 (y) 설정
transformStamped.transform.rotation.z = q.z(); // 회전 정보 (z) 설정
br.sendTransform(transformStamped); // TF2로 변환 정보를 브로드캐스트

// keypose를 게시
sensor_msgs::PointCloud2 cloudMsgTemp; // ROS 메시지로 변환할 PointCloud2 생성
pcl::toROSMsg(*cloudKeyPoses3D_, cloudMsgTemp); // PointCloud2로 변환
cloudMsgTemp.header.stamp = ros::Time().fromSec(factorNewArrive.time); // 타임스탬프 설정
cloudMsgTemp.header.frame_id = "map"; // 현재 프레임 설정
pubKeyPoses_.publish(cloudMsgTemp); // keypose를 게시

void feature_tracker::factorGraphThread(){
    ros::Rate rate(100); // 주기를 100Hz로 설정한 ROS Rate 객체 생성
    while (ros::ok()){ // ROS가 정상 동작하는 동안 반복 실행
        if(true){ // 항상 참인 조건 (항상 실행)
            TicToc pgo_time; // 시간 측정을 위한 TicToc 객체 생성
            mapOdomHandle(); // mapOdomHandle 함수 호출 (무엇을 하는 함수인지는 주석에 없음)
            ROS_INFO("pgo_time: %f", pgo_time.toc()); // pgo_time 출력
            pgo_time_ = pgo_time.toc(); // pgo_time_ 변수에 측정 시간 저장
        }  
        rate.sleep(); // 주기를 유지하기 위해 대기
    }
}

void feature_tracker::detectfeatures(ros::Time &time, 
                                     const cv::Mat &image_intensity, 
                                     const pcl::PointCloud<PointType>::Ptr _cloud_track,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoint_)
{
    // 새로운 키프레임을 처리합니다.
    TicToc intensity_odom_time;
    TicToc intensity_feature_extraction_time;
    static int global_frame_index = 0;
    if(global_frame_index == 0) first_frame_time_ = time.toSec(); // 처음 프레임의 시간을 기록합니다.
    time_stamp_ = time; // 현재 시간을 기록합니다.
    status_.clear(); // 상태를 초기화합니다.
    cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);

    cv::Mat image = image_intensity; // 주어진 이미지를 변수에 저장합니다.
    cloudTrack_ = _cloud_track; // 포인트 클라우드를 저장합니다.
    cur_cloud_ = *_cloud_track;
    ground_cloud_ = *groundPoint_;
    
    cur_img_ = image;

    // 특징점을 추출합니다.
    TicToc detectOrbFeaturesTime; // 시간 측정을 시작합니다.
    detector->detect(cur_img_, cur_keypoints_, MASK); // ORB 디텍터를 사용하여 키포인트를 검출합니다.
    
    cur_orb_point_2d_uv_ = keypoint2uv(cur_keypoints_); // 키포인트를 2D 좌표로 변환합니다.
    extractPointsAndFilterZeroValue(cur_orb_point_2d_uv_, cloudTrack_, cur_out_point3d_, status_); // 키포인트와 포인트 클라우드를 연결하고 0 값 필터링을 수행합니다.
    reduceVector(cur_out_point3d_, status_); // 벡터를 축소합니다.
    reduceVector(cur_orb_point_2d_uv_, status_); // 벡터를 축소합니다.
    reduceVector(cur_keypoints_, status_); // 벡터를 축소합니다.

    detector->compute(cur_img_, cur_keypoints_, cur_descriptors_); // 이미지에서 키포인트의 디스크립터를 계산합니다.

    std::vector<cv::DMatch> matches, good_matches; 
    cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
    
    if(!prev_img_.empty()){ // 첫 번째 프레임이 아닌 경우에만 특징점 매칭을 수행합니다.
        TicToc matchTime;                 
        matcher.match(cur_descriptors_, prev_descriptors_, matches); // 현재 디스크립터와 이전 디스크립터 간의 매칭을 수행합니다.
        ROS_DEBUG("cur_keyPoints num:%ld", cur_keypoints_.size());
        ROS_DEBUG("prev_keyPoints num:%ld", prev_keypoints_.size());
        ROS_DEBUG("Matches num:%ld, matched time:%f", matches.size(), matchTime.toc());
        
        std::sort(matches.begin(), matches.end());
    
        //remove bad matching
        for (size_t i = 0; i < matches.size() * 0.3; ++i)
        {
            good_matches.push_back(matches[i]);
        }
        ROS_DEBUG("good_matches num:%ld", good_matches.size());
    
        std::string detectTime = std::to_string(detectOrbFeaturesTime.toc()); // 시간 측정을 종료합니다.
    
        // 다시 매칭을 수행합니다.
        if(!(prev_keypoints_.size() != cur_keypoints_.size() && good_matches.size() >=4 && good_matches.size()!= matches.size())){
            // 현재 프레임과 이전 프레임에 대해 다시 특징점을 검출합니다.
            TicToc reDetectTime;
            cv::Ptr<cv::ORB> detector2 = cv::ORB::create(NUM_ORB_FEATURES*2, 1.2f, 8, 1);
    
            // 현재 프레임에 대해 특징점을 검출합니다.
            detector2->detect(cur_img_, cur_keypoints_, MASK);
            cur_orb_point_2d_uv_ = keypoint2uv(cur_keypoints_);
            extractPointsAndFilterZeroValue(cur_orb_point_2d_uv_, cloudTrack_, cur_out_point3d_, status_);
            reduceVector(cur_out_point3d_, status_);
            reduceVector(cur_orb_point_2d_uv_, status_);
            reduceVector(cur_keypoints_, status_);
            detector2->compute(cur_img_, cur_keypoints_, cur_descriptors_);

            // 이전 프레임에 대해 특징점을 검출하고 매칭을 수행합니다.
            // 현재 프레임과 이전 프레임 간의 매칭을 재수행하는 경우입니다.
            detector2->detect(prev_img_, prev_keypoints_, MASK); // 이전 이미지에서 특징점을 검출합니다.
            prev_orb_point_2d_uv_ = keypoint2uv(prev_keypoints_); // 이전 특징점을 2D 좌표로 변환합니다.
            pcl::PointCloud<PointType>::Ptr prev_cloud_ptr_(new pcl::PointCloud<PointType>(prev_cloud_)); // 이전 포인트 클라우드를 포인터로 생성합니다.
            
            extractPointsAndFilterZeroValue(prev_orb_point_2d_uv_, prev_cloud_ptr_, prev_out_point3d_, status_); // 이전 프레임의 특징점과 포인트 클라우드를 필터링합니다.
            reduceVector(prev_out_point3d_, status_); // 필터링된 포인트 클라우드를 축소합니다.
            reduceVector(prev_orb_point_2d_uv_, status_); // 필터링된 특징점을 축소합니다.
            reduceVector(prev_keypoints_, status_); // 필터링된 특징점을 축소합니다.
            detector2->compute(prev_img_, prev_keypoints_, prev_descriptors_); // 이전 이미지에서 특징점의 디스크립터를 계산합니다.
            
            matcher.match(cur_descriptors_, prev_descriptors_, matches); // 현재 디스크립터와 이전 디스크립터 간의 매칭을 수행합니다.
            
            std::sort(matches.begin(), matches.end());
            
            // 나쁜 매칭을 제거합니다.
            good_matches.clear();
            for (size_t i = 0; i < matches.size() * 0.2; ++i)
            {
                good_matches.push_back(matches[i]);
            }
            ROS_DEBUG("good_matches num:%ld", good_matches.size());
            
            detectTime = "re-detect" + std::to_string(detectOrbFeaturesTime.toc()); // 시간 측정을 종료합니다.
            // std::cout << "re-detect time:" << reDetectTime.toc() << " ms" << std::endl;
        }
        if (prev_keypoints_.size() != cur_keypoints_.size() && good_matches.size() >= 4 && good_matches.size() != matches.size()) {
    global_frame_index++; // 전역 프레임 인덱스를 증가시킵니다.
    ROS_DEBUG("global frame:%d", global_frame_index); // 전역 프레임 인덱스를 디버그로 출력합니다.

    // 프레임 처리 주파수를 계산하고 출력 문자열에 추가합니다.
    frequency_ = round(1.0 * global_frame_index / (time.toSec() - first_frame_time_));
    std::string frequency_s = std::to_string(frequency_);
    std::string name(", Hz:");
    detectTime += name;
    detectTime += frequency_s;

    // 매칭된 특징점이 표시된 이미지를 표시합니다.
    image_show(good_matches, detectTime);

    // 매칭된 포인트 3D 좌표를 추출합니다.
    std::vector<cv::Point3f> prev_matched_points3d, cur_matched_points3d;
    extractMatchedPoints(good_matches, prev_matched_points3d, cur_matched_points3d, prev_out_point3d_, cur_out_point3d_);

    ROS_INFO("features detectation time:%lf ms", intensity_feature_extraction_time.toc()); // 특징점 검출 시간을 출력합니다.
    feature_extraction_time_ = intensity_feature_extraction_time.toc();

    // 회전 및 변환 행렬을 계산합니다.
    TicToc scan_registration_time;
    T_s2s_ = p2p_calculateRandT(cur_matched_points3d, prev_matched_points3d);
    scan_matching_time_ = scan_registration_time.toc(); // 스캔 매칭 시간을 측정합니다.

    tfBroadcast(); // 변환 행렬을 브로드캐스트합니다.

    getKeyframe(time_stamp_.toSec(), cur_out_point3d_); // 키프레임을 가져옵니다.
} else { // 나쁜 프레임인 경우, 이 프레임을 건너뛰고 기하학적 특징을 사용하여 변환 행렬을 계산합니다.
    static size_t skip_count = 0;
    skip_count++;
    ROS_ERROR("skip this bad frame:%ld ", skip_count); // 나쁜 프레임을 건너뛰는 로그를 출력합니다.
    skip_flag_ = true;
    skip_intensity_ = true;
    T_s2s_ = Eigen::Matrix4d::Identity(); // 변환 행렬을 단위 행렬로 설정합니다.
    tfBroadcast(); // 변환 행렬을 브로드캐스트합니다.
}

prev_cloud_ = cur_cloud_; // 이전 포인트 클라우드를 현재 클라우드로 업데이트합니다.
prev_img_ = cur_img_; // 이전 이미지를 현재 이미지로 업데이트합니다.
prev_keypoints_ = cur_keypoints_; // 이전 특징점을 현재 특징점으로 업데이트합니다.
prev_descriptors_ = cur_descriptors_; // 이전 디스크립터를 현재 디스크립터로 업데이트합니다.
prev_orb_point_2d_uv_ = cur_orb_point_2d_uv_; // 이전 ORB 포인트를 현재 ORB 포인트로 업데이트합니다.
prev_out_point3d_ = cur_out_point3d_; // 이전 포인트 3D를 현재 포인트 3D로 업데이트합니다.
}


void feature_tracker::getKeyframe(double time, std::vector<cv::Point3f> &featurMatched3Dpoints){
    // 현재 포지션 및 회전 정보를 가져옵니다.
    cur_position_ = T_s2m_.block(0,3,3,1); 
    cur_rotation_ = T_s2m_.block(0,0,3,3);

    double image_time = time;
    static double last_skip_time = -1; 

    if(cloudKeyPoses3D_->points.empty()){
        {
            const std::lock_guard<std::mutex> lock(factorMutex);

            // 새로운 FactorGraphNode을 생성하여 factorGraphNode_에 추가합니다.
            std::shared_ptr<loopClosureProcessor::FactorGraphNode> factorNew(new loopClosureProcessor::FactorGraphNode(time, keyframeId_, cur_position_, prev_position_, cur_rotation_, prev_rotation_)); 
            factorGraphNode_.emplace_back(factorNew);
        }
        {
            const std::lock_guard<std::mutex> lock(factorMutex);

            // 새로운 Keyframe을 생성하여 keyframesQueue_에 추가합니다.
            std::shared_ptr<loopClosureProcessor::Keyframe> keyframeNew(new loopClosureProcessor::Keyframe(time, keyframeId_, cur_img_, cur_keypoints_, cur_descriptors_, featurMatched3Dpoints, cur_position_, prev_position_, cur_rotation_, prev_rotation_, *cloudTrack_));
            keyframesQueue_.emplace_back(keyframeNew);
        }

        // 파라미터 업데이트 및 초기화
        last_skip_time = image_time; 
        prev_position_ = cur_position_;
        prev_rotation_ = cur_rotation_;  
        keyframeId_++;
        return;
    }

    // 다른 키프레임 업데이트
    if(image_time - last_skip_time > KEYFRAME_TIME_INTERVAL){
        double distance = (cur_position_ - prev_position_).norm();

        if(distance > KEYFRAME_DISTANCE_INTERVAL){
            // keyframe_ 구조체에 현재 프레임 정보 저장
            keyframe_ = {
                time,
                keyframeId_,
                cur_img_,
                cur_keypoints_,
                cur_descriptors_,
                featurMatched3Dpoints,
                cur_position_,
                prev_position_,
                cur_rotation_,
                prev_rotation_,
                *cloudTrack_
            };

            {
                const std::lock_guard<std::mutex> lock(factorMutex);

                // 새로운 FactorGraphNode을 생성하여 factorGraphNode_에 추가합니다.
                std::shared_ptr<loopClosureProcessor::FactorGraphNode> factorNew(new loopClosureProcessor::FactorGraphNode(time, keyframeId_, cur_position_, prev_position_, cur_rotation_, prev_rotation_)); 
                factorGraphNode_.emplace_back(factorNew);
            }
            {
                const std::lock_guard<std::mutex> lock(factorMutex);

                // 새로운 Keyframe을 생성하여 keyframesQueue_에 추가합니다.
                std::shared_ptr<loopClosureProcessor::Keyframe> keyframeNew(new loopClosureProcessor::Keyframe(time, keyframeId_, cur_img_, cur_keypoints_, cur_descriptors_, featurMatched3Dpoints, cur_position_, prev_position_, cur_rotation_, prev_rotation_, *cloudTrack_));
                keyframesQueue_.emplace_back(keyframeNew);
            }

            // 파라미터 업데이트 및 초기화
            last_skip_time = image_time; 
            prev_position_ = cur_position_;
            prev_rotation_ = cur_rotation_;  
            keyframeId_++;
        }
    }
}

void feature_tracker::tfBroadcast(){
    // 전역 변환 행렬
    T_s2m_ = T_s2m_ * T_s2s_;

    {
        const std::lock_guard<std::mutex> lock(factorMutex);
        T_s2pgo_ = T_s2pgo_ * T_s2s_;
    }

    // 전역 회전 정보 추출
    Eigen::Matrix3d rot; 
    rot = T_s2m_.block(0,0,3,3);
    Eigen::Quaterniond q(rot);    
    q = q.normalized();

    // 전역 위치 정보 추출
    Eigen::Vector3d p(T_s2m_.block(0,3,3,1));

    // 브로드캐스트
    static tf2_ros::TransformBroadcaster br; 
    geometry_msgs::TransformStamped transformStamped; 

    transformStamped.header.stamp = time_stamp_;

    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "intensity_odom";

    transformStamped.transform.translation.x = p[0];
    transformStamped.transform.translation.y = p[1];
    transformStamped.transform.translation.z = p[2];

    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();

    br.sendTransform(transformStamped);

    // 레이저 오도메트리 발행
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/map";
    
    if(skip_flag_){
        skip_flag_ = false;
        laserOdometry.child_frame_id = "/odom_skip";
    }
    else{
        laserOdometry.child_frame_id = "/laser_odom";
    }

    laserOdometry.header.stamp = time_stamp_;
    laserOdometry.pose.pose.orientation.x = q.x();
    laserOdometry.pose.pose.orientation.y = q.y();
    laserOdometry.pose.pose.orientation.z = q.z();
    laserOdometry.pose.pose.orientation.w = q.w();
    laserOdometry.pose.pose.position.x = p[0];
    laserOdometry.pose.pose.position.y = p[1];
    laserOdometry.pose.pose.position.z = p[2];

    pubLaserOdometry_.publish(laserOdometry);
}


Eigen::Matrix4d feature_tracker::p2p_calculateRandT(std::vector<cv::Point3f> &src_cloud, std::vector<cv::Point3f> &dst_cloud){
    // 초기 Ceres Solver 매개변수 설정
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); // 손실 함수 설정
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization(); // 로컬 파라미터화 방법 설정
    ceres::Problem::Options problem_options; // 문제 옵션 설정

    ceres::Problem problem(problem_options); // Ceres 문제 생성
    double fe_parameters_[7] = {0, 0, 0, 1, 0, 0, 0}; // 전방 회전 및 변환 파라미터 초기화
    problem.AddParameterBlock(fe_parameters_, 4, q_parameterization); // 회전 파라미터 블록 추가
    problem.AddParameterBlock(fe_parameters_ + 4, 3); // 변환 파라미터 블록 추가

    // src_cloud 및 dst_cloud의 각 포인트를 반복 처리
    for (size_t i = 0; i < src_cloud.size(); i++)
    {
        Eigen::Vector3d src_point = Eigen::Vector3d(src_cloud[i].x, src_cloud[i].y, src_cloud[i].z); // 소스 클라우드 포인트
        Eigen::Vector3d dst_point = Eigen::Vector3d(dst_cloud[i].x, dst_cloud[i].y, dst_cloud[i].z); // 대상 클라우드 포인트
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<front_end_residual, 3, 4, 3>(
                    new front_end_residual(src_point, dst_point)); // 비용 함수 생성
        problem.AddResidualBlock(cost_function, loss_function, fe_parameters_, fe_parameters_ + 4); // 비용 함수 및 매개변수 블록 추가
    }

    // 문제 해결
    ceres::Solver::Options options; // 솔버 옵션 설정
    options.linear_solver_type = ceres::DENSE_QR; // 선형 솔버 유형 설정
    options.max_num_iterations = 20; // 최대 반복 횟수 설정
    options.minimizer_progress_to_stdout = false; // 최적화 진행 정보 출력 설정
    ceres::Solver::Summary summary; // 솔버 요약 정보
    ceres::Solve(options, &problem, &summary); // 문제 해결

    // q 및 t 업데이트
    Eigen::Map<Eigen::Quaterniond> q_w_curr(fe_parameters_); // Quaternion 업데이트
    Eigen::Map<Eigen::Vector3d> t_w_curr(fe_parameters_ + 4); // 변환 벡터 업데이트

    // q를 회전 행렬로 변환
    Eigen::Matrix3d R_w_curr = q_w_curr.toRotationMatrix(); // Quaternion을 회전 행렬로 변환

    // R 및 T를 T_s2s_로 변환
    Eigen::Matrix3d R = R_w_curr; // 회전 행렬
    Eigen::Vector3d T = t_w_curr; // 변환 벡터

    Eigen::Matrix4d T_s2s_temp; // 변환 행렬 초기화
    T_s2s_temp.block(0,0,3,3) = R; // 회전 행렬 설정
    T_s2s_temp.block(0,3,3,1) = T; // 변환 벡터 설정
    T_s2s_temp.block(3,0,1,4) << 0, 0, 0, 1; // 행렬 끝 부분 설정

    return T_s2s_temp; // 변환 행렬 반환
}

void feature_tracker::extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_points, std::vector<cv::Point3f> &cur_points, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d){
    prev_points.resize(matches.size()); // 이전 프레임에서 일치하는 포인트의 수만큼 이전 포인트 벡터 크기 설정
    cur_points.resize(matches.size()); // 현재 프레임에서 일치하는 포인트의 수만큼 현재 포인트 벡터 크기 설정

    #pragma omp parallel for num_threads(NUM_THREADS) // 병렬 처리를 위한 pragma 지시어, NUM_THREADS 개수의 스레드를 사용
    for(size_t i=0; i<matches.size(); i++){ // 모든 일치하는 포인트에 대해 반복
        int prev_point_index = matches[i].trainIdx; // 이전 프레임에서 일치한 포인트의 인덱스
        int cur_point_index  = matches[i].queryIdx; // 현재 프레임에서 일치한 포인트의 인덱스
        prev_points[i] = prev_out_point3d_[prev_point_index]; // 이전 포인트 벡터에 해당 인덱스의 이전 포인트 저장
        cur_points[i] = cur_out_point3d_[cur_point_index]; // 현재 포인트 벡터에 해당 인덱스의 현재 포인트 저장
    }
}

void feature_tracker::image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img_, cv::Mat cur_img_, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_){
    int gap =10; // 이미지 간의 간격 설정
    cv::Mat gap_image(gap, prev_img_.size().width, CV_8UC1, cv::Scalar(255,255,255)); // 간격 이미지 생성
    cv::Mat img_show;

    cv::vconcat(cur_img_, gap_image, img_show); // 현재 이미지 위에 간격 이미지를 추가하여 새 이미지 생성
    cv::vconcat(img_show, prev_img_, img_show); // 새 이미지 아래에 이전 이미지를 추가
    cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2RGB); // 이미지 컬러 포맷 변경 (그레이스케일에서 RGB로)

    // 현재 프레임의 키포인트를 그리기
    for(size_t i = 0; i< (size_t)cur_orb_point_2d_uv_.size(); i++){
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[i];
        cv::circle(img_show, cur_pt, 5, cv::Scalar(0,255,0)); // 초록색 원으로 키포인트 그리기
    }
    // 이전 프레임의 키포인트를 그리기
    for(size_t i = 0; i< (size_t)prev_orb_point_2d_uv_.size(); i++){
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[i];
        prev_pt.y += cur_img_.size().height + gap; // 이미지 간격을 고려하여 위치 조정
        cv::circle(img_show, prev_pt, 5, cv::Scalar(0,0,255)); // 빨간색 원으로 키포인트 그리기
    }

    // 매칭된 포인트들을 연결하는 선 그리기
    for(size_t i = 0; i< (size_t)matches.size(); i++){
        int cur_pt_index = matches[i].queryIdx;
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[cur_pt_index];
        int prev_pt_index = matches[i].trainIdx;
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[prev_pt_index]; 
        prev_pt.y += cur_img_.size().height + gap;

        cv::line(img_show, cur_pt, prev_pt, cv::Scalar(0,255,0), 2, 8, 0); // 일치하는 포인트들을 연결하는 선 그리기
    }

    std::string keypoint_cur_img_text("cur_img, time cost ms:");
    keypoint_cur_img_text.append(detectTime); // 감지 시간 정보 추가

    std::string match_num("Match num:");
    int match_size = (int)matches.size();
    match_num += std::to_string(match_size); // 매치된 개수 정보 추가

    cv::putText(img_show, keypoint_cur_img_text,   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2); // 이미지에 텍스트 추가

    cv::putText(img_show, match_num,   cv::Point2f(5, 60 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2); // 이미지에 매치 개수 텍스트 추가

    cv::putText(img_show, "prev_img",   cv::Point2f(5, 20 + IMAGE_HEIGHT*1 + gap), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2); // 이미지에 이전 이미지 텍스트 추가

    if(matched_keypoints_img_pub_front_end.getNumSubscribers() > 0){
        cv_bridge::CvImage output_image;
        output_image.header.frame_id = "map";
        output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        output_image.image = img_show;
        matched_keypoints_img_pub_front_end.publish(output_image); // 이미지를 ROS 토픽으로 게시
    }
}


void feature_tracker::image_show(std::vector<cv::DMatch> &matches, std::string& detectTime){
    int gap = 10; // 이미지 간의 간격 설정
    cv::Mat gap_image(gap, prev_img_.size().width, CV_8UC1, cv::Scalar(255,255,255)); // 간격 이미지 생성
    cv::Mat img_show;

    cv::vconcat(cur_img_, gap_image, img_show); // 현재 이미지 위에 간격 이미지를 추가하여 새 이미지 생성
    cv::vconcat(img_show, prev_img_, img_show); // 새 이미지 아래에 이전 이미지를 추가
    cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2RGB); // 이미지 컬러 포맷 변경 (그레이스케일에서 RGB로)

    // 현재 프레임의 키포인트를 그리기
    for(size_t i = 0; i < (size_t)cur_orb_point_2d_uv_.size(); i++){
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[i];
        cv::circle(img_show, cur_pt, 5, cv::Scalar(0,255,0)); // 초록색 원으로 키포인트 그리기
    }
    // 이전 프레임의 키포인트를 그리기
    for(size_t i = 0; i < (size_t)prev_orb_point_2d_uv_.size(); i++){
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[i];
        prev_pt.y += cur_img_.size().height + gap; // 이미지 간격을 고려하여 위치 조정
        cv::circle(img_show, prev_pt, 5, cv::Scalar(0,0,255)); // 빨간색 원으로 키포인트 그리기
    }

    // 매칭된 포인트들을 연결하는 선 그리기
    for(size_t i = 0; i < (size_t)matches.size(); i++){
        int cur_pt_index = matches[i].queryIdx;
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[cur_pt_index];
        int prev_pt_index = matches[i].trainIdx;
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[prev_pt_index]; 
        prev_pt.y += cur_img_.size().height + gap;

        cv::line(img_show, cur_pt, prev_pt, cv::Scalar(0,255,0), 2, 8, 0); // 일치하는 포인트들을 연결하는 선 그리기
    }

    std::string keypoint_cur_img_text("cur_img, time cost ms:"); // 텍스트 추가: 현재 이미지 및 감지 시간 정보
    keypoint_cur_img_text.append(detectTime);

    std::string match_num("Match num:"); // 텍스트 추가: 매치된 포인트 개수 정보
    int match_size = (int)matches.size();
    match_num += std::to_string(match_size);

    cv::putText(img_show, keypoint_cur_img_text,   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2); // 이미지에 텍스트 추가

    cv::putText(img_show, match_num,   cv::Point2f(5, 60 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2); // 이미지에 매치 개수 텍스트 추가

    cv::putText(img_show, "prev_img",   cv::Point2f(5, 20 + IMAGE_HEIGHT*1 + gap), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2); // 이미지에 이전 이미지 텍스트 추가

    if(matched_keypoints_img_pub_front_end.getNumSubscribers() > 0){
        cv_bridge::CvImage output_image;
        output_image.header.frame_id = "map";
        output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        output_image.image = img_show;
        matched_keypoints_img_pub_front_end.publish(output_image); // 이미지를 ROS 토픽으로 게시
    }
}

void feature_tracker::keypoint2uv(){
    cur_orb_point_2d_uv_.resize(cur_keypoints_.size()); // 현재 프레임의 ORB 키포인트와 동일한 크기의 2D 포인트 배열 생성
    for (size_t i = 0; i < cur_keypoints_.size(); i++)
    {
        cur_orb_point_2d_uv_[i] = cur_keypoints_[i].pt; // 현재 프레임의 ORB 키포인트를 2D 이미지 좌표로 변환하여 저장
    } 
}

std::vector<cv::Point2f> feature_tracker::keypoint2uv(std::vector<cv::KeyPoint> cur_keypoints){
    std::vector<cv::Point2f> cur_orb_point_2d_uv; // 현재 프레임의 ORB 키포인트를 저장할 2D 포인트 배열 생성
    cur_orb_point_2d_uv.resize(cur_keypoints.size()); // 배열의 크기를 현재 프레임의 ORB 키포인트 수와 동일하게 조절
    for (size_t i = 0; i < cur_keypoints.size(); i++)
    {
        cur_orb_point_2d_uv[i] = cur_keypoints[i].pt; // 현재 프레임의 ORB 키포인트를 2D 이미지 좌표로 변환하여 저장
    } 
    return cur_orb_point_2d_uv; // 변환된 2D 포인트 배열 반환
}


void feature_tracker::extractPointsAndFilterZeroValue(std::vector<cv::Point2f> cur_orb_point_2d_uv, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudTrack, std::vector<cv::Point3f> &cur_out_point3d, std::vector<uchar> &status){
    ROS_DEBUG("Extract Points and filter zero value"); // 로그 메시지 출력: "Extract Points and filter zero value"
    assert(cloudTrack->size()>0); // cloudTrack이 비어있지 않음을 확인

    cur_out_point3d.resize(cur_orb_point_2d_uv.size()); // 출력용 3D 포인트 배열의 크기를 현재 ORB 키포인트 수와 동일하게 조절
    status.resize(cur_orb_point_2d_uv.size()); // 상태 배열의 크기를 현재 ORB 키포인트 수와 동일하게 조절

    #pragma omp parallel for num_threads(NUM_THREADS) // 병렬 처리를 위한 OpenMP 지시어 사용
    for(size_t i=0; i< cur_orb_point_2d_uv.size(); i++){ // 현재 ORB 키포인트 수만큼 반복
        int col_id = cvRound(cur_orb_point_2d_uv[i].x); // 현재 ORB 키포인트의 x 좌표를 반올림하여 열 인덱스로 사용
        int row_id = cvRound(cur_orb_point_2d_uv[i].y); // 현재 ORB 키포인트의 y 좌표를 반올림하여 행 인덱스로 사용
        int index = row_id * IMAGE_WIDTH + col_id; // 이미지 픽셀 인덱스를 계산

        pcl::PointXYZI *point_i = &cloudTrack->points[index]; // cloudTrack에서 해당 인덱스의 3D 포인트를 가져옴

        cv::Point3f p_3d(0.0f, 0.0f, 0.0f); // 초기화된 3D 포인트 생성

        if(abs(point_i->x) < 0.01){ // 만약 해당 픽셀에 대해 x, y, z 값이 0.01 미만이면
            status[i] = 0; // 상태를 0으로 설정하여 해당 포인트를 필터링
        }
        else{
            status[i] = 1; // 그렇지 않으면 상태를 1로 설정하여 포인트를 유지
            p_3d.x = point_i->x; // 3D 포인트의 x 좌표 설정
            p_3d.y = point_i->y; // 3D 포인트의 y 좌표 설정
            p_3d.z = point_i->z; // 3D 포인트의 z 좌표 설정
        } 
        cur_out_point3d[i] = p_3d; // 현재 프레임의 출력용 3D 포인트 배열에 결과 저장
    } 
}

void feature_tracker::readParameters(){    
    nh_.getParam("/intensity_feature_tracker/project_name", PROJECT_NAME); // ROS 파라미터 서버에서 프로젝트 이름을 읽어와 PROJECT_NAME 변수에 저장
    nh_.getParam("/intensity_feature_tracker/cloud_topic", CLOUD_TOPIC); // ROS 파라미터 서버에서 클라우드 토픽을 읽어와 CLOUD_TOPIC 변수에 저장
    nh_.getParam("/intensity_feature_tracker/image_width", IMAGE_WIDTH); // ROS 파라미터 서버에서 이미지 너비를 읽어와 IMAGE_WIDTH 변수에 저장
    nh_.getParam("/intensity_feature_tracker/image_height", IMAGE_HEIGHT); // ROS 파라미터 서버에서 이미지 높이를 읽어와 IMAGE_HEIGHT 변수에 저장
    nh_.getParam("/intensity_feature_tracker/image_crop", IMAGE_CROP); // ROS 파라미터 서버에서 이미지 크롭 값을 읽어와 IMAGE_CROP 변수에 저장
    nh_.getParam("/intensity_feature_tracker/use_orb", USE_ORB); // ROS 파라미터 서버에서 ORB 사용 여부를 읽어와 USE_ORB 변수에 저장
    nh_.getParam("/intensity_feature_tracker/num_orb_features", NUM_ORB_FEATURES); // ROS 파라미터 서버에서 ORB 특징 수를 읽어와 NUM_ORB_FEATURES 변수에 저장
    nh_.getParam("/intensity_feature_tracker/skip_time", SKIP_TIME); // ROS 파라미터 서버에서 시간 스킵 값을 읽어와 SKIP_TIME 변수에 저장
    nh_.getParam("/intensity_feature_tracker/num_threads", NUM_THREADS); // ROS 파라미터 서버에서 스레드 수를 읽어와 NUM_THREADS 변수에 저장
    nh_.getParam("/intensity_feature_tracker/hand_held_flag", HAND_HELD_FLAG); // ROS 파라미터 서버에서 핸드헬드 플래그를 읽어와 HAND_HELD_FLAG 변수에 저장
    nh_.getParam("/intensity_feature_tracker/use_teaser", USE_TEASER); // ROS 파라미터 서버에서 TEASER 사용 여부를 읽어와 USE_TEASER 변수에 저장
    nh_.getParam("/intensity_feature_tracker/use_pnpransac", USE_PNPRANSAC); // ROS 파라미터 서버에서 PNPRANSAC 사용 여부를 읽어와 USE_PNPRANSAC 변수에 저장
    nh_.param<bool>("/intensity_feature_tracker/use_icp", USE_ICP, true); // ROS 파라미터 서버에서 ICP 사용 여부를 읽어와 USE_ICP 변수에 저장 (기본값은 true)
    nh_.param<bool>("/loop_closure_parameters/use_crop", USE_CROP, true); // ROS 파라미터 서버에서 크롭 사용 여부를 읽어와 USE_CROP 변수에 저장 (기본값은 true)
    nh_.param<bool>("/loop_closure_parameters/use_voxel_downsample", USE_DOWNSAMPLE, true); // ROS 파라미터 서버에서 복셀 다운샘플링 사용 여부를 읽어와 USE_DOWNSAMPLE 변수에 저장 (기본값은 true)
    nh_.param<double>("/loop_closure_parameters/crop_size", CROP_SIZE, 1.0); // ROS 파라미터 서버에서 크롭 크기를 읽어와 CROP_SIZE 변수에 저장 (기본값은 1.0)
    nh_.param<double>("/loop_closure_parameters/vf_scan_res", VOXEL_SIZE, 0.25); // ROS 파라미터 서버에서 복셀 크기를 읽어와 VOXEL_SIZE 변수에 저장 (기본값은 0.25)
    nh_.param<double>("/loop_closure_parameters/icp_fitness_score", FITNESS_SCORE, 0.3); // ROS 파라미터 서버에서 ICP 피트니스 스코어를 읽어와 FITNESS_SCORE 변수에 저장 (기본값은 0.3)
    nh_.param<double>("/loop_closure_parameters/keyframe_time_intervals", KEYFRAME_TIME_INTERVAL, 15); // ROS 파라미터 서버에서 키프레임 시간 간격을 읽어와 KEYFRAME_TIME_INTERVAL 변수에 저장 (기본값은 15)
    nh_.param<double>("/loop_closure_parameters/keyframe_distance_intervals", KEYFRAME_DISTANCE_INTERVAL, 0.3); // ROS 파라미터 서버에서 키프레임 거리 간격을 읽어와 KEYFRAME_DISTANCE_INTERVAL 변수에 저장 (기본값은 0.3)

    if(HAND_HELD_FLAG) setMask(); // HAND_HELD_FLAG가 true인 경우, 마스크 설정 함수(setMask) 호출
}

void feature_tracker::setMask(){
    MASK = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(255)); // MASK 변수를 8-bit 단일 채널 이미지로 초기화하고 모든 픽셀을 255로 설정

    for (int i = 0; i < IMAGE_HEIGHT; ++i) { // 이미지 높이만큼 반복
        for (int j = 0; j < IMAGE_WIDTH; ++j) { // 이미지 너비만큼 반복
            if (j < IMAGE_CROP || j > IMAGE_WIDTH - IMAGE_CROP) { // 이미지 크롭 영역 바깥의 픽셀은
                MASK.at<uchar>(i,j) = 0; // 0으로 설정하여 마스크에서 배제
            }
        }
    }
}





