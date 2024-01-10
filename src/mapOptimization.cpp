// "mapOptimization.hpp" 헤더 파일을 포함합니다.
#include "mapOptimization.hpp"

// ROS 관련 헤더 파일을 포함합니다.
#include <ros/ros.h>

// 시각화 관련 메시지 헤더 파일을 포함합니다.
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Ceres 라이브러리 관련 헤더 파일을 포함합니다.
#include <ceres/ceres.h>

// "lidarFeaturePointsFunction.hpp" 헤더 파일을 포함합니다.
#include "lidarFeaturePointsFunction.hpp"



void mapOptimization::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
    // 현재 시간 측정을 위한 TicToc 객체 생성
    TicToc laserOdomHandler_time;

    // 현재 레이저 오도메트리 메시지에서 자세 정보를 추출하여 Eigen Quaternion 및 벡터로 저장
    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;
    q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
    q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
    q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
    q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
    t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
    t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
    t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

    // 월드 좌표계에서 현재 레이저 오도메트리의 자세 정보를 적용하여 새로운 자세 정보 계산
    Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
    Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

    // 매핑된 후의 오도메트리 메시지 생성 및 설정
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "high_freq_odom";
    odomAftMapped.header.stamp = laserOdometry->header.stamp;
    odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
    odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
    odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
    odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
    odomAftMapped.pose.pose.position.x = t_w_curr.x();
    odomAftMapped.pose.pose.position.y = t_w_curr.y();
    odomAftMapped.pose.pose.position.z = t_w_curr.z();

    // 매핑된 후의 오도메트리 메시지를 발행
    pubOdomAftMappedHighFrec.publish(odomAftMapped);

	
	// 로봇 모델을 시각화하기 위한 메시지 생성 및 설정
	visualization_msgs::Marker mesh_marker;
	mesh_marker.header.frame_id = "map"; // 메시지의 좌표 프레임 설정
	mesh_marker.header.stamp = laserOdometry->header.stamp;
	mesh_marker.ns = "spot"; // 메시지 네임스페이스 설정
	mesh_marker.id = 0; // 메시지 고유 ID 설정
	mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE; // 메시지 타입 설정 (메시 리소스)
	mesh_marker.action = visualization_msgs::Marker::ADD; // 메시지 액션 설정 (추가)
	mesh_marker.pose.position.x = t_w_curr.x(); // 로봇 위치(x) 설정
	mesh_marker.pose.position.y = t_w_curr.y(); // 로봇 위치(y) 설정
	mesh_marker.pose.position.z = t_w_curr.z(); // 로봇 위치(z) 설정
	mesh_marker.pose.orientation.x = q_w_curr.x(); // 로봇 자세(x) 설정
	mesh_marker.pose.orientation.y = q_w_curr.y(); // 로봇 자세(y) 설정
	mesh_marker.pose.orientation.z = q_w_curr.z(); // 로봇 자세(z) 설정
	mesh_marker.pose.orientation.w = q_w_curr.w(); // 로봇 자세(w) 설정
	mesh_marker.scale.x = 1; // 메시 스케일(x) 설정
	mesh_marker.scale.y = 1; // 메시 스케일(y) 설정
	mesh_marker.scale.z = 1; // 메시 스케일(z) 설정
	mesh_marker.color.r = 1; // 메시 색상(R) 설정
	mesh_marker.color.g = 1; // 메시 색상(G) 설정
	mesh_marker.color.b = 1; // 메시 색상(B) 설정
	mesh_marker.color.a = 1.0; // 메시 투명도 설정
	mesh_marker.lifetime = ros::Duration(0.5); // 메시 수명 설정 (0.5초)
	
	// 로봇 모델의 3D 메시 리소스 경로 설정
	std::string PROJECT_NAME("intensity_feature_tracker");
	std::string pkg_path = ros::package::getPath(PROJECT_NAME);
	std::string spot_mesh_file("/config/spot.stl");
	std::string prefix_str("package://");
	std::string mesh_path_str = prefix_str + PROJECT_NAME + spot_mesh_file;
	mesh_marker.mesh_resource = mesh_path_str;
	mesh_marker.mesh_use_embedded_materials = true;
	
	// 로봇 모델 메시지를 발행
	robot_marker_pub.publish(mesh_marker);
}

void mapOptimization::filterNaNPoints()
{
    
}


void mapOptimization::mapOptimizationCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& plane_cloud_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_corner_msg) {
    TicToc map_optimization_time; // 맵 최적화 실행 시간 측정용 타이머

    // 현재 로봇의 자세 정보를 가져와 변수에 저장
    q_wodom_curr.x() = odom_msg->pose.pose.orientation.x;
    q_wodom_curr.y() = odom_msg->pose.pose.orientation.y;
    q_wodom_curr.z() = odom_msg->pose.pose.orientation.z;
    q_wodom_curr.w() = odom_msg->pose.pose.orientation.w;
    t_wodom_curr.x() = odom_msg->pose.pose.position.x;
    t_wodom_curr.y() = odom_msg->pose.pose.position.y;
    t_wodom_curr.z() = odom_msg->pose.pose.position.z;

    // 로봇의 현재 자세를 맵에 대한 자세로 변환
    transformAssociateToMap();

    // 현재 로봇의 자세 정보를 변수에 매핑
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4);

    // 현재 로봇의 원본 자세 정보를 변수에 저장
    Eigen::Quaterniond q_w_curr_original(q_w_curr);
    Eigen::Vector3d t_w_curr_original(t_w_curr);

    TicToc tic_toc_pc_processing; // 포인트 클라우드 처리 시간 측정용 타이머

    // 포인트 클라우드를 저장할 변수 선언 및 초기화
    pcl::PointCloud<GroundPlanePointType> pc_plane;
    pcl::PointCloud<GroundPlanePointType> pc_corner;

    // 이미지 핸들러에서 GroundPointOut의 포인트를 초기화
    image_handler_->GroundPointOut->points.clear();



#pragma omp parallel sections num_threads(4) //~20ms
{
    #pragma omp section
    image_handler_->groundPlaneExtraction(cloud_msg); // 지면 포인트 추출 작업을 병렬로 실행

    #pragma omp section
    pcl::fromROSMsg(*plane_cloud_msg, pc_plane); // 평면 포인트 클라우드 데이터를 읽어와 변수에 저장

    #pragma omp section
    pcl::fromROSMsg(*pc_corner_msg, pc_corner); // 모서리 포인트 클라우드 데이터를 읽어와 변수에 저장

    #pragma omp section
    image_handler_->cloud_handler(cloud_msg); //~5ms 소요되는 인텐시티 이미지 처리 작업을 병렬로 실행
}

	*image_handler_->GroundPointOut += pc_plane; // 지면 포인트 클라우드에 평면 포인트 클라우드를 추가합니다.
	
	std::vector<int> idx;
	pcl::removeNaNFromPointCloud(*image_handler_->GroundPointOut, *image_handler_->GroundPointOut, idx); // 지면 포인트 클라우드에서 NaN 값을 제거합니다.
	pcl::removeNaNFromPointCloud(pc_corner, pc_corner, idx); // 모서리 포인트 클라우드에서 NaN 값을 제거합니다.
	
	cv::Mat image_intensity = image_handler_->image_intensity; // 인텐시티 이미지를 변수에 저장합니다.
	cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES*2, 1.2f, 8, 1); // ORB 특징 검출기를 생성합니다.
	
	detector->detect(image_intensity, cur_keypoints_, MASK); // ORB 특징을 이미지에서 검출합니다.
	keypoint2uv(cur_keypoints_, cur_orb_point_2d_uv_); // ORB 특징점을 2D 이미지 좌표로 변환합니다.
	
	extractPointsAndFilterZeroValue(cur_orb_point_2d_uv_, image_handler_->cloud_track, cur_out_point3d_, status_); // 특징점 위치 추출 및 0 값을 필터링합니다.
	reduceVector(cur_out_point3d_, status_); // 벡터에서 필터링된 항목을 제거합니다.
	reduceVector(cur_orb_point_2d_uv_, status_); // 벡터에서 필터링된 항목을 제거합니다.
	reduceVector(cur_keypoints_, status_); // 벡터에서 필터링된 항목을 제거합니다.
	
	detector->compute(image_intensity, cur_keypoints_, cur_descriptors_); // ORB 디스크립터를 계산합니다.
	
	// 새로운 슬라이딩 윈도우 키프레임을 생성하고 현재 데이터로 초기화합니다.
	std::shared_ptr<mapProcessor::SlideWindowKeyframe> mapKeyframeNew(new mapProcessor::SlideWindowKeyframe(
	    cur_descriptors_, image_intensity, cur_orb_point_2d_uv_, q_w_curr, t_w_curr, cur_out_point3d_));

	// 이전 키프레임이 비어 있으면 초기화합니다.
	std::vector<cv::DMatch> matches, good_matches;
	if (prev_keyframe_img.empty()) {
	    prev_keyframe_img = image_intensity; // 이전 키프레임 이미지를 현재 이미지로 설정합니다.
	    prev_keypoints_ = cur_keypoints_; // 이전 키프레임의 키포인트를 현재 키포인트로 설정합니다.
	    prev_descriptors_ = cur_descriptors_; // 이전 키프레임의 디스크립터를 현재 디스크립터로 설정합니다.
	    prev_orb_point_2d_uv_ = cur_orb_point_2d_uv_; // 이전 키프레임의 2D 특징점을 현재 값으로 설정합니다.
	    prev_out_point3d_ = cur_out_point3d_; // 이전 키프레임의 3D 포인트를 현재 값으로 설정합니다.
	    keyframeId_ = 0; // 키프레임 ID를 0으로 초기화합니다.
	
	    // 이전 키프레임을 생성하고 현재 데이터로 초기화합니다.
	    std::shared_ptr<mapProcessor::Keyframe> keyframetmp(new mapProcessor::Keyframe(keyframeId_, *image_handler_->cloud_track, *image_handler_->GroundPointOut, q_w_curr, t_w_curr));
	    prev_keyframe = *keyframetmp;
	
	    // 지면 포인트 클라우드를 현재 위치로 변환하고 k-d 트리를 빌드합니다.
	    ground_plane_cloud_ = *image_handler_->GroundPointOut;
	    Eigen::Matrix4d cur_transform = Eigen::Matrix4d::Identity();
	    cur_transform.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();
	    cur_transform.block<3, 1>(0, 3) = t_w_curr;
	    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	    pcl::transformPointCloud(ground_plane_cloud_, *tmp_cloud, cur_transform);
	    ikdtree->Build((*tmp_cloud).points);
	    tmp_cloud->clear();
	    pcl::transformPointCloud(pc_corner, *tmp_cloud, cur_transform);
	    corner_ikdtree_->Build((*tmp_cloud).points);
	}

	  else {
	    matcher_.match(cur_descriptors_, prev_descriptors_, matches); // 디스크립터를 매칭합니다.
	    std::sort(matches.begin(), matches.end()); // 매칭 결과를 정렬합니다.
	    for (size_t i = 0; i < matches.size() * 0.2; ++i) {
	        good_matches.push_back(matches[i]); // 상위 20%의 좋은 매칭을 선택합니다.
	    }
	
	    if (1) {
	        keyframe_flag_ = true; // 키프레임 플래그를 설정하여 새로운 키프레임을 생성합니다.
	        keyframeId_++; // 키프레임 ID를 증가시킵니다.
	
	        // 매칭된 포인트를 추출하고 키프레임을 초기화합니다.
	        extractMatchedPoints(good_matches, prev_matched_points3d_, prev_keyframe.q_map_cur_k_, prev_keyframe.t_map_cur_k_, cur_matched_points3d_, q_w_curr, t_w_curr, prev_out_point3d_, cur_out_point3d_, cloud_msg->header);
	    }
	
	    // 새로운 키프레임을 생성하고 현재 데이터로 초기화합니다.
	    std::shared_ptr<mapProcessor::Keyframe> keyframetmp(new mapProcessor::Keyframe(keyframeId_, *image_handler_->cloud_track, *image_handler_->GroundPointOut, q_w_curr, t_w_curr));
	    cur_keyframe = *keyframetmp;
	}

	if (keyframe_flag_ == true) {
	    keyframe_flag_ = false; // 키프레임 플래그를 비활성화합니다.
	
	    if (ground_points_pub.getNumSubscribers() != 0) {
	        // 지면 포인트 클라우드를 퍼블리시하려면 구독자가 있는지 확인합니다.
	        pcl::PointCloud<pcl::PointXYZ>::Ptr ikdtree_points(new pcl::PointCloud<pcl::PointXYZ>());
	        corner_ikdtree_->flatten(ikdtree->Root_Node, ikdtree->PCL_Storage, NOT_RECORD);
	        ikdtree_points->points = ikdtree->PCL_Storage;
	        sensor_msgs::PointCloud2 groundPlaneMapCloud_msg;
	        pcl::toROSMsg(*ikdtree_points, groundPlaneMapCloud_msg);
	        groundPlaneMapCloud_msg.header.frame_id = "map";
	        groundPlaneMapCloud_msg.header.stamp = cloud_msg->header.stamp;
	        ground_points_pub.publish(groundPlaneMapCloud_msg); // 지면 포인트 클라우드를 퍼블리시합니다.
	    }
	
	    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); // Huber 손실 함수를 사용합니다.
	    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization(); // 쿼터니언 파라미터화를 정의합니다.
	    ceres::Problem::Options problem_options;
	
	    ceres::Problem problem(problem_options);
	    problem.AddParameterBlock(parameters_, 4, q_parameterization); // 최적화 변수에 쿼터니언 파라미터 블록을 추가합니다.
	    problem.AddParameterBlock(parameters_ + 4, 3); // 최적화 변수에 변환 파라미터 블록을 추가합니다.
	
	    size_t num_good_matches = 0; // 좋은 매칭의 수를 초기화합니다.

		
        // 시각화를 위한 라인 리스트를 퍼블리시합니다.
	initialLineList(line_list, cloud_msg->header);
	
	// 이전 프레임과 현재 프레임의 키포인트 수가 다르고, 좋은 매칭이 최소 4개 이상이며, 좋은 매칭 수가 전체 매칭 수와 다른 경우
	if (prev_keypoints_.size() != cur_keypoints_.size() && good_matches.size() >= 4 && good_matches.size() != matches.size()) {
	    // 이전 키프레임과 현재 키프레임의 키포인트 수가 다르고, 좋은 매칭이 있으며, 전체 매칭 수와 다른 경우
	
	    for (size_t i = 0; i < prev_matched_points3d_.size() && false; i++) {
	        // 매칭된 포인트들에 대한 반복문
	        Eigen::Vector3d prev_point3d = prev_keyframe.q_map_cur_k_ * Eigen::Vector3d(prev_matched_points3d_[i].x, prev_matched_points3d_[i].y, prev_matched_points3d_[i].z) + prev_keyframe.t_map_cur_k_;
	        Eigen::Vector3d cur_point3d = cur_keyframe.q_map_cur_k_ * Eigen::Vector3d(cur_matched_points3d_[i].x, cur_matched_points3d_[i].y, cur_matched_points3d_[i].z) + cur_keyframe.t_map_cur_k_;
	        geometry_msgs::Point prev_point, cur_point;
	        prev_point.x = prev_point3d(0);
	        prev_point.y = prev_point3d(1);
	        prev_point.z = prev_point3d(2);
	        cur_point.x = cur_point3d(0);
	        cur_point.y = cur_point3d(1);
	        cur_point.z = cur_point3d(2);
	
	        {
	            std::lock_guard<std::mutex> lock(map_mutex_);
	            appendLines(line_list, prev_point, cur_point); // 라인 리스트에 이전 포인트에서 현재 포인트로의 선을 추가합니다.
	        }
	
	        double distance = (prev_point3d - cur_point3d).norm();
	        if (distance > 0.5) {
	            continue; // 거리가 일정 값 이상인 경우 스킵합니다.
	        }
	        num_good_matches++;
	
	        Eigen::Vector3d cur_point3d_vector3d(cur_matched_points3d_[i].x, cur_matched_points3d_[i].y, cur_matched_points3d_[i].z);
	        Eigen::Vector3d point_w;
	        point_w = q_w_curr * cur_point3d_vector3d + t_w_curr;
	
	        // Bundle Adjustment(BA)용 비용 함수를 설정합니다.
	        ceres::CostFunction* cost_function =
	            new ceres::AutoDiffCostFunction<FeatureMatchingResidual, 3, 4, 3>(
	                new FeatureMatchingResidual(cur_point3d_vector3d, prev_point3d));
	
	        {
	            std::lock_guard<std::mutex> lock(map_mutex_);
	            problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4); // 최적화 문제에 비용 함수를 추가합니다.
	        }
	    }
	}

		if (matched_lines_pub.getNumSubscribers() != 0) {
    // 라인 리스트를 구독 중인 경우, 라인 리스트를 퍼블리시합니다.
    matched_lines_pub.publish(line_list);
}

if (keyframe_sliding_window_.size() >= 1 && true) {
    // 슬라이딩 윈도우에 현재 키프레임이 존재하고, 조건을 만족하는 경우

    auto cur_keyframe_from_SW = mapKeyframeNew;

    // 슬라이딩 윈도우의 키프레임들에 대한 반복문
    #pragma omp parallel for num_threads(NUM_THREADS)
    for (auto it = keyframe_sliding_window_.rbegin(); it != keyframe_sliding_window_.rend(); ++it) {
        auto prev_keyframe_from_SW = *it;

        cv::BFMatcher matcher_tmp;
        std::vector<cv::DMatch> matches_tmp, good_matches_tmp;
        matcher_tmp.match(cur_keyframe_from_SW->descriptors, prev_keyframe_from_SW->descriptors, matches_tmp);

        if (matches_tmp.size() > 100) {
            // 매칭된 포인트 수가 100개 이상인 경우

            std::sort(matches_tmp.begin(), matches_tmp.end());

            for (size_t i = 0; i < matches_tmp.size() * 0.2; ++i) {
                good_matches_tmp.push_back(matches_tmp[i]);
            }

            {
                std::string detectTime = std::to_string(tic_toc_pc_processing.toc());

                // 이미지 시각화 함수 호출
                image_show(good_matches_tmp, detectTime, prev_keyframe_from_SW->image_intensity, cur_keyframe_from_SW->image_intensity, cur_keyframe_from_SW->orb_point_2d_uv, prev_keyframe_from_SW->orb_point_2d_uv);
            }

            std::vector<cv::Point3f> prev_matched_points3d_tmp, cur_matched_points3d_tmp;

            if (good_matches_tmp.size() > 50 && good_matches_tmp.size() != matches_tmp.size() && prev_keyframe_from_SW->descriptors.size() != cur_keyframe_from_SW->descriptors.size()) {
                // 좋은 매칭 수가 50개 이상이고, 좋은 매칭 수가 전체 매칭 수와 다르며, 이전과 현재 키프레임의 디스크립터 수가 다른 경우

                // 매칭된 포인트 추출 함수 호출
                extractMatchedPoints(good_matches_tmp, prev_matched_points3d_tmp, cur_matched_points3d_tmp, prev_keyframe_from_SW->cur_point3d_wrt_orb_features, cur_keyframe_from_SW->cur_point3d_wrt_orb_features);

                if (prev_matched_points3d_tmp.size() > 0 && cur_matched_points3d_tmp.size() > 0) {
                    // 매칭된 포인트가 있는 경우

                    // 매칭된 포인트에 대한 반복문
                    #pragma omp parallel for num_threads(NUM_THREADS)
                    for (size_t i = 0; i < prev_matched_points3d_tmp.size(); i++) {
                        Eigen::Vector3d prev_point3d = prev_keyframe_from_SW->q_map_cur_tk * Eigen::Vector3d(prev_matched_points3d_tmp[i].x, prev_matched_points3d_tmp[i].y, prev_matched_points3d_tmp[i].z) + prev_keyframe_from_SW->t_map_cur_tk;
                        Eigen::Vector3d cur_point3d = cur_keyframe_from_SW->q_map_cur_tk * Eigen::Vector3d(cur_matched_points3d_tmp[i].x, cur_matched_points3d_tmp[i].y, cur_matched_points3d_tmp[i].z) + cur_keyframe_from_SW->t_map_cur_tk;

                        double distance = (prev_point3d - cur_point3d).norm();

                        if (distance > 0.3) {
                            continue; // 거리가 일정 값 이상인 경우 스킵합니다.
                        }

                        Eigen::Vector3d cur_point3d_vector3d = Eigen::Vector3d(cur_matched_points3d_tmp[i].x, cur_matched_points3d_tmp[i].y, cur_matched_points3d_tmp[i].z);

                        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FeatureMatchingResidual, 3, 4, 3>(new FeatureMatchingResidual(cur_point3d_vector3d, prev_point3d));

                        {
                            std::lock_guard<std::mutex> lock(map_mutex_);
                            problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4); // 최적화 문제에 비용 함수를 추가합니다.
                        }
                    }
                }
            }
        }
    }
}

	
	if (true) {
    // 다운샘플링을 수행하는 블록으로, 항상 실행됩니다.

    // pcl::VoxelGrid 객체를 사용하여 다운샘플링을 수행합니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneCloudDS(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_grid_.setInputCloud(image_handler_->GroundPointOut); // 다운샘플링을 적용할 입력 포인트 클라우드를 설정합니다.
    voxel_grid_.filter(*image_handler_->GroundPointOut); // 다운샘플링된 결과를 출력 포인트 클라우드에 저장합니다.

        if (true) {
    // 이 블록은 항상 실행됩니다.

    // 입력 포인트 클라우드에 대한 다운샘플링을 이미 수행했습니다.

    for (size_t i = 0; i < image_handler_->GroundPointOut->points.size(); i++) {
        // 클라우드에서 각 지면 포인트를 가져옵니다.
        ground_point_sensor_ = image_handler_->GroundPointOut->points[i];

        // 현재 지면 포인트의 좌표를 Eigen 벡터로 변환합니다.
        Eigen::Vector3d point_curr(ground_point_sensor_.x, ground_point_sensor_.y, ground_point_sensor_.z);

        // 현재 지면 포인트를 월드 좌표계로 변환합니다.
        Eigen::Vector3d point_w_tmp = q_w_curr * point_curr + t_w_curr;
        ground_point_world_.x = point_w_tmp.x();
        ground_point_world_.y = point_w_tmp.y();
        ground_point_world_.z = point_w_tmp.z();

        // 주변 포인트를 검색합니다.
        KD_TREE<GroundPlanePointType>::PointVector searchResults;
        std::vector<float> pointSearchSqDis;
        ikdtree->Nearest_Search(ground_point_world_, 5, searchResults, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

        // 가장 가까운 5개의 포인트에 대해 정규 방향 벡터를 계산합니다.
        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = searchResults[j].x;
                matA0(j, 1) = searchResults[j].y;
                matA0(j, 2) = searchResults[j].z;
            }

            // 정규 방향 벡터를 계산합니다.
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();
            bool planeValid = true;

            // 지면 추정의 유효성을 검증합니다.
            for (int j = 0; j < 5; j++) {
                if (fabs(norm(0) * searchResults[j].x +
                            norm(1) * searchResults[j].y +
                            norm(2) * searchResults[j].z + negative_OA_dot_norm) > 0.2) {
                    planeValid = false;
                    break;
                }
            }

            // 추정된 지면이 유효한 경우, Ceres 최적화 문제에 제약 조건을 추가합니다.
            Eigen::Vector3d curr_point(ground_point_sensor_.x, ground_point_sensor_.y, ground_point_sensor_.z);
            if (planeValid) {
                ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
            }
        }
    }
}

	// Ceres Solver의 옵션을 설정합니다.
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR; // 선형 솔버 유형 설정
	options.max_num_iterations = 10; // 최대 반복 횟수 설정
	options.minimizer_progress_to_stdout = false; // 최적화 진행 상황을 콘솔에 출력하지 않도록 설정
	options.check_gradients = false; // 그래디언트 체크 비활성화
	options.gradient_check_relative_precision = 1e-4; // 그래디언트 체크 상대 정확도 설정
	
	// 최적화 문제를 해결하고 결과를 summary에 저장합니다.
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	
	// 최적화 결과를 로그로 출력합니다.
	ROS_INFO("%s", summary.BriefReport().c_str());
	
	// 최적화가 수렴한 경우, 변환을 업데이트합니다.
	if (summary.termination_type == ceres::CONVERGENCE) {
	    transformUpdate();
	}

        if (true) {
    if (summary.termination_type == ceres::CONVERGENCE) {
        // 현재 키프레임의 회전 및 위치 정보를 업데이트합니다.
        cur_keyframe.q_map_cur_k_ = q_w_curr;
        cur_keyframe.t_map_cur_k_ = t_w_curr;
    }

    // 이전 키프레임 정보를 현재 키프레임으로 업데이트합니다.
    prev_keyframe = cur_keyframe;
    prev_keyframe_img = image_intensity;
    prev_keypoints_ = cur_keypoints_;
    prev_descriptors_ = cur_descriptors_;
    prev_orb_point_2d_uv_ = cur_orb_point_2d_uv_;
    prev_out_point3d_ = cur_out_point3d_;

    // 지면 포인트 클라우드를 현재 키프레임의 변환에 맞게 변환하고, KD 트리에 추가합니다.
    pcl::PointCloud<pcl::PointXYZ> ground_plane_cloud_;
    ground_plane_cloud_ = *image_handler_->GroundPointOut;
    Eigen::Matrix4d cur_transform = Eigen::Matrix4d::Identity();
    cur_transform.block<3, 3>(0, 0) = cur_keyframe.q_map_cur_k_.toRotationMatrix();
    cur_transform.block<3, 1>(0, 3) = cur_keyframe.t_map_cur_k_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(ground_plane_cloud_, *tmp_cloud, cur_transform);
    ikdtree->Add_Points((*tmp_cloud).points, true);

    tmp_cloud->clear();
    pcl::transformPointCloud(pc_corner, *tmp_cloud, cur_transform);
    corner_ikdtree_->Add_Points((*tmp_cloud).points, true);

    if (summary.termination_type == ceres::CONVERGENCE) {
        // 맵의 새로운 키프레임의 회전 및 위치 정보를 업데이트합니다.
        mapKeyframeNew->q_map_cur_tk = q_w_curr;
        mapKeyframeNew->t_map_cur_tk = t_w_curr;
    }

    // 키프레임 슬라이딩 윈도우에 현재 키프레임을 추가합니다.
    keyframe_sliding_window_.emplace_back(mapKeyframeNew);

    // 키프레임 슬라이딩 윈도우의 크기가 SLIDING_WINDOW_SIZE를 초과하면 가장 오래된 키프레임을 제거합니다.
    if (keyframe_sliding_window_.size() > (size_t)SLIDING_WINDOW_SIZE) {
        keyframe_sliding_window_.pop_front();
    }
}

    }


    
}
	
// mapOptimization 클래스의 생성자입니다.
mapOptimization::mapOptimization():
    ikdtree(new KD_TREE<GroundPlanePointType>(0.3, 0.6, 0.4)), // ikdtree를 초기화하고 파라미터를 설정합니다.
    corner_ikdtree_(new KD_TREE<GroundPlanePointType>(0.3, 0.6, 0.8)) // corner_ikdtree를 초기화하고 파라미터를 설정합니다.
{
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_); // 현재 시점의 회전 행렬을 가져옵니다.
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4); // 현재 시점의 변환 벡터를 가져옵니다.

    // 아래 주석 처리된 코드는 q_w_curr, t_w_curr, parameters_의 정보를 출력하는 부분입니다.
    // 이 정보는 디버깅과 로그 기록을 위해 사용될 수 있습니다.
    // ROS_INFO("q_w_curr: %f, %f, %f, %f", q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w());
    // ROS_INFO("t_w_curr: %f, %f, %f", t_w_curr.x(), t_w_curr.y(), t_w_curr.z());
    // ROS_INFO("parameters_ address: %p", parameters_);
    // ROS_INFO("parameters_+4 address: %p", parameters_+4);


	// ROS 노드 핸들을 생성합니다.
	ros::NodeHandle nh;
	
	// ROS 매개변수 서버에서 이미지의 너비와 높이를 가져옵니다.
	nh.getParam("/intensity_feature_tracker/image_width", IMAGE_WIDTH);
	nh.getParam("/intensity_feature_tracker/image_height", IMAGE_HEIGHT);
	
	// 이미지 크롭 여부를 가져옵니다.
	nh.getParam("/intensity_feature_tracker/image_crop", IMAGE_CROP);
	
	// 스레드 수를 가져옵니다.
	nh.getParam("/intensity_feature_tracker/num_threads", NUM_THREADS);
	
	// ORB 기능점의 수를 가져옵니다.
	nh.getParam("/intensity_feature_tracker/num_orb_features", NUM_ORB_FEATURES);
	
	// 핸드헬드 모드 여부를 가져옵니다.
	nh.getParam("/intensity_feature_tracker/hand_held_flag", HAND_HELD_FLAG);
	
	// 슬라이딩 윈도우 크기를 가져옵니다.
	nh.getParam("/map_optimization_parameters/sliding_window_size", SLIDING_WINDOW_SIZE);
	
	// 지면 평면 윈도우 크기를 가져옵니다.
	nh.getParam("/map_optimization_parameters/ground_plane_window_size", GROUND_PLANE_WINDOW_SIZE);
	

	// ImageHandler 객체를 초기화합니다.
	image_handler_ = new intensity_slam::ImageHandler(IMAGE_HEIGHT, IMAGE_WIDTH, NUM_THREADS);
	
	// MASK를 설정합니다. HAND_HELD_FLAG 값에 따라 크롭 여부가 결정됩니다.
	if (HAND_HELD_FLAG) {
	    // 핸드헬드 모드인 경우 크롭값을 사용하여 MASK를 설정합니다.
	    MASK = setMask(IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CROP);
	} else {
	    // 핸드헬드 모드가 아닌 경우 크롭값을 사용하지 않고 MASK를 설정합니다.
	    MASK = setMask(IMAGE_HEIGHT, IMAGE_WIDTH, 0);
	}
	
	// BFMatcher를 초기화합니다. cv::NORM_HAMMING은 매칭 알고리즘의 종류를 나타냅니다.
	// true는 크로스 체크를 사용하도록 설정합니다.
	matcher_ = cv::BFMatcher(cv::NORM_HAMMING, true);
	
	// keyframe_flag_를 false로 초기화합니다.
	keyframe_flag_ = false;


	
	// 맵 초기 변환값을 설정합니다.
	q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
	t_wmap_wodom = Eigen::Vector3d(0, 0, 0);
	
	// ROS 노드 핸들을 사용하여 퍼블리셔(Publisher)를 초기화합니다.
	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100); // 고주파로 매핑된 오도메트리 메시지를 퍼블리시합니다.
	robot_marker_pub = nh.advertise<visualization_msgs::Marker>("/car_model_Marker_array", 10); // 차량 모델 마커를 퍼블리시합니다.
	matched_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/matched_points", 10); // 일치하는 포인트 클라우드를 퍼블리시합니다.
	ground_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10); // 지면 포인트 클라우드를 퍼블리시합니다.
	matched_lines_pub = nh.advertise<visualization_msgs::Marker>("/matched_lines", 10); // 일치하는 라인을 퍼블리시합니다.
	matched_keypoints_img_pub = nh.advertise<sensor_msgs::Image>("/matched_keypoints_img", 10); // 일치하는 키포인트 이미지를 퍼블리시합니다.
	
	// Voxel Grid 필터의 리프 사이즈를 설정합니다.
	voxel_grid_.setLeafSize(0.8, 0.8, 0.8);
	corner_voxel_grid_.setLeafSize(0.4, 0.4, 0.4);
}

// mapOptimization 클래스의 소멸자입니다.
mapOptimization::~mapOptimization(){}

// 이미지 크기와 마스크를 사용하여 초기 마스크를 설정하는 함수입니다.
cv::Mat mapOptimization::setMask(int height, int width, int crop){
    auto mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < height; ++i)
        for (int j = 0; j < width; ++j)
            if (j < crop || j > width - crop)
                mask.at<uchar>(i,j) = 0;
    return mask;
}

// 키포인트를 이미지 좌표로 변환하는 함수입니다.
void mapOptimization::keypoint2uv(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& uv){
    uv.clear();
    for (long unsigned int i = 0; i < keypoints.size(); ++i)
        uv.push_back(keypoints[i].pt);
}


// 키포인트와 포인트 클라우드를 사용하여 3D 포인트를 추출하고, 값이 거의 0인 포인트를 필터링하는 함수입니다.
void mapOptimization::extractPointsAndFilterZeroValue(const std::vector<cv::Point2f>& cur_orb_point_2d_uv, const pcl::PointCloud<PointType>::Ptr& cloudTrack, std::vector<cv::Point3f>& cur_out_point3d, std::vector<uchar>& status){
    cur_out_point3d.clear(); // 현재 3D 포인트 벡터 초기화
    status.clear(); // 상태 벡터 초기화
    cur_out_point3d.resize(cur_orb_point_2d_uv.size()); // 출력 3D 포인트 벡터 크기 설정
    status.resize(cur_orb_point_2d_uv.size()); // 상태 벡터 크기 설정

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i< cur_orb_point_2d_uv.size(); i++){
        int col_id = cvRound(cur_orb_point_2d_uv[i].x); // 키포인트의 열 좌표
        int row_id = cvRound(cur_orb_point_2d_uv[i].y); // 키포인트의 행 좌표
        int index = row_id * IMAGE_WIDTH + col_id; // 이미지 상의 인덱스 계산

        PointType *point_i = &cloudTrack->points[index]; // 포인트 클라우드에서 해당 인덱스의 포인트를 가져옴

        cv::Point3f p_3d(0.0f, 0.0f, 0.0f); // 초기 3D 포인트를 0으로 설정

        if(abs(point_i->x) < 0.01 && abs(point_i->y) < 0.01 && abs(point_i->z) < 0.01){
            status[i] = 0; // x, y, z 값이 거의 0이면 해당 포인트를 필터링하고 상태를 0으로 설정
        }
        else{
            status[i] = 1; // 그렇지 않으면 상태를 1로 설정하고 포인트 값을 설정
            p_3d.x = point_i->x;
            p_3d.y = point_i->y; 
            p_3d.z = point_i->z; 
        }

        cur_out_point3d[i] = p_3d; // 현재 3D 포인트 벡터에 결과를 저장
    }
}

// @in: 이전 프레임과 현재 프레임에서의 매칭된 포인트들 (matches)
// @out: 이전 프레임과 현재 프레임에서의 매칭된 포인트들의 3D 좌표 (prev_matched_points3d, cur_matched_points3d)
void mapOptimization::extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> &cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t, std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d, std_msgs::Header header){
    prev_matched_points3d.clear(); // 이전 프레임의 매칭된 3D 포인트 벡터 초기화
    cur_matched_points3d.clear(); // 현재 프레임의 매칭된 3D 포인트 벡터 초기화
    prev_matched_points3d.resize(matches.size()); // 이전 프레임의 매칭된 포인트 수에 맞게 벡터 크기 설정
    cur_matched_points3d.resize(matches.size()); // 현재 프레임의 매칭된 포인트 수에 맞게 벡터 크기 설정

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i=0; i<matches.size(); i++){
        int prev_point_index = matches[i].trainIdx; // 이전 프레임에서의 매칭된 포인트의 인덱스
        int cur_point_index  = matches[i].queryIdx; // 현재 프레임에서의 매칭된 포인트의 인덱스
        prev_matched_points3d[i] = prev_out_point3d[prev_point_index]; // 이전 프레임에서의 매칭된 포인트의 3D 좌표 저장
        cur_matched_points3d[i] = cur_out_point3d[cur_point_index]; // 현재 프레임에서의 매칭된 포인트의 3D 좌표 저장
    }

	// publish matched points
	if(prev_matched_points3d.size() > 0 && cur_matched_points3d.size() > 0){
	    // cur_matched_points3d와 prev_matched_points3d에 대한 PointcloudXYZRGB 생성
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_matched_points3d_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_matched_points3d_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    
	    // cur_matched_points3d_cloud와 prev_matched_points3d_cloud의 크기 설정
	    cur_matched_points3d_cloud->points.resize(cur_matched_points3d.size());
	    prev_matched_points3d_cloud->points.resize(prev_matched_points3d.size());
	    
	    // 현재 프레임에서의 매칭된 포인트들에 빨간색을 설정하여 PointcloudXYZRGB에 저장
	    #pragma omp parallel for num_threads(NUM_THREADS)
	    for(size_t i=0; i<cur_matched_points3d.size(); i++){
	        cur_matched_points3d_cloud->points[i].x = cur_matched_points3d[i].x;
	        cur_matched_points3d_cloud->points[i].y = cur_matched_points3d[i].y;
	        cur_matched_points3d_cloud->points[i].z = cur_matched_points3d[i].z;
	        cur_matched_points3d_cloud->points[i].r = 255; // 빨간색 설정
	        cur_matched_points3d_cloud->points[i].g = 0;
	        cur_matched_points3d_cloud->points[i].b = 0;
	    }
	
	    // 이전 프레임에서의 매칭된 포인트들에 초록색을 설정하여 PointcloudXYZRGB에 저장
	    #pragma omp parallel for num_threads(NUM_THREADS)
	    for(size_t i=0; i<prev_matched_points3d.size(); i++){
	        prev_matched_points3d_cloud->points[i].x = prev_matched_points3d[i].x;
	        prev_matched_points3d_cloud->points[i].y = prev_matched_points3d[i].y;
	        prev_matched_points3d_cloud->points[i].z = prev_matched_points3d[i].z;
	        prev_matched_points3d_cloud->points[i].r = 0;
	        prev_matched_points3d_cloud->points[i].g = 255; // 초록색 설정
	        prev_matched_points3d_cloud->points[i].b = 0;
	    }

	// cur_matched_points3d_cloud를 map에 대한 좌표계로 변환
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_matched(new pcl::PointCloud<pcl::PointXYZRGB>());
	Eigen::Matrix4d cur_transform = Eigen::Matrix4d::Identity();
	cur_transform.block<3,3>(0,0) = cur_q.toRotationMatrix();
	cur_transform.block<3,1>(0,3) = cur_t;
	pcl::transformPointCloud(*cur_matched_points3d_cloud, *cur_matched, cur_transform);
	
	// prev_matched_points3d_cloud를 map에 대한 좌표계로 변환
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_matched(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4d prev_transform = Eigen::Matrix4d::Identity();
	prev_transform.block<3,3>(0,0) = prev_q.toRotationMatrix();
	prev_transform.block<3,1>(0,3) = prev_t;
	pcl::transformPointCloud(*prev_matched_points3d_cloud, *prev_matched, prev_transform);
	
	// cur_matched_points3d_cloud와 prev_matched_points3d_cloud를 결합
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_points3d_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	*matched_points3d_cloud = *cur_matched + *prev_matched;
	
	// 매칭된 포인트들을 publish
	if(matched_points_pub.getNumSubscribers() != 0){
	    sensor_msgs::PointCloud2 matched_points3d_cloud_msg;
	    pcl::toROSMsg(*matched_points3d_cloud, matched_points3d_cloud_msg);
	    matched_points3d_cloud_msg.header = header;
	    matched_points3d_cloud_msg.header.frame_id = "map";
	    matched_points_pub.publish(matched_points3d_cloud_msg);
	}
	}
	}


	// @in: prev_out_point3d_, cur_out_point3d_
	// @out: prev_matched_points3d_, cur_matched_points3d_
	void mapOptimization::extractMatchedPoints(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &prev_matched_points3d, std::vector<cv::Point3f> &cur_matched_points3d,  std::vector<cv::Point3f> &prev_out_point3d, std::vector<cv::Point3f> &cur_out_point3d){
	    prev_matched_points3d.clear();         // 이전 프레임에서 매칭된 포인트들을 초기화
	    cur_matched_points3d.clear();          // 현재 프레임에서 매칭된 포인트들을 초기화
	    prev_matched_points3d.resize(matches.size()); // 이전 프레임에서 매칭된 포인트들의 크기 설정
	    cur_matched_points3d.resize(matches.size());  // 현재 프레임에서 매칭된 포인트들의 크기 설정
	
	    #pragma omp parallel for num_threads(NUM_THREADS) // 병렬 처리를 위한 omp 지시어
	    for(size_t i=0; i<matches.size(); i++){ // 모든 매치된 포인트들에 대해 반복
	        int prev_point_index = matches[i].trainIdx; // 이전 프레임에서의 매치 인덱스
	        int cur_point_index  = matches[i].queryIdx; // 현재 프레임에서의 매치 인덱스
	        prev_matched_points3d[i] = prev_out_point3d[prev_point_index]; // 이전 프레임에서 매치된 포인트를 저장
	        cur_matched_points3d[i] = cur_out_point3d[cur_point_index];   // 현재 프레임에서 매치된 포인트를 저장
	    }
	}

// set initial guess
void mapOptimization::transformAssociateToMap()
{
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_);  // 현재 프레임의 회전 Quaternion을 q_w_curr에 매핑
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4); // 현재 프레임의 위치 벡터를 t_w_curr에 매핑
	q_w_curr = q_wmap_wodom * q_wodom_curr; // 현재 프레임의 회전 Quaternion을 맵 좌표계로 변환
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; // 현재 프레임의 위치 벡터를 맵 좌표계로 변환
}



void mapOptimization::transformUpdate()
{
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters_); // 현재 프레임의 회전 Quaternion을 q_w_curr에 매핑
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters_ + 4); // 현재 프레임의 위치 벡터를 t_w_curr에 매핑
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse(); // 맵 좌표계에서 로봇의 회전 Quaternion을 갱신
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr; // 맵 좌표계에서 로봇의 위치 벡터를 갱신
}

void mapOptimization::calculateAverageDistance(double &avg_distance, std::vector<cv::Point3f> prev_matched_points3d, Eigen::Quaterniond prev_q, Eigen::Vector3d prev_t, std::vector<cv::Point3f> cur_matched_points3d, Eigen::Quaterniond cur_q, Eigen::Vector3d cur_t){
    double sum_distance = 0;  // 거리의 합을 저장할 변수 초기화
    double min_distance = std::numeric_limits<double>::max(); // 최소 거리를 저장할 변수 초기화
    double max_distance = std::numeric_limits<double>::min(); // 최대 거리를 저장할 변수 초기화
	
    for(size_t i=0; i<prev_matched_points3d.size(); i++){
        // 이전 및 현재 프레임에서의 대응점 좌표 계산
        Eigen::Vector3d prev_point3d = prev_q * Eigen::Vector3d(prev_matched_points3d[i].x, prev_matched_points3d[i].y, prev_matched_points3d[i].z) + prev_t;
        Eigen::Vector3d cur_point3d = cur_q * Eigen::Vector3d(cur_matched_points3d[i].x, cur_matched_points3d[i].y, cur_matched_points3d[i].z) + cur_t;
        double distance = (prev_point3d - cur_point3d).norm(); // 대응점 간 거리 계산
        sum_distance += distance; // 거리를 누적하여 합산
        if(distance < min_distance) // 현재 거리가 최소 거리보다 작으면 최소 거리 갱신
            min_distance = distance;
        if(distance > max_distance) // 현재 거리가 최대 거리보다 크면 최대 거리 갱신
            max_distance = distance;
    }
    avg_distance = sum_distance / prev_matched_points3d.size(); // 평균 거리 계산
    
    // 아래 주석 처리된 코드는 최소, 최대, 평균 거리를 ROS 메시지로 출력하는 부분입니다.
    // 이 부분을 필요에 따라 주석 해제하여 사용할 수 있습니다.
    // ROS_INFO("min_distance: %f", min_distance);
    // ROS_INFO("max_distance: %f", max_distance);
    // ROS_INFO("avg_distance: %f", avg_distance);
}

void mapOptimization::appendLines(visualization_msgs::Marker &line_list, geometry_msgs::Point p1, geometry_msgs::Point p2){
    line_list.points.push_back(p1); // 라인 리스트에 첫 번째 점 추가
    line_list.points.push_back(p2); // 라인 리스트에 두 번째 점 추가
}

void mapOptimization::initialLineList(visualization_msgs::Marker &line_list, std_msgs::Header header){
    // 라인 리스트 초기화
    line_list.points.clear(); // 기존의 점들을 지우고 새로운 점들을 추가할 준비
    line_list.header = header; // 헤더 정보 설정
    line_list.header.frame_id = "map"; // 프레임 ID 설정
    line_list.ns = "basic_line"; // 네임스페이스 설정
    line_list.id = 0; // 고유 ID 설정
    line_list.type = visualization_msgs::Marker::LINE_LIST; // 라인 리스트 유형 설정
    line_list.action = visualization_msgs::Marker::ADD; // 추가 액션 설정
    line_list.scale.x = 0.005; // 라인 두께 설정
    line_list.scale.y = 0.005; // 라인 높이 설정
    line_list.scale.z = 0.005; // 라인 깊이 설정
    line_list.color.a = 1.0; // 투명도 설정
    line_list.color.r = 0.0; // 빨간색 설정
    line_list.color.g = 0.0; // 녹색 설정
    line_list.color.b = 1.0; // 파란색 설정
    line_list.pose.orientation.w = 1.0; // 방향 설정
}

void mapOptimization::image_show(std::vector<cv::DMatch> &matches, std::string& detectTime, cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::Point2f> cur_orb_point_2d_uv_, std::vector<cv::Point2f> prev_orb_point_2d_uv_){
    int gap =10; // 이미지 사이 간격 설정

    // 간격 이미지 생성
    cv::Mat gap_image(gap, prev_img.size().width, CV_8UC1, cv::Scalar(255,255,255));
    cv::Mat img_show;

    // 현재 이미지와 이전 이미지를 수직으로 연결
    cv::vconcat(cur_img, gap_image, img_show); 
    cv::vconcat(img_show, prev_img, img_show);  
    cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2RGB); // 이미지 컬러로 변환

    // 현재 프레임에서 키포인트 그리기
    for(size_t i = 0; i < cur_orb_point_2d_uv_.size(); i++){
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[i];
        cv::circle(img_show, cur_pt, 5, cv::Scalar(0,255,0)); // 초록색 원 그리기
    }
    
    // 이전 프레임에서 키포인트 그리기
    for(size_t i = 0; i < prev_orb_point_2d_uv_.size(); i++){
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[i];
        prev_pt.y += cur_img.size().height + gap; // 위치 조정
        cv::circle(img_show, prev_pt, 5, cv::Scalar(0,0,255)); // 빨간색 원 그리기
    }

    // 매칭된 점에 대한 선 그리기
    for(size_t i = 0; i < matches.size(); i++){
        int cur_pt_index = matches[i].queryIdx;
        cv::Point2f cur_pt = cur_orb_point_2d_uv_[cur_pt_index];
        int prev_pt_index = matches[i].trainIdx;
        cv::Point2f prev_pt = prev_orb_point_2d_uv_[prev_pt_index]; 
        prev_pt.y += cur_img.size().height + gap;

        cv::line(img_show, cur_pt, prev_pt, cv::Scalar(0,255,0), 2, 8, 0); // 매칭된 선 그리기
    }

    std::string keypoint_cur_imgtext("cur_img, time cost ms:");
    keypoint_cur_imgtext.append(detectTime); // 이미지 처리 시간 추가

    std::string match_num("Match num:");
    int match_size = (int)matches.size();
    match_num += std::to_string(match_size); // 매칭된 키포인트 개수 추가

    // 이미지에 텍스트 추가
    cv::putText(img_show, keypoint_cur_imgtext,   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    cv::putText(img_show, match_num,   cv::Point2f(5, 60 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    cv::putText(img_show, "prev_img",   cv::Point2f(5, 20 + IMAGE_HEIGHT*1 + gap), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    // 이미지를 ROS 메시지로 변환하고 게시
    if(matched_keypoints_img_pub.getNumSubscribers() > 0){
        cv_bridge::CvImage output_image;
        output_image.header.frame_id = "map";
        output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        output_image.image = img_show;
        matched_keypoints_img_pub.publish(output_image);
    }
}
