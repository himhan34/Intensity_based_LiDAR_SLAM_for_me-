// 루프 클로저 핸들러 헤더 파일 포함
#include "loop_closure_handler.h"

// 루프 클로저 핸들러 클래스의 생성자
loopClosureHandler::loopClosureHandler(/* args */)
{
    // 프로젝트 이름 설정
    std::string PROJECT_NAME("intensity_feature_tracker");
    // ROS 패키지 경로 가져오기
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);
    // ORB 단어장 파일 경로 설정
    std::string vocabulary_file("/config/orbvoc.dbow3");
    vocabulary_file = pkg_path + vocabulary_file;
    // 단어장 로드
    voc_= new DBoW3::Vocabulary(vocabulary_file);
    // 데이터베이스에 단어장 설정
    db_.setVocabulary(*voc_, false, 0);
    // 루프 인덱스 초기화
    loop_index_ = -1;
    // KD-트리 초기화
    kdtreeKeyframeNearSearch_.reset(new pcl::KdTreeFLANN<PointType>());
}

// 루프 클로저 핸들러 클래스의 소멸자
loopClosureHandler::~loopClosureHandler()
{
}

// cv::Mat 이미지를 sensor_msgs::Image 메시지로 변환하는 함수
sensor_msgs::ImagePtr loopClosureHandler::cvMat2Image(std_msgs::Header header, cv::Mat & image){
    // cv_bridge를 사용하여 ROS 이미지 메시지 생성
    static cv_bridge::CvImage outImg;
    outImg.header = header;
    outImg.encoding = "bgr8";
    outImg.image = image;
    // ROS 이미지 메시지로 변환
    auto imageOut = outImg.toImageMsg();
    return imageOut;
}

// 키 프레임 처리 함수
void loopClosureHandler::keyframeProcessor(loopClosureProcessor::Keyframe keyframe, pcl::PointCloud<PointType>::Ptr cloud_keyPose3D){
    // 키 프레임 풀에 현재 키 프레임 저장
    keyframe_pool_[keyframe.keyframeId] = keyframe;
    // 루프 인덱스 초기화
    loop_index_ = -1;
    // 포인트 클라우드에 포인트가 없으면 함수 종료
    if(cloud_keyPose3D->size() == 0){
        return;
    }

    // 현재 포즈 포인트
    PointType cur_pose;
    {
        // KD-트리 검색을 위한 뮤텍스 잠금
        std::lock_guard<std::mutex> lock(kdtreeKeyframeNearSearch_mutex_);
        // KD-트리에 포인트 클라우드 설정
        kdtreeKeyframeNearSearch_->setInputCloud(cloud_keyPose3D);
        // 가장 최근 포즈 포인트 가져오기
        cur_pose = cloud_keyPose3D->points[cloud_keyPose3D->size()-1];
    }
        
    // 근접 키 프레임 검색을 위한 벡터들
    std::vector<int> keyframe_near_search_indices;
    std::vector<float> keyframe_near_search_distances;

    // 현재 포즈를 중심으로 반경 검색 수행
    kdtreeKeyframeNearSearch_->radiusSearch(cur_pose, 7, keyframe_near_search_indices, keyframe_near_search_distances);

    // 근접 키 프레임을 순회하면서 루프 클로저 확인
    for(size_t i = 0; i< keyframe_near_search_indices.size(); i++){        
        int id_tmp = keyframe_near_search_indices[i];
        auto loop_keyframe = keyframe_pool_.find(id_tmp);

        // 루프 클로저 후보가 키 프레임 풀에 존재하고, 시간 차이가 충분하면 루프 클로저로 판단
        if(loop_keyframe != keyframe_pool_.end()){
            if(abs(loop_keyframe->second.time - keyframe.time) > 40){ // 루프 클로저 시간 임계값 40초
                loop_index_ = id_tmp;
                ROS_INFO("loop index: %i, cur index: %li", loop_index_, keyframe.keyframeId);
                break; 
            }
        }
    }
}

// 키 프레임 처리 함수 정의
void loopClosureHandler::keyframeProcessor(loopClosureProcessor::Keyframe keyframe){
    // 키 프레임 풀에 현재 키 프레임 저장
    keyframe_pool_[keyframe.keyframeId] = keyframe;
    // 루프 인덱스 초기화
    loop_index_ = -1;

    // 루프 클로저 검출을 위한 방법 선택 플래그
    bool USE_SCANCONTEXT = false;
    bool USE_KDTREE = false;
    bool USE_ORBLOOP = true;

    // 스캔 컨텍스트를 사용하는 경우
    if(USE_SCANCONTEXT){
        // 스캔 컨텍스트 생성 및 저장
        scManager.makeAndSaveScancontextAndKeys(keyframe.cloud_track_);
        // 루프 클로저 검출
        auto detectResult = scManager.detectLoopClosureID();
        loop_index_ = detectResult.first;
    }

    // ORB 루프를 사용하는 경우
    if(USE_ORBLOOP){
        // ORB 기반 루프 클로저 검출
        loop_index_ = detectLoop(keyframe.image, keyframe.descriptors, keyframe.keyframeId); // 루프가 감지되지 않으면 -1, 감지되면 loop_index >=1
    }

    // KD-트리를 사용하는 경우
    if(USE_KDTREE){
        // KD-트리 기반 루프 클로저 검출 (내용 없음)
        
    }

    // 루프 인덱스가 0 이상인 경우 (루프 클로저 감지된 경우)
    if(loop_index_ >= 0){
        // 현재 ID와 후보 ID 출력
        std::cout << "\n Current id:" << keyframe.keyframeId << ", candidate id: " << loop_index_ << std::endl;
    }
}

// 루프 클로저 검출 함수 정의
int loopClosureHandler::detectLoop(cv::Mat & image, cv::Mat & descriptors, int frame_index){
    // DBoW3 쿼리 결과를 저장할 변수
    DBoW3::QueryResults ret;
    // 루프 검색에 사용할 최소 간격 및 점수 임계값 설정
    int MIN_LOOP_SEARCH_GAP = 50;
    double MIN_LOOP_BOW_TH = 0.015;
    int SKIPED_FRAMES = 5;

    // ROS 파라미터 설정
    ros::NodeHandle nh;
    nh.param<double>("/loop_closure_parameters/min_loop_bow_threshold", MIN_LOOP_BOW_TH, 0.0155);
    nh.param<int>("/loop_closure_parameters/min_loop_search_gap", MIN_LOOP_SEARCH_GAP, 50);
    nh.param<int>("/loop_closure_parameters/skiped_frames", SKIPED_FRAMES, 5);

    // DBoW3 데이터베이스 쿼리 및 추가
    db_.query(descriptors, ret, 4, frame_index - MIN_LOOP_SEARCH_GAP);
    db_.add(descriptors);

    // 이미지 풀에 현재 이미지 추가
    image_pool_[frame_index] = image.clone();
    cv::Mat bow_images = image.clone();

    // 설정된 검색 간격보다 작으면 -1 반환
    if (frame_index - MIN_LOOP_SEARCH_GAP < 0)
        return -1;

    // 루프 찾기 플래그
    bool find_loop = false;
    // 검색 결과가 임계값을 초과하는 경우 루프 찾기
    if (ret.size() >= 1 && ret[0].Score > MIN_LOOP_BOW_TH)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            ROS_INFO("ret [%i] score is %f", i, ret[i].Score);
            if (ret[i].Score > MIN_LOOP_BOW_TH)
            {          
                find_loop = true;
            }
        }
    }

    // 루프 찾은 경우 최소 인덱스 확인
    if (find_loop && frame_index > 5)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || ((int)ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        ROS_INFO("find loop: %i", min_index);
        // 찾은 인덱스가 설정된 범위 내이면 해당 인덱스 반환, 아니면 -1 반환
        if(min_index < 6){
            return min_index;
        }
        else{
            return -1;
        }
    }
    else
        return -1;
}
