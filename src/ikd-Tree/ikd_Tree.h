#pragma once // 이 헤더 파일이 한 번만 포함되도록 하는 프리프로세서 지시문

#include <stdio.h> // 표준 입출력 라이브러리
#include <queue> // 큐 자료 구조 헤더 파일
#include <pthread.h> // POSIX 스레드 관련 헤더 파일
#include <chrono> // 시간 측정을 위한 헤더 파일
#include <time.h> // 시간과 관련된 헤더 파일
#include <unistd.h> // 유닉스 시스템 관련 함수 헤더 파일
#include <math.h> // 수학 함수 헤더 파일
#include <algorithm> // 알고리즘 관련 헤더 파일
#include <memory> // 스마트 포인터와 관련된 헤더 파일
#include <pcl/point_types.h> // PCL(Point Cloud Library) 포인트 타입 헤더 파일

#define EPSS 1e-6 // 에러 허용치 정의
#define Minimal_Unbalanced_Tree_Size 10 // 최소 불균형 트리 크기 정의
#define Multi_Thread_Rebuild_Point_Num 1500 // 멀티 스레드 재구성 포인트 수 정의
#define DOWNSAMPLE_SWITCH true // 다운샘플링 스위치(true 또는 false)
#define ForceRebuildPercentage 0.2 // 강제 재구성 비율 정의
#define Q_LEN 1000000 // 큐 길이 정의

using namespace std; // 표준 네임스페이스 사용

// 3차원 점 데이터를 나타내는 구조체
struct ikdTree_PointType
{
    float x, y, z; // x, y, z 좌표를 저장하는 변수
    ikdTree_PointType(float px = 0.0f, float py = 0.0f, float pz = 0.0f)
    {
        x = px; // x 좌표 초기화
        y = py; // y 좌표 초기화
        z = pz; // z 좌표 초기화
    }
};

// 상자의 최소 및 최대 꼭지점을 나타내는 구조체
struct BoxPointType
{
    float vertex_min[3]; // 3차원 공간에서 최소 꼭지점의 x, y, z 좌표를 저장하는 배열
    float vertex_max[3]; // 3차원 공간에서 최대 꼭지점의 x, y, z 좌표를 저장하는 배열
};

// 작업 유형을 나타내는 열거형
enum operation_set {ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE, PUSH_DOWN};

// 포인트 삭제 및 저장 설정을 나타내는 열거형
enum delete_point_storage_set {NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC};

// 템플릿을 사용하여 정의된 MANUAL_Q 클래스
template <typename T>
class MANUAL_Q {
private:
    int head = 0, tail = 0, counter = 0; // 큐 관련 변수 초기화
    T q[Q_LEN]; // 큐의 요소를 저장하는 배열
    bool is_empty; // 큐가 비어있는지 여부를 나타내는 변수
public:
    void pop(); // 큐의 맨 앞 요소 제거
    T front(); // 큐의 맨 앞 요소 반환
    T back(); // 큐의 맨 뒤 요소 반환
    void clear(); // 큐 비우기
    void push(T op); // 큐에 요소 추가
    bool empty(); // 큐가 비어있는지 확인
    int size(); // 큐의 크기 반환
};

template<typename PointType>
class KD_TREE {
public:
    // PointVector 타입 정의
    using PointVector = vector<PointType, Eigen::aligned_allocator<PointType>>;
    // KD_TREE 포인터 타입 정의
    using Ptr = shared_ptr<KD_TREE<PointType>>;

    // KD 트리 노드 구조체 정의
    struct KD_TREE_NODE {
        PointType point; // 현재 노드의 포인트
        uint8_t division_axis; // 분할 축
        int TreeSize = 1; // 현재 노드를 포함하는 서브트리의 크기
        int invalid_point_num = 0; // 유효하지 않은 포인트 수
        int down_del_num = 0; // 다운샘플 및 삭제된 포인트 수
        bool point_deleted = false; // 포인트 삭제 여부 플래그
        bool tree_deleted = false; // 서브트리 삭제 여부 플래그
        bool point_downsample_deleted = false; // 다운샘플된 포인트 삭제 여부 플래그
        bool tree_downsample_deleted = false; // 다운샘플된 서브트리 삭제 여부 플래그
        bool need_push_down_to_left = false; // 왼쪽 자식으로 밀어내야 하는지 여부 플래그
        bool need_push_down_to_right = false; // 오른쪽 자식으로 밀어내야 하는지 여부 플래그
        bool working_flag = false; // 작업 중인지 여부 플래그
        float radius_sq; // 반경 제곱
        pthread_mutex_t push_down_mutex_lock; // 노드 밀어내기 뮤텍스 락
        float node_range_x[2], node_range_y[2], node_range_z[2]; // 노드 범위 (X, Y, Z)

        KD_TREE_NODE *left_son_ptr = nullptr; // 왼쪽 자식 포인터
        KD_TREE_NODE *right_son_ptr = nullptr; // 오른쪽 자식 포인터
        KD_TREE_NODE *father_ptr = nullptr; // 부모 노드 포인터

        // 논문 데이터 레코드용 추가 변수
        float alpha_del; // 삭제 알파 값
        float alpha_bal; // 균형 알파 값
    };

    // 작업 로깅을 위한 구조체
    struct Operation_Logger_Type {
        PointType point; // 포인트
        BoxPointType boxpoint; // 상자 포인트
        bool tree_deleted; // 트리 삭제 여부
        bool tree_downsample_deleted; // 다운샘플된 트리 삭제 여부
        operation_set op; // 작업 유형
    };
    
    // 포인트와 거리를 비교하는 구조체
    struct PointType_CMP {
        PointType point; // 포인트
        float dist = 0.0; // 거리, 기본값은 무한대(INFINITY)
    
        PointType_CMP(PointType p = PointType(), float d = INFINITY) {
            this->point = p;
            this->dist = d;
        }
    
        // 포인트 간 거리 비교 연산자 오버로딩
        bool operator < (const PointType_CMP &a) const {
            if (fabs(dist - a.dist) < 1e-10) return point.x < a.point.x;
            else return dist < a.dist;
        }
    };


class MANUAL_HEAP {
public:
    // 생성자: 최대 용량(max_capacity)을 받아서 초기화
    MANUAL_HEAP(int max_capacity = 100) {
        cap = max_capacity; // 최대 용량 설정
        heap = new PointType_CMP[max_capacity]; // 배열 할당
        heap_size = 0; // 힙 크기 초기화
    }

    // 소멸자: 동적 배열 메모리 해제
    ~MANUAL_HEAP() { delete[] heap; }

    // 힙에서 가장 큰 요소 제거
    void pop() {
        if (heap_size == 0) return; // 힙이 비어있으면 아무것도 하지 않음
        heap[0] = heap[heap_size - 1]; // 루트 노드를 마지막 노드로 대체
        heap_size--; // 힙 크기 감소
        MoveDown(0); // 루트 노드를 아래로 이동
        return;
    }

    // 힙에서 가장 큰 요소 반환
    PointType_CMP top() { return heap[0]; }

    // 힙에 요소 추가
    void push(PointType_CMP point) {
        if (heap_size >= cap) return; // 최대 용량에 도달하면 아무것도 하지 않음
        heap[heap_size] = point; // 요소 추가
        FloatUp(heap_size); // 추가한 요소를 위로 이동
        heap_size++; // 힙 크기 증가
        return;
    }

    // 힙의 현재 크기 반환
    int size() { return heap_size; }

    // 힙 비우기
    void clear() { heap_size = 0; }

private:
    int heap_size = 0; // 현재 힙의 크기
    int cap = 0; // 최대 용량
    PointType_CMP *heap; // 힙을 저장하는 배열

    // 특정 노드를 아래로 이동시키는 함수
    void MoveDown(int heap_index) {
        int l = heap_index * 2 + 1; // 왼쪽 자식 인덱스
        PointType_CMP tmp = heap[heap_index]; // 현재 노드 저장
        while (l < heap_size) {
            if (l + 1 < heap_size && heap[l] < heap[l + 1]) l++; // 더 큰 자식 선택
            if (tmp < heap[l]) {
                heap[heap_index] = heap[l]; // 자식 노드를 위로 이동
                heap_index = l; // 현재 노드 위치 업데이트
                l = heap_index * 2 + 1; // 새로운 왼쪽 자식 인덱스 계산
            } else break;
        }
        heap[heap_index] = tmp; // 최종 위치에 저장
        return;
    }

    // 특정 노드를 위로 이동시키는 함수
    void FloatUp(int heap_index) {
        int ancestor = (heap_index - 1) / 2; // 조상 노드 인덱스
        PointType_CMP tmp = heap[heap_index]; // 현재 노드 저장
        while (heap_index > 0) {
            if (heap[ancestor] < tmp) {
                heap[heap_index] = heap[ancestor]; // 조상 노드를 아래로 이동
                heap_index = ancestor; // 현재 노드 위치 업데이트
                ancestor = (heap_index - 1) / 2; // 새로운 조상 노드 인덱스 계산
            } else break;
        }
        heap[heap_index] = tmp; // 최종 위치에 저장
        return;
    }
};


private:
    // Multi-thread Tree Rebuild
    bool termination_flag = false; // 멀티스레드 트리 재구성 종료 플래그
    bool rebuild_flag = false; // 멀티스레드 트리 재구성 플래그
    pthread_t rebuild_thread; // 재구성 스레드
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex; // 다양한 뮤텍스 락
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock; // 재구성 로거 및 삭제된 포인트 뮤텍스 락
    MANUAL_Q<Operation_Logger_Type> Rebuild_Logger; // 재구성 로그를 저장하는 큐
    PointVector Rebuild_PCL_Storage; // 재구성 포인트 클라우드 저장
    KD_TREE_NODE **Rebuild_Ptr = nullptr; // 재구성 포인트를 가리키는 포인터 배열
    int search_mutex_counter = 0; // 검색 뮤텍스 카운터
    static void *multi_thread_ptr(void *arg); // 멀티스레드 포인터 함수 정의
    void multi_thread_rebuild(); // 멀티스레드 트리 재구성 함수
    void start_thread(); // 스레드 시작 함수
    void stop_thread(); // 스레드 중지 함수
    void run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation); // 작업을 실행하는 함수

    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0; // 임시로 사용되는 KD 트리 크기 및 유효한 노드 수
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0; // 임시로 사용되는 균형 및 삭제 알파 값
    float delete_criterion_param = 0.5f; // 삭제 기준 매개변수
    float balance_criterion_param = 0.7f; // 균형 기준 매개변수
    float downsample_size = 0.2f; // 다운샘플 크기
    bool Delete_Storage_Disabled = false; // 삭제 저장 비활성화 플래그
    KD_TREE_NODE *STATIC_ROOT_NODE = nullptr; // 정적 KD 트리 루트 노드 포인터
    PointVector Points_deleted; // 삭제된 포인트 저장용 벡터
    PointVector Downsample_Storage; // 다운샘플 저장용 벡터
    PointVector Multithread_Points_deleted; // 멀티스레드에서 삭제된 포인트 저장용 벡터
    void InitTreeNode(KD_TREE_NODE *root); // KD 트리 노드 초기화 함수
    void Test_Lock_States(KD_TREE_NODE *root); // 락 상태 테스트 함수
    void BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage); // KD 트리 빌드 함수
    void Rebuild(KD_TREE_NODE **root); // KD 트리 재구성 함수
    int Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample); // 범위 내 포인트 삭제 함수
    void Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild); // 포인트 삭제 함수
    void Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis); // 포인트 추가 함수
    void Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild); // 범위 내 포인트 추가 함수
    void Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist); // 이웃 검색 함수
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage); // 범위 내 포인트 검색 함수
    void Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage); // 반경 내 포인트 검색 함수
    bool Criterion_Check(KD_TREE_NODE *root); // 기준 확인 함수
    void Push_Down(KD_TREE_NODE *root); // 노드를 아래로 밀어내는 함수
    void Update(KD_TREE_NODE *root); // 업데이트 함수
    void delete_tree_nodes(KD_TREE_NODE **root); // 트리 노드 삭제 함수
    void downsample(KD_TREE_NODE **root); // 다운샘플 함수
    bool same_point(PointType a, PointType b); // 두 포인트가 같은지 확인하는 함수
    float calc_dist(PointType a, PointType b); // 두 포인트 간 거리 계산 함수
    float calc_box_dist(KD_TREE_NODE *node, PointType point); // 상자 내 포인트와의 거리 계산 함수
    static bool point_cmp_x(PointType a, PointType b); // X 좌표로 포인트 비교 함수
    static bool point_cmp_y(PointType a, PointType b); // Y 좌표로 포인트 비교 함수
    static bool point_cmp_z(PointType a, PointType b); // Z 좌표로 포인트 비교 함수


public:
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6, float box_length = 0.2); // KD 트리 생성자, 기본 매개변수 값 설정
    ~KD_TREE(); // KD 트리 소멸자
    void Set_delete_criterion_param(float delete_param); // 삭제 기준 매개변수 설정
    void Set_balance_criterion_param(float balance_param); // 균형 기준 매개변수 설정
    void set_downsample_param(float box_length); // 다운샘플 기준 매개변수 설정
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2); // KD 트리 초기화, 기본 매개변수 값 설정
    int size(); // KD 트리의 노드 수 반환
    int validnum(); // 유효한 노드 수 반환
    void root_alpha(float &alpha_bal, float &alpha_del); // 루트 노드의 균형 및 삭제 알파값 반환
    void Build(PointVector point_cloud); // KD 트리를 포인트 클라우드로 빌드
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, double max_dist = INFINITY); // 가장 가까운 이웃 검색
    void Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage); // 상자 내부의 포인트 검색
    void Radius_Search(PointType point, const float radius, PointVector &Storage); // 반경 내 포인트 검색
    int Add_Points(PointVector &PointToAdd, bool downsample_on); // 포인트 추가
    void Add_Point_Boxes(vector<BoxPointType> &BoxPoints); // 상자 포인트 추가
    void Delete_Points(PointVector &PointToDel); // 포인트 삭제
    int Delete_Point_Boxes(vector<BoxPointType> &BoxPoints); // 상자 포인트 삭제
    void flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type); // KD 트리를 평평하게 만들고 데이터 저장
    void acquire_removed_points(PointVector &removed_points); // 삭제된 포인트 수집
    BoxPointType tree_range(); // KD 트리의 범위 반환
    PointVector PCL_Storage; // 포인트 저장용 벡터
    KD_TREE_NODE *Root_Node = nullptr; // KD 트리 루트 노드 포인터 초기화
    int max_queue_size = 0; // 최대 큐 크기
};


