#include "ikd_Tree.h"

/*
    Description: ikd-Tree: an incremental k-d tree for robotic applications
    Author: Yixi Cai
    Email: yixicai@connect.hku.hk
*/

// KD_TREE 클래스의 생성자
template <typename PointType>
KD_TREE<PointType>::KD_TREE(float delete_param, float balance_param, float box_length) {
    // 주어진 매개변수로 초기화
    delete_criterion_param = delete_param; // 삭제 기준 매개변수 설정
    balance_criterion_param = balance_param; // 균형 기준 매개변수 설정
    downsample_size = box_length; // 다운샘플 크기 설정
    Rebuild_Logger.clear(); // 재구성 로그 초기화
    termination_flag = false; // 종료 플래그 초기화
    start_thread(); // 스레드 시작
}

// KD_TREE 클래스의 소멸자
template <typename PointType>
KD_TREE<PointType>::~KD_TREE() {
    stop_thread(); // 스레드 중지
    Delete_Storage_Disabled = true; // 삭제 저장 비활성화 플래그 설정
    delete_tree_nodes(&Root_Node); // KD 트리 노드 삭제
    PointVector().swap(PCL_Storage); // 포인트 클라우드 초기화 및 메모리 해제
    Rebuild_Logger.clear(); // 재구성 로그 초기화
}

// 삭제 기준 매개변수 설정 함수
template <typename PointType>
void KD_TREE<PointType>::Set_delete_criterion_param(float delete_param) {
    delete_criterion_param = delete_param; // 주어진 값으로 삭제 기준 매개변수 설정
}

// 균형 기준 매개변수 설정 함수
template <typename PointType>
void KD_TREE<PointType>::Set_balance_criterion_param(float balance_param) {
    balance_criterion_param = balance_param; // 주어진 값으로 균형 기준 매개변수 설정
}

// 다운샘플 크기 설정 함수
template <typename PointType>
void KD_TREE<PointType>::set_downsample_param(float downsample_param) {
    downsample_size = downsample_param; // 주어진 값으로 다운샘플 크기 설정
}

// KD 트리 초기화 함수
template <typename PointType>
void KD_TREE<PointType>::InitializeKDTree(float delete_param, float balance_param, float box_length) {
    // 삭제 기준 매개변수와 균형 기준 매개변수 설정
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length); // 다운샘플 크기 설정
}

// KD 트리 노드 초기화 함수
template <typename PointType>
void KD_TREE<PointType>::InitTreeNode(KD_TREE_NODE *root) {
    // 노드 내부 변수 및 속성 초기화
    root->point.x = 0.0f; // X 좌표 초기화
    root->point.y = 0.0f; // Y 좌표 초기화
    root->point.z = 0.0f; // Z 좌표 초기화
    root->node_range_x[0] = 0.0f; // X 범위 초기화 (최소)
    root->node_range_x[1] = 0.0f; // X 범위 초기화 (최대)
    root->node_range_y[0] = 0.0f; // Y 범위 초기화 (최소)
    root->node_range_y[1] = 0.0f; // Y 범위 초기화 (최대)
    root->node_range_z[0] = 0.0f; // Z 범위 초기화 (최소)
    root->node_range_z[1] = 0.0f; // Z 범위 초기화 (최대)
    root->division_axis = 0; // 분할 축 초기화
    root->father_ptr = nullptr; // 부모 포인터 초기화
    root->left_son_ptr = nullptr; // 왼쪽 자식 포인터 초기화
    root->right_son_ptr = nullptr; // 오른쪽 자식 포인터 초기화
    root->TreeSize = 0; // 트리 크기 초기화
    root->invalid_point_num = 0; // 유효하지 않은 포인트 수 초기화
    root->down_del_num = 0; // 다운샘플 및 삭제된 포인트 수 초기화
    root->point_deleted = false; // 포인트 삭제 여부 플래그 초기화
    root->tree_deleted = false; // 서브트리 삭제 여부 플래그 초기화
    root->need_push_down_to_left = false; // 왼쪽 자식으로 밀어내야 하는지 여부 플래그 초기화
    root->need_push_down_to_right = false; // 오른쪽 자식으로 밀어내야 하는지 여부 플래그 초기화
    root->point_downsample_deleted = false; // 다운샘플된 포인트 삭제 여부 플래그 초기화
    root->working_flag = false; // 작업 중인지 여부 플래그 초기화
    pthread_mutex_init(&(root->push_down_mutex_lock), NULL); // 뮤텍스 락 초기화
}

// KD 트리의 크기를 반환하는 함수
template <typename PointType>
int KD_TREE<PointType>::size() {
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
        // 재구성 포인터가 유효하지 않거나 재구성 포인터가 루트 노드와 다를 경우
        if (Root_Node != nullptr) {
            // 루트 노드가 존재할 경우, 루트 노드의 트리 크기 반환
            return Root_Node->TreeSize;
        } else {
            // 루트 노드가 존재하지 않을 경우 크기 0 반환
            return 0;
        }
    } else {
        // 재구성 중인 경우
        if (!pthread_mutex_trylock(&working_flag_mutex)) {
            // 작업 중인 플래그를 락할 수 있는 경우
            s = Root_Node->TreeSize; // 루트 노드의 트리 크기 반환
            pthread_mutex_unlock(&working_flag_mutex); // 락 해제
            return s;
        } else {
            // 작업 중인 플래그를 락할 수 없는 경우
            return Treesize_tmp; // 임시 트리 크기 반환
        }
    }
}

// KD 트리의 범위를 반환하는 함수
template <typename PointType>
BoxPointType KD_TREE<PointType>::tree_range() {
    BoxPointType range; // 범위를 저장할 변수

    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
        // 재구성 포인터가 유효하지 않거나 재구성 포인터가 루트 노드와 다를 경우
        if (Root_Node != nullptr) {
            // 루트 노드가 존재할 경우
            // 루트 노드의 X, Y, Z 범위 정보를 범위 변수에 저장
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        } else {
            // 루트 노드가 존재하지 않을 경우, 범위를 0으로 초기화
            memset(&range, 0, sizeof(range));
        }
    } else {
        // 재구성 중인 경우
        if (!pthread_mutex_trylock(&working_flag_mutex)) {
            // 작업 중인 플래그를 락할 수 있는 경우
            // 현재 루트 노드의 X, Y, Z 범위 정보를 범위 변수에 저장
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            pthread_mutex_unlock(&working_flag_mutex); // 락 해제
        } else {
            // 작업 중인 플래그를 락할 수 없는 경우, 범위를 0으로 초기화
            memset(&range, 0, sizeof(range));
        }
    }

    return range; // 범위 변수 반환
}

// 유효한 포인트 수를 반환하는 함수
template <typename PointType>
int KD_TREE<PointType>::validnum() {
    int s = 0;

    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
        // 재구성 포인터가 유효하지 않거나 재구성 포인터가 루트 노드와 다를 경우
        if (Root_Node != nullptr) {
            // 루트 노드가 존재할 경우
            // 전체 트리 크기에서 유효하지 않은 포인트 수를 뺀 값을 반환
            return (Root_Node->TreeSize - Root_Node->invalid_point_num);
        } else {
            // 루트 노드가 존재하지 않을 경우, 유효한 포인트 수는 0
            return 0;
        }
    } else {
        // 재구성 중인 경우
        if (!pthread_mutex_trylock(&working_flag_mutex)) {
            // 작업 중인 플래그를 락할 수 있는 경우
            // 현재 루트 노드의 유효한 포인트 수를 반환
            s = Root_Node->TreeSize - Root_Node->invalid_point_num;
            pthread_mutex_unlock(&working_flag_mutex); // 락 해제
            return s;
        } else {
            // 작업 중인 플래그를 락할 수 없는 경우, -1 반환
            return -1;
        }
    }
}

// 루트 노드의 알파 값들을 반환하는 함수
template <typename PointType>
void KD_TREE<PointType>::root_alpha(float &alpha_bal, float &alpha_del) {
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
        // 재구성 포인터가 유효하지 않거나 재구성 포인터가 루트 노드와 다를 경우
        alpha_bal = Root_Node->alpha_bal; // 루트 노드의 균형 알파 값을 반환
        alpha_del = Root_Node->alpha_del; // 루트 노드의 삭제 알파 값을 반환
        return;
    } else {
        // 재구성 중인 경우
        if (!pthread_mutex_trylock(&working_flag_mutex)) {
            // 작업 중인 플래그를 락할 수 있는 경우
            alpha_bal = Root_Node->alpha_bal; // 루트 노드의 균형 알파 값을 반환
            alpha_del = Root_Node->alpha_del; // 루트 노드의 삭제 알파 값을 반환
            pthread_mutex_unlock(&working_flag_mutex); // 락 해제
            return;
        } else {
            // 작업 중인 플래그를 락할 수 없는 경우, 임시 알파 값들을 반환
            alpha_bal = alpha_bal_tmp; // 임시 균형 알파 값을 반환
            alpha_del = alpha_del_tmp; // 임시 삭제 알파 값을 반환
            return;
        }
    }
}

// 멀티 스레드를 시작하는 함수
template <typename PointType>
void KD_TREE<PointType>::start_thread() {
    // 스레드에서 사용할 뮤텍스 락 초기화
    pthread_mutex_init(&termination_flag_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_ptr_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_logger_mutex_lock, NULL);
    pthread_mutex_init(&points_deleted_rebuild_mutex_lock, NULL);
    pthread_mutex_init(&working_flag_mutex, NULL);
    pthread_mutex_init(&search_flag_mutex, NULL);

    // 멀티 스레드 생성 및 시작
    pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void*) this);

    // 멀티 스레드가 시작되었음을 출력
    printf("Multi thread started \n");
}

// 멀티 스레드를 중지하는 함수
template <typename PointType>
void KD_TREE<PointType>::stop_thread() {
    // 중지 플래그 뮤텍스 락을 얻어서 중지 플래그를 설정
    pthread_mutex_lock(&termination_flag_mutex_lock);
    termination_flag = true;
    pthread_mutex_unlock(&termination_flag_mutex_lock);

    // 멀티 스레드가 종료될 때까지 대기하고 스레드 자원 해제
    if (rebuild_thread) pthread_join(rebuild_thread, NULL);

    // 사용한 뮤텍스 락들을 해제하고 파괴
    pthread_mutex_destroy(&termination_flag_mutex_lock);
    pthread_mutex_destroy(&rebuild_logger_mutex_lock);
    pthread_mutex_destroy(&rebuild_ptr_mutex_lock);
    pthread_mutex_destroy(&points_deleted_rebuild_mutex_lock);
    pthread_mutex_destroy(&working_flag_mutex);
    pthread_mutex_destroy(&search_flag_mutex);
}

// 멀티 스레드 포인터 함수
template <typename PointType>
void * KD_TREE<PointType>::multi_thread_ptr(void *arg) {
    KD_TREE *handle = (KD_TREE*)arg; // KD_TREE 객체로 캐스팅하여 핸들 획득
    handle->multi_thread_rebuild(); // 멀티 스레드 재구성 함수 호출
    return nullptr; // 스레드 종료
}


// 멀티 스레드로 재구성 작업을 수행하는 함수
template <typename PointType>
void KD_TREE<PointType>::multi_thread_rebuild() {
    bool terminated = false; // 스레드 종료 플래그 초기화
    KD_TREE_NODE *father_ptr, **new_node_ptr; // 부모 노드와 새로운 노드 포인터

    // 스레드 종료 플래그를 조작하는데 사용되는 뮤텍스 락을 얻어서
    pthread_mutex_lock(&termination_flag_mutex_lock);
    terminated = termination_flag; // 종료 플래그 상태 저장
    pthread_mutex_unlock(&termination_flag_mutex_lock); // 뮤텍스 락 해제

    while (!terminated) { // 종료 플래그가 설정되지 않은 동안 반복
        // 재구성 포인터와 작업 중 플래그를 조작하는데 사용되는 뮤텍스 락 얻기
        pthread_mutex_lock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&working_flag_mutex);

        if (Rebuild_Ptr != nullptr) { // 재구성 포인터가 존재하는 경우
            /* Traverse and copy */
            if (!Rebuild_Logger.empty()) {
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true; // 재구성 플래그 설정

            if (*Rebuild_Ptr == Root_Node) {
                Treesize_tmp = Root_Node->TreeSize; // 임시 트리 크기 저장
                Validnum_tmp = Root_Node->TreeSize - Root_Node->invalid_point_num; // 임시 유효한 포인트 수 저장
                alpha_bal_tmp = Root_Node->alpha_bal; // 임시 균형 알파 값 저장
                alpha_del_tmp = Root_Node->alpha_del; // 임시 삭제 알파 값 저장
            }

            KD_TREE_NODE *old_root_node = (*Rebuild_Ptr); // 이전 루트 노드 포인터
            father_ptr = (*Rebuild_Ptr)->father_ptr; // 부모 노드 포인터
            PointVector().swap(Rebuild_PCL_Storage); // 재구성 포인트 저장 벡터 초기화

            // Search 작업을 위한 뮤텍스 락을 얻고, 검색 작업 중인 동안 뮤텍스가 다른 스레드에 의해 차단되는 것을 방지합니다.
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0) {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1); // 다른 스레드가 뮤텍스를 해제할 때까지 대기
                pthread_mutex_lock(&search_flag_mutex);
            }
            

            // Search 뮤텍스를 잠금
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0) {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1); // 다른 스레드가 뮤텍스를 해제할 때까지 대기
            }
            
            search_mutex_counter = -1; // Search 뮤텍스 잠금을 표시
            pthread_mutex_unlock(&search_flag_mutex); // Search 뮤텍스 해제
            
            // 삭제된 포인트 캐시를 위한 뮤텍스 락
            pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
            
            // 재구성 포인트 저장 벡터를 통해 누락된 포인트 정보를 가져옴
            flatten(*Rebuild_Ptr, Rebuild_PCL_Storage, MULTI_THREAD_REC);
            
            // 삭제된 포인트 캐시 뮤텍스 해제
            pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
            
            // Search 뮤텍스 다시 잠금
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0; // Search 뮤텍스를 다른 스레드에 허용
            pthread_mutex_unlock(&search_flag_mutex);
            
            pthread_mutex_unlock(&working_flag_mutex); // 작업 플래그 뮤텍스 해제


            /* Rebuild and update missed operations */
            // 재구성 및 누락된 작업 업데이트
            
            Operation_Logger_Type Operation; // 작업 로그를 저장할 변수
            KD_TREE_NODE *new_root_node = nullptr; // 새로운 루트 노드 포인터 초기화
            
            if (int(Rebuild_PCL_Storage.size()) > 0) { // 재구성 포인트 저장 벡터에 포인트가 존재하는 경우
                // 새로운 트리를 빌드하고 새로운 루트 노드 포인터를 얻음
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size() - 1, Rebuild_PCL_Storage);
            
                // 재구성 작업이 완료되었으므로 차단된 작업을 새 트리로 업데이트
                pthread_mutex_lock(&working_flag_mutex);
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
            
                int tmp_counter = 0;
                while (!Rebuild_Logger.empty()) { // 차단된 작업이 남아있는 동안 반복
                    Operation = Rebuild_Logger.front(); // 가장 먼저 들어온 작업을 가져옴
                    max_queue_size = max(max_queue_size, Rebuild_Logger.size()); // 최대 큐 크기 업데이트
                    Rebuild_Logger.pop(); // 큐에서 작업 제거
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock); // 빌드 작업 뮤텍스 해제
                    pthread_mutex_unlock(&working_flag_mutex); // 작업 플래그 뮤텍스 해제
            
                    run_operation(&new_root_node, Operation); // 작업을 새 트리에 적용
            
                    tmp_counter++;
                    if (tmp_counter % 10 == 0) usleep(1); // 잠시 대기하여 스레드 성능 조절
            
                    pthread_mutex_lock(&working_flag_mutex); // 작업 플래그 뮤텍스 잠금
                    pthread_mutex_lock(&rebuild_logger_mutex_lock); // 빌드 작업 뮤텍스 잠금
                }
            
                pthread_mutex_unlock(&rebuild_logger_mutex_lock); // 빌드 작업 뮤텍스 해제
            }

            /* 원래 트리로 대체하기 */
            // pthread_mutex_lock(&working_flag_mutex); - 작업 플래그 뮤텍스 락을 획득합니다. (주석처리됨)
            pthread_mutex_lock(&search_flag_mutex); // 검색 플래그 뮤텍스 락을 획득합니다.
            while (search_mutex_counter != 0){ // 검색 뮤텍스 카운터가 0이 아닌 동안 반복합니다.
                pthread_mutex_unlock(&search_flag_mutex); // 검색 플래그 뮤텍스 언락합니다.
                usleep(1); // 잠시 대기합니다.
                pthread_mutex_lock(&search_flag_mutex); // 검색 플래그 뮤텍스 락을 다시 획득합니다.
            }
            search_mutex_counter = -1; // 검색 뮤텍스 카운터를 -1로 설정합니다.
            pthread_mutex_unlock(&search_flag_mutex); // 검색 플래그 뮤텍스 언락합니다.
            
            if (father_ptr->left_son_ptr == *Rebuild_Ptr) { // 부모 노드의 왼쪽 자식 포인터가 Rebuild_Ptr와 일치하는지 확인합니다.
                father_ptr->left_son_ptr = new_root_node; // 부모 노드의 왼쪽 자식 포인터를 새로운 루트 노드로 설정합니다.
            } else if (father_ptr->right_son_ptr == *Rebuild_Ptr){ // 부모 노드의 오른쪽 자식 포인터가 Rebuild_Ptr와 일치하는지 확인합니다.
                father_ptr->right_son_ptr = new_root_node; // 부모 노드의 오른쪽 자식 포인터를 새로운 루트 노드로 설정합니다.
            } else {
                throw "Error: Father ptr incompatible with current node\n"; // 그렇지 않으면 오류를 던집니다. (부모 포인터와 현재 노드가 호환되지 않음)
            }
            
            if (new_root_node != nullptr) new_root_node->father_ptr = father_ptr; // 새로운 루트 노드가 널이 아닌 경우, 부모 포인터를 설정합니다.
            (*Rebuild_Ptr) = new_root_node; // Rebuild_Ptr를 새로운 루트 노드로 업데이트합니다.
            
            int valid_old = old_root_node->TreeSize-old_root_node->invalid_point_num; // 이전 루트 노드의 유효한 포인트 수를 계산합니다.
            int valid_new = new_root_node->TreeSize-new_root_node->invalid_point_num; // 새로운 루트 노드의 유효한 포인트 수를 계산합니다.
            
            if (father_ptr == STATIC_ROOT_NODE) Root_Node = STATIC_ROOT_NODE->left_son_ptr; // 부모 포인터가 STATIC_ROOT_NODE와 같다면, Root_Node를 업데이트합니다.
            
            KD_TREE_NODE * update_root = *Rebuild_Ptr; // 업데이트할 루트 노드를 Rebuild_Ptr로 설정합니다.
            
            while (update_root != nullptr && update_root != Root_Node){ // 업데이트 루프를 실행합니다.
                update_root = update_root->father_ptr; // 부모 노드로 이동합니다.
                if (update_root->working_flag) break; // 작업 플래그가 설정된 경우 루프를 종료합니다.
                if (update_root == update_root->father_ptr->left_son_ptr && update_root->father_ptr->need_push_down_to_left) break; // 왼쪽 자식 포인터이며 왼쪽으로 푸시 다운이 필요한 경우 루프를 종료합니다.
                if (update_root == update_root->father_ptr->right_son_ptr && update_root->father_ptr->need_push_down_to_right) break; // 오른쪽 자식 포인터이며 오른쪽으로 푸시 다운이 필요한 경우 루프를 종료합니다.
                Update(update_root); // 업데이트 함수를 호출합니다.
            }
        
        pthread_mutex_lock(&search_flag_mutex); // 검색 플래그 뮤텍스 락을 획득합니다.
        search_mutex_counter = 0; // 검색 뮤텍스 카운터를 0으로 설정합니다.
        pthread_mutex_unlock(&search_flag_mutex); // 검색 플래그 뮤텍스 언락합니다.
        
        Rebuild_Ptr = nullptr; // Rebuild_Ptr을 널 포인터로 설정합니다.
        pthread_mutex_unlock(&working_flag_mutex); // 작업 플래그 뮤텍스 언락합니다.
        rebuild_flag = false; // rebuild_flag를 거짓으로 설정합니다.
        
        /* 폐기된 트리 노드 삭제 */
        delete_tree_nodes(&old_root_node); // 폐기된 트리 노드를 삭제하는 함수를 호출합니다.
        
        } else {
            pthread_mutex_unlock(&working_flag_mutex); // 작업 플래그 뮤텍스 언락합니다.
        }
        
        pthread_mutex_unlock(&rebuild_ptr_mutex_lock); // rebuild_ptr_mutex_lock 뮤텍스 언락합니다.
        pthread_mutex_lock(&termination_flag_mutex_lock); // termination_flag_mutex_lock 뮤텍스 락을 획득합니다.
        terminated = termination_flag; // terminated를 termination_flag로 설정합니다.
        pthread_mutex_unlock(&termination_flag_mutex_lock); // termination_flag_mutex_lock 뮤텍스 언락합니다.
        
        usleep(100); // 잠시 대기합니다.
            }
        printf("Rebuild thread terminated normally\n"); // "Rebuild 스레드가 정상적으로 종료되었습니다" 메시지를 출력합니다.
            }

    
    template <typename PointType>
    void KD_TREE<PointType>::run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation) {
        switch (operation.op) {
        case ADD_POINT:
            // 포인트를 추가하는 작업입니다.
            // Add_by_point 함수를 호출하여 포인트를 추가하고, 해당 노드의 division_axis를 사용합니다.
            Add_by_point(root, operation.point, false, (*root)->division_axis);
            break;
        case ADD_BOX:
            // 박스를 추가하는 작업입니다.
            // Add_by_range 함수를 호출하여 박스를 추가합니다.
            Add_by_range(root, operation.boxpoint, false);
            break;
        case DELETE_POINT:
            // 포인트를 삭제하는 작업입니다.
            // Delete_by_point 함수를 호출하여 포인트를 삭제합니다.
            Delete_by_point(root, operation.point, false);
            break;
        case DELETE_BOX:
            // 박스를 삭제하는 작업입니다.
            // Delete_by_range 함수를 호출하여 박스를 삭제하며, tree_downsample_deleted 플래그를 고려하지 않습니다.
            Delete_by_range(root, operation.boxpoint, false, false);
            break;
        case DOWNSAMPLE_DELETE:
            // 다운샘플링 삭제 작업입니다.
            // Delete_by_range 함수를 호출하여 박스를 삭제하며, tree_downsample_deleted 플래그를 고려합니다.
            Delete_by_range(root, operation.boxpoint, false, true);
            break;
        case PUSH_DOWN:
            // PUSH_DOWN 작업입니다.
            // 노드의 플래그와 상태를 수정합니다.
            (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
            (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
            (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
            (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
            if (operation.tree_downsample_deleted) (*root)->down_del_num = (*root)->TreeSize;
            if (operation.tree_deleted) (*root)->invalid_point_num = (*root)->TreeSize;
            else (*root)->invalid_point_num = (*root)->down_del_num;
            (*root)->need_push_down_to_left = true;
            (*root)->need_push_down_to_right = true;
            break;
        default:
            break;
        }
    }

template <typename PointType>
void KD_TREE<PointType>::Build(PointVector point_cloud) {
    if (Root_Node != nullptr) {
        // 기존의 KD 트리 노드가 존재하는 경우, 해당 노드들을 삭제합니다.
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0) {
        // 포인트 클라우드가 비어있는 경우, 빌드를 중단하고 함수를 종료합니다.
        return;
    }
    STATIC_ROOT_NODE = new KD_TREE_NODE;
    // STATIC_ROOT_NODE를 새로운 KD_TREE_NODE로 할당합니다.
    InitTreeNode(STATIC_ROOT_NODE);
    // 새로운 노드를 초기화합니다.
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size() - 1, point_cloud);
    // BuildTree 함수를 호출하여 KD 트리를 빌드합니다.
    Update(STATIC_ROOT_NODE);
    // 업데이트 함수를 호출하여 트리를 업데이트합니다.
    STATIC_ROOT_NODE->TreeSize = 0;
    // STATIC_ROOT_NODE의 트리 크기를 0으로 설정합니다.
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
    // Root_Node를 STATIC_ROOT_NODE의 왼쪽 자식 포인터로 설정합니다.
}

template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(PointType point, int k_nearest, PointVector& Nearest_Points, vector<float> & Point_Distance, double max_dist) {
    MANUAL_HEAP q(2 * k_nearest);
    // MANUAL_HEAP을 초기화합니다.
    q.clear();
    // 큐를 비웁니다.
    vector<float> ().swap(Point_Distance);
    // Point_Distance 벡터를 비웁니다.

    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
        // Rebuild_Ptr가 널이거나 Root_Node가 Rebuild_Ptr가 가리키는 노드가 아닌 경우,
        // 일반적인 검색(Search)을 수행합니다.
        Search(Root_Node, k_nearest, point, q, max_dist);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        // 검색 플래그 뮤텍스 락을 획득합니다.

        while (search_mutex_counter == -1) {
            // 검색 뮤텍스 카운터가 -1인 동안 대기합니다.
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);
            pthread_mutex_lock(&search_flag_mutex);
        }

        search_mutex_counter += 1;
        // 검색 뮤텍스 카운터를 증가시킵니다.
        pthread_mutex_unlock(&search_flag_mutex);

        Search(Root_Node, k_nearest, point, q, max_dist);
        // 검색(Search)을 수행합니다.

        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        // 검색 뮤텍스 카운터를 감소시킵니다.
        pthread_mutex_unlock(&search_flag_mutex);
    }

    int k_found = min(k_nearest, int(q.size()));
    // 찾은 가장 가까운 점의 개수를 결정합니다. (k_nearest와 큐의 크기 중 작은 값)

    PointVector ().swap(Nearest_Points);
    // Nearest_Points 벡터를 비웁니다.
    vector<float> ().swap(Point_Distance);
    // Point_Distance 벡터를 비웁니다.

    for (int i = 0; i < k_found; i++) {
        // 찾은 가장 가까운 점을 Nearest_Points와 Point_Distance 벡터에 삽입합니다.
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }

    return;
}

template <typename PointType>
void KD_TREE<PointType>::Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage)
{
    Storage.clear();
    // Storage 벡터를 비웁니다.

    Search_by_range(Root_Node, Box_of_Point, Storage);
    // 주어진 박스 내에 있는 포인트를 검색하여 Storage에 저장합니다.
}

template <typename PointType>
void KD_TREE<PointType>::Radius_Search(PointType point, const float radius, PointVector &Storage)
{
    Storage.clear();
    // Storage 벡터를 비웁니다.

    Search_by_radius(Root_Node, point, radius, Storage);
    // 주어진 반경 내에 있는 포인트를 검색하여 Storage에 저장합니다.
}

template <typename PointType>
int KD_TREE<PointType>::Add_Points(PointVector & PointToAdd, bool downsample_on){
    // 포인트를 KD 트리에 추가하는 함수입니다.

    int NewPointSize = PointToAdd.size();
    // 추가할 포인트의 개수를 저장합니다.

    int tree_size = size();
    // 현재 KD 트리의 크기를 저장합니다.

    BoxPointType Box_of_Point;
    // 포인트를 포함하는 상자를 정의합니다.

    PointType downsample_result, mid_point;
    // 다운샘플링 결과와 중간 포인트를 저장할 변수를 정의합니다.

    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    // 다운샘플링 여부를 결정하는 스위치 변수를 설정합니다.

    float min_dist, tmp_dist;
    int tmp_counter = 0;

    for (int i = 0; i < PointToAdd.size(); i++){
        // 주어진 포인트들을 반복적으로 처리합니다.

        if (downsample_switch){
            // 다운샘플링 스위치가 활성화된 경우,

            // 포인트를 포함하는 상자의 경계를 계산합니다.
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0] + downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1] + downsample_size; 
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2] + downsample_size;   
            
            // 상자의 중심 포인트를 계산합니다.
            mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0] - Box_of_Point.vertex_min[0]) / 2.0;
            mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1] - Box_of_Point.vertex_min[1]) / 2.0;
            mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2] - Box_of_Point.vertex_min[2]) / 2.0;

            // 다운샘플링을 수행하기 위한 임시 벡터를 초기화합니다.
            PointVector().swap(Downsample_Storage);

            // 상자 내의 포인트를 검색하여 Downsample_Storage 벡터에 저장합니다.
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);

            // 초기 최소 거리를 설정하고 다운샘플링 결과 포인트를 초기화합니다.
            min_dist = calc_dist(PointToAdd[i], mid_point);
            downsample_result = PointToAdd[i];                

            // 상자 내의 포인트와 중간 포인트 사이의 거리를 계산하고,
            // 가장 가까운 포인트를 다운샘플링 결과로 설정합니다.
            for (int index = 0; index < Downsample_Storage.size(); index++){
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist){
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }

                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {  
            // Rebuild_Ptr가 nullptr이거나, Rebuild_Ptr가 가리키는 값이 Root_Node와 같지 않을 때
            if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)) {
                // Downsample_Storage 크기가 1보다 크거나, PointToAdd[i]와 downsample_result가 같을 때
                if (Downsample_Storage.size() > 0) Delete_by_range(&Root_Node, Box_of_Point, true, true);
                // Downsample_Storage 크기가 0보다 크면 Root_Node에서 Box_of_Point 범위 내의 노드 삭제 (true: 병합 및 재구성 수행)
                Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
                // downsample_result를 Root_Node에 추가하고, 재구성을 위해 division_axis 사용 (true: 재구성 수행)
                tmp_counter++;
                // tmp_counter 증가
            }
        } else {
            // Rebuild_Ptr가 nullptr이 아니고, Rebuild_Ptr가 가리키는 값이 Root_Node와 같을 때
            if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)) {
                // Downsample_Storage 크기가 1보다 크거나, PointToAdd[i]와 downsample_result가 같을 때
                Operation_Logger_Type  operation_delete, operation;
                // Operation_Logger_Type 타입의 변수들 operation_delete, operation 선언
                operation_delete.boxpoint = Box_of_Point;
                operation_delete.op = DOWNSAMPLE_DELETE;
                // operation_delete에 Box_of_Point 할당 및 연산 유형 설정
                operation.point = downsample_result;
                operation.op = ADD_POINT;
                // operation에 downsample_result 할당 및 연산 유형 설정
                pthread_mutex_lock(&working_flag_mutex);
                // 스레드 뮤텍스 (working_flag_mutex) 락
                if (Downsample_Storage.size() > 0) Delete_by_range(&Root_Node, Box_of_Point, false, true);
                // Downsample_Storage 크기가 0보다 크면 Root_Node에서 Box_of_Point 범위 내의 노드 삭제 (false: 병합하지 않고, true: 재구성 수행)
                Add_by_point(&Root_Node, downsample_result, false, Root_Node->division_axis);
                // downsample_result를 Root_Node에 추가하고, 병합하지 않고 division_axis 사용 (false: 병합하지 않음)
                tmp_counter++;
                // tmp_counter 증가
                if (rebuild_flag) {
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 락
                    if (Downsample_Storage.size() > 0) Rebuild_Logger.push(operation_delete);
                    // Downsample_Storage 크기가 0보다 크면 operation_delete를 Rebuild_Logger에 추가
                    Rebuild_Logger.push(operation);
                    // operation을 Rebuild_Logger에 추가
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 언락
                }
                pthread_mutex_unlock(&working_flag_mutex);
                // 스레드 뮤텍스 (working_flag_mutex) 언락
            };
        }
        else {
            // Rebuild_Ptr가 nullptr이 아니고, Rebuild_Ptr가 가리키는 값이 Root_Node와 같지 않을 때
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
                // Rebuild_Ptr가 nullptr이거나, Rebuild_Ptr가 가리키는 값이 Root_Node와 같지 않을 때
                Add_by_point(&Root_Node, PointToAdd[i], true, Root_Node->division_axis);
                // PointToAdd[i]를 Root_Node에 추가하고, 재구성을 위해 division_axis 사용 (true: 재구성 수행)
            } else {
                // 그 외의 경우
                Operation_Logger_Type operation;
                // Operation_Logger_Type 타입의 변수 operation 선언
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;                
                // operation에 PointToAdd[i] 할당 및 연산 유형 설정
                pthread_mutex_lock(&working_flag_mutex);
                // 스레드 뮤텍스 (working_flag_mutex) 락
                Add_by_point(&Root_Node, PointToAdd[i], false, Root_Node->division_axis);
                // PointToAdd[i]를 Root_Node에 추가하고, 병합하지 않고 division_axis 사용 (false: 병합하지 않음)
                if (rebuild_flag) {
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 락
                    Rebuild_Logger.push(operation);
                    // operation을 Rebuild_Logger에 추가
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 언락
                }
                pthread_mutex_unlock(&working_flag_mutex);       
                // 스레드 뮤텍스 (working_flag_mutex) 언락
            }
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Add_Point_Boxes(vector<BoxPointType> & BoxPoints){
    // BoxPoints 벡터에 있는 각 BoxPointType을 처리하기 위한 반복문
    for (int i = 0; i < BoxPoints.size(); i++) {
        // Rebuild_Ptr가 nullptr이거나, Rebuild_Ptr가 가리키는 값이 Root_Node와 같지 않을 때
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
            // BoxPoints[i]를 Root_Node에 추가하고, 병합 및 재구성을 수행 (true: 재구성 수행)
            Add_by_range(&Root_Node, BoxPoints[i], true);
        } else {
            // 그 외의 경우
            Operation_Logger_Type operation;
            // Operation_Logger_Type 타입의 변수 operation 선언
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            // operation에 BoxPoints[i] 할당 및 연산 유형 설정
            pthread_mutex_lock(&working_flag_mutex);
            // 스레드 뮤텍스 (working_flag_mutex) 락
            // BoxPoints[i]를 Root_Node에 추가하고, 병합하지 않고 division_axis 사용 (false: 병합하지 않음)
            Add_by_range(&Root_Node, BoxPoints[i], false);
            if (rebuild_flag) {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 락
                // operation을 Rebuild_Logger에 추가
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 언락
            }
            pthread_mutex_unlock(&working_flag_mutex);
            // 스레드 뮤텍스 (working_flag_mutex) 언락
        }
    }
    // 함수 종료
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_Points(PointVector & PointToDel){
    // PointToDel 벡터에 있는 각 PointType을 처리하기 위한 반복문
    for (int i = 0; i < PointToDel.size(); i++) {
        // Rebuild_Ptr가 nullptr이거나, Rebuild_Ptr가 가리키는 값이 Root_Node와 같지 않을 때
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
            // PointToDel[i]를 Root_Node에서 삭제하고, 병합 및 재구성을 수행 (true: 재구성 수행)
            Delete_by_point(&Root_Node, PointToDel[i], true);
        } else {
            // 그 외의 경우
            Operation_Logger_Type operation;
            // Operation_Logger_Type 타입의 변수 operation 선언
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;
            // operation에 PointToDel[i] 할당 및 연산 유형 설정
            pthread_mutex_lock(&working_flag_mutex);
            // 스레드 뮤텍스 (working_flag_mutex) 락
            // PointToDel[i]를 Root_Node에서 삭제하고, 병합하지 않고 division_axis 사용 (false: 병합하지 않음)
            Delete_by_point(&Root_Node, PointToDel[i], false);
            if (rebuild_flag) {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 락
                // operation을 Rebuild_Logger에 추가
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 언락
            }
            pthread_mutex_unlock(&working_flag_mutex);
            // 스레드 뮤텍스 (working_flag_mutex) 언락
        }
    }
    // 함수 종료
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_Point_Boxes(vector<BoxPointType> & BoxPoints) {
    int tmp_counter = 0;
    // 삭제된 노드의 수를 저장하기 위한 임시 변수 tmp_counter 초기화
    for (int i = 0; i < BoxPoints.size(); i++) { 
        // BoxPoints 벡터에 있는 각 BoxPointType을 처리하기 위한 반복문
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {               
            // Rebuild_Ptr가 nullptr이거나, Rebuild_Ptr가 가리키는 값이 Root_Node와 같지 않을 때
            // BoxPoints[i]를 Root_Node에서 삭제하고, 병합 및 재구성을 수행 (true: 재구성 수행, false: 병합하지 않음)
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], true, false);
        } else {
            // 그 외의 경우
            Operation_Logger_Type operation;
            // Operation_Logger_Type 타입의 변수 operation 선언
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;     
            // operation에 BoxPoints[i] 할당 및 연산 유형 설정
            pthread_mutex_lock(&working_flag_mutex); 
            // 스레드 뮤텍스 (working_flag_mutex) 락
            // BoxPoints[i]를 Root_Node에서 삭제하고, 병합하지 않고 division_axis 사용 (false: 병합하지 않음)
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], false, false);
            if (rebuild_flag) {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 락
                // operation을 Rebuild_Logger에 추가
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                // 스레드 뮤텍스 (rebuild_logger_mutex_lock) 언락
            }                
            pthread_mutex_unlock(&working_flag_mutex);
            // 스레드 뮤텍스 (working_flag_mutex) 언락
        }
    } 
    // 반복문이 끝난 후 삭제된 노드의 총 수를 반환
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::acquire_removed_points(PointVector & removed_points) {
    // points_deleted_rebuild_mutex_lock 뮤텍스를 락 (다른 스레드와의 동기화를 위함)
    pthread_mutex_lock(&points_deleted_rebuild_mutex_lock); 

    // Points_deleted 벡터에 있는 모든 요소를 removed_points 벡터에 추가
    for (int i = 0; i < Points_deleted.size(); i++) {
        removed_points.push_back(Points_deleted[i]);
    }

    // Multithread_Points_deleted 벡터에 있는 모든 요소를 removed_points 벡터에 추가
    for (int i = 0; i < Multithread_Points_deleted.size(); i++) {
        removed_points.push_back(Multithread_Points_deleted[i]);
    }

    // Points_deleted와 Multithread_Points_deleted 벡터를 비움
    Points_deleted.clear();
    Multithread_Points_deleted.clear();

    // points_deleted_rebuild_mutex_lock 뮤텍스 언락 (락을 해제하여 다른 스레드의 접근 허용)
    pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);

    // 함수 종료
    return;
}

template <typename PointType>
void KD_TREE<PointType>::BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage) {
    // 재귀적으로 KD 트리를 구축하는 함수입니다.

    if (l > r) return;
    // 범위 [l, r]가 유효하지 않으면 함수 종료

    *root = new KD_TREE_NODE;
    // KD_TREE_NODE 타입의 새로운 노드를 할당하고 root 포인터에 연결
    InitTreeNode(*root);
    // 새로운 노드를 초기화합니다.

    int mid = (l + r) >> 1;
    // 중간 인덱스를 계산합니다.
    int div_axis = 0;
    int i;

    // 최적의 분할 축을 찾습니다.
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0, 0, 0};
    // 세 개의 축 (x, y, z)에 대한 최솟값, 최댓값, 범위를 나타내는 배열

    // 범위 [l, r]에 있는 점들 중에서 x, y, z 축에 대한 최솟값과 최댓값을 계산합니다.
    for (i = l; i <= r; i++) {
        min_value[0] = min(min_value[0], Storage[i].x);
        min_value[1] = min(min_value[1], Storage[i].y);
        min_value[2] = min(min_value[2], Storage[i].z);
        max_value[0] = max(max_value[0], Storage[i].x);
        max_value[1] = max(max_value[1], Storage[i].y);
        max_value[2] = max(max_value[2], Storage[i].z);
    }

    // 가장 긴 차원을 분할 축으로 선택합니다.
    for (i = 0; i < 3; i++) dim_range[i] = max_value[i] - min_value[i];
    for (i = 1; i < 3; i++) if (dim_range[i] > dim_range[div_axis]) div_axis = i;
    // 선택된 분할 축을 노드의 division_axis에 할당합니다.
    (*root)->division_axis = div_axis;

    // 분할 축에 따라 포인트를 정렬합니다.
    switch (div_axis)
    {
    case 0:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        // x 축을 기준으로 포인트를 정렬합니다.
        break;
    case 1:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_y);
        // y 축을 기준으로 포인트를 정렬합니다.
        break;
    case 2:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_z);
        // z 축을 기준으로 포인트를 정렬합니다.
        break;
    default:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        // 기본적으로 x 축을 기준으로 포인트를 정렬합니다.
        break;
    }

    (*root)->point = Storage[mid];
    // 현재 노드에 중간 위치의 포인트를 할당합니다.

    KD_TREE_NODE * left_son = nullptr, * right_son = nullptr;
    // 왼쪽 자식과 오른쪽 자식 노드를 초기화합니다.
    BuildTree(&left_son, l, mid - 1, Storage);
    // 왼쪽 서브트리를 재귀적으로 구축합니다.
    BuildTree(&right_son, mid + 1, r, Storage);
    // 오른쪽 서브트리를 재귀적으로 구축합니다.

    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    // 현재 노드의 왼쪽과 오른쪽 자식 포인터를 설정합니다.
    
    Update((*root));
    // 현재 노드를 업데이트합니다.

    return;
}


template <typename PointType>
void KD_TREE<PointType>::Rebuild(KD_TREE_NODE ** root) {
    KD_TREE_NODE * father_ptr;
    
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num) { 
        // 만약 현재 노드의 트리 크기가 재구성 임계값을 초과하면,
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock)){     
            // rebuild_ptr_mutex_lock 뮤텍스를 시도하여 다른 스레드에서 락을 가지고 있는지 확인
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize)) {
                // Rebuild_Ptr가 nullptr이거나 현재 노드의 트리 크기가 더 크면,
                Rebuild_Ptr = root; 
                // Rebuild_Ptr를 현재 노드로 설정 (재구성 대상 노드로 지정)
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
            // rebuild_ptr_mutex_lock 뮤텍스 언락
        }
    } else {
        // 만약 현재 노드의 트리 크기가 재구성 임계값을 초과하지 않으면,
        father_ptr = (*root)->father_ptr;
        // 현재 노드의 부모 노드 포인터를 저장
        int size_rec = (*root)->TreeSize;
        // 현재 노드의 트리 크기를 저장
        PCL_Storage.clear();
        // 포인트를 저장할 벡터 PCL_Storage를 비움
        flatten(*root, PCL_Storage, DELETE_POINTS_REC);
        // 현재 노드 아래의 모든 포인트를 삭제하면서 PCL_Storage에 저장
        delete_tree_nodes(root);
        // 현재 노드 아래의 모든 노드를 삭제
        BuildTree(root, 0, PCL_Storage.size() - 1, PCL_Storage);
        // PCL_Storage에 저장된 포인트를 이용하여 새로운 KD 트리를 구축
        if (*root != nullptr) (*root)->father_ptr = father_ptr;
        // 현재 노드의 부모 노드 포인터를 재설정
        if (*root == Root_Node) STATIC_ROOT_NODE->left_son_ptr = *root;
        // 만약 현재 노드가 루트 노드라면, 정적 루트 노드의 왼쪽 자식으로 현재 노드를 설정
    } 
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample) {
    // 주어진 범위 내의 포인트를 삭제하는 함수입니다.

    if ((*root) == nullptr || (*root)->tree_deleted) return 0;
    // 현재 노드가 nullptr이거나 이미 삭제된 경우, 삭제된 포인트 수를 0으로 반환합니다.

    (*root)->working_flag = true;
    // 현재 노드의 작업 플래그를 활성화합니다.

    Push_Down(*root);
    // 현재 노드 아래로 작업을 전파합니다.

    int tmp_counter = 0;

    // 주어진 박스의 x, y, z 범위가 현재 노드의 범위를 벗어나면 삭제할 필요가 없습니다.
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return 0;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return 0;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return 0;

    // 주어진 박스가 현재 노드의 범위를 완전히 포함하는 경우
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] &&
        boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1]) {
        // 현재 노드와 그 아래의 모든 노드를 삭제합니다.
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;

        // 다운샘플링 중인 경우, 다운샘플링 관련 정보도 업데이트합니다.
        if (is_downsample) {
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }

        return tmp_counter;
    }
        // 현재 노드의 포인트가 아직 삭제되지 않았고, 주어진 박스가 해당 포인트를 포함하는 경우
    if (!(*root)->point_deleted && boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x &&
        boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y &&
        boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z) {
        // 현재 노드의 포인트를 삭제 처리합니다.
        (*root)->point_deleted = true;
        tmp_counter += 1;
        // 삭제된 포인트 수를 증가시킵니다.
        if (is_downsample) (*root)->point_downsample_deleted = true;
        // 다운샘플링 중인 경우, 관련 정보를 업데이트합니다.
    }

        // Operation_Logger_Type 타입의 삭제 로그를 생성합니다.
    Operation_Logger_Type delete_box_log;
    // 스레드 대기 시간을 설정하기 위한 timespec 구조체를 생성합니다.
    struct timespec Timeout;

    // 다운샘플링인 경우 DOWNSAMPLE_DELETE 작업을, 그렇지 않은 경우 DELETE_BOX 작업을 설정합니다.
    if (is_downsample)
        delete_box_log.op = DOWNSAMPLE_DELETE;
    else
        delete_box_log.op = DELETE_BOX;
    
    delete_box_log.boxpoint = boxpoint;

    // Rebuild_Ptr가 nullptr이거나 현재 노드의 왼쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr) {
        // 왼쪽 자식 노드로 이동하여 삭제 작업을 수행하고, tmp_counter를 업데이트합니다.
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        // Rebuild_Ptr와 현재 노드의 왼쪽 자식 노드가 같은 경우
        pthread_mutex_lock(&working_flag_mutex);
        // working_flag_mutex 뮤텍스를 락하여 동기화를 수행합니다.
        
        // 왼쪽 자식 노드로 이동하여 삭제 작업을 수행하고, tmp_counter를 업데이트합니다.
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, false, is_downsample);
        
        if (rebuild_flag) {
            // 재구성 플래그가 활성화된 경우
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            // rebuild_logger_mutex_lock 뮤텍스를 락하여 로그 작업 동기화를 수행합니다.
            Rebuild_Logger.push(delete_box_log);
            // 삭제 로그를 재구성 로그 큐에 추가합니다.
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            // rebuild_logger_mutex_lock 뮤텍스 언락
        }
        
        pthread_mutex_unlock(&working_flag_mutex);
        // working_flag_mutex 뮤텍스 언락
    }

        // Rebuild_Ptr가 nullptr이거나 현재 노드의 오른쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr) {
        // 오른쪽 자식 노드로 이동하여 삭제 작업을 수행하고, tmp_counter를 업데이트합니다.
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        // Rebuild_Ptr와 현재 노드의 오른쪽 자식 노드가 같은 경우
        pthread_mutex_lock(&working_flag_mutex);
        // working_flag_mutex 뮤텍스를 락하여 동기화를 수행합니다.
        
        // 오른쪽 자식 노드로 이동하여 삭제 작업을 수행하고, tmp_counter를 업데이트합니다.
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, false, is_downsample);
        
        if (rebuild_flag) {
            // 재구성 플래그가 활성화된 경우
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            // rebuild_logger_mutex_lock 뮤텍스를 락하여 로그 작업 동기화를 수행합니다.
            Rebuild_Logger.push(delete_box_log);
            // 삭제 로그를 재구성 로그 큐에 추가합니다.
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            // rebuild_logger_mutex_lock 뮤텍스 언락
        }
        
        pthread_mutex_unlock(&working_flag_mutex);
        // working_flag_mutex 뮤텍스 언락
    }    

    Update(*root);
    // 현재 노드를 업데이트합니다.

    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) 
        Rebuild_Ptr = nullptr;
    // 만약 Rebuild_Ptr가 현재 노드를 가리키고, 현재 노드의 트리 크기가 Multi_Thread_Rebuild_Point_Num보다 작다면, Rebuild_Ptr를 nullptr로 설정합니다.

    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    // 재구성이 필요한지 여부를 결정하기 위한 조건을 검사합니다.
    
    if (need_rebuild) Rebuild(root);
    // 재구성이 필요하다면 KD 트리를 재구성합니다.

    if ((*root) != nullptr) (*root)->working_flag = false;
    // 현재 노드의 작업 플래그를 비활성화합니다.

    return tmp_counter;
    // 삭제된 포인트 수를 반환합니다.
}

template <typename PointType>
void KD_TREE<PointType>::Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild) {
    // 주어진 포인트를 삭제하는 함수입니다.

    if ((*root) == nullptr || (*root)->tree_deleted) return;
    // 현재 노드가 nullptr이거나 이미 삭제된 경우, 삭제 작업을 수행하지 않고 함수를 종료합니다.

    (*root)->working_flag = true;
    // 현재 노드의 작업 플래그를 활성화합니다.

    Push_Down(*root);
    // 현재 노드 아래로 작업을 전파합니다.

    if (same_point((*root)->point, point) && !(*root)->point_deleted) {          
        // 현재 노드의 포인트가 삭제할 포인트와 동일하고 아직 삭제되지 않은 경우
        (*root)->point_deleted = true;
        // 현재 노드의 포인트를 삭제 처리합니다.
        (*root)->invalid_point_num += 1;
        // 현재 노드의 유효하지 않은 포인트 수를 증가시킵니다.
        if ((*root)->invalid_point_num == (*root)->TreeSize) (*root)->tree_deleted = true;
        // 만약 모든 포인트가 삭제된 경우, 현재 노드를 삭제 처리합니다.
        return;
    }
    
    // Operation_Logger_Type 타입의 삭제 로그를 생성합니다.
    Operation_Logger_Type delete_log;
    
    // 스레드 대기 시간을 설정하기 위한 timespec 구조체를 생성합니다.
    struct timespec Timeout;

    // DELETE_POINT 작업을 설정합니다.
    delete_log.op = DELETE_POINT;
    // 삭제할 포인트 정보를 로그에 추가합니다.
    delete_log.point = point;
    // 삭제할 포인트를 로그에 설정합니다.

      // 현재 노드의 분할 축에 따라 포인트를 왼쪽 노드 또는 오른쪽 노드로 분류하여 삭제 작업을 수행합니다.
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) ||
        ((*root)->division_axis == 1 && point.y < (*root)->point.y) ||
        ((*root)->division_axis == 2 && point.z < (*root)->point.z)) {           
        // 포인트가 현재 노드의 분할 축을 기준으로 왼쪽에 있는 경우

        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr) {          
            // Rebuild_Ptr가 nullptr이거나 현재 노드의 왼쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
            // 왼쪽 자식 노드로 이동하여 삭제 작업을 수행합니다.
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);         
        } else {
            // Rebuild_Ptr와 현재 노드의 왼쪽 자식 노드가 같은 경우
            pthread_mutex_lock(&working_flag_mutex);
            // working_flag_mutex 뮤텍스를 락하여 동기화를 수행합니다.

            // 왼쪽 자식 노드로 이동하여 삭제 작업을 수행하고, Rebuild 관련 로그를 추가합니다.
            Delete_by_point(&(*root)->left_son_ptr, point, false);
            
            if (rebuild_flag) {
                // 재구성 플래그가 활성화된 경우
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                // rebuild_logger_mutex_lock 뮤텍스를 락하여 로그 작업 동기화를 수행합니다.
                Rebuild_Logger.push(delete_log);
                // 삭제 로그를 재구성 로그 큐에 추가합니다.
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                // rebuild_logger_mutex_lock 뮤텍스 언락
            }

            pthread_mutex_unlock(&working_flag_mutex);
            // working_flag_mutex 뮤텍스 언락
        }
    }else {       
        // 포인트가 현재 노드의 분할 축을 기준으로 오른쪽에 있는 경우

        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr) {         
            // Rebuild_Ptr가 nullptr이거나 현재 노드의 오른쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
            // 오른쪽 자식 노드로 이동하여 삭제 작업을 수행합니다.
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);         
        } else {
            // Rebuild_Ptr와 현재 노드의 오른쪽 자식 노드가 같은 경우
            pthread_mutex_lock(&working_flag_mutex); 
            // working_flag_mutex 뮤텍스를 락하여 동기화를 수행합니다.

            // 오른쪽 자식 노드로 이동하여 삭제 작업을 수행하고, Rebuild 관련 로그를 추가합니다.
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            
            if (rebuild_flag) {
                // 재구성 플래그가 활성화된 경우
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                // rebuild_logger_mutex_lock 뮤텍스를 락하여 로그 작업 동기화를 수행합니다.
                Rebuild_Logger.push(delete_log);
                // 삭제 로그를 재구성 로그 큐에 추가합니다.
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                // rebuild_logger_mutex_lock 뮤텍스 언락
            }

            pthread_mutex_unlock(&working_flag_mutex);
            // working_flag_mutex 뮤텍스 언락
        }        
    }

    Update(*root);
    // 현재 노드를 업데이트합니다.

    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) 
        Rebuild_Ptr = nullptr;
    // 만약 Rebuild_Ptr가 현재 노드를 가리키고, 현재 노드의 트리 크기가 Multi_Thread_Rebuild_Point_Num보다 작다면, Rebuild_Ptr를 nullptr로 설정합니다.

    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    // 재구성이 필요한지 여부를 결정하기 위한 조건을 검사합니다.
    
    if (need_rebuild) Rebuild(root);
    // 재구성이 필요하다면 KD 트리를 재구성합니다.

    if ((*root) != nullptr) (*root)->working_flag = false;   
    // 현재 노드의 작업 플래그를 비활성화합니다.
}

    
template <typename PointType>
void KD_TREE<PointType>::Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild){
    // 주어진 범위 내에 포함된 상자 포인트를 KD 트리에 추가하는 함수입니다.

    if ((*root) == nullptr) return;
    // 루트 노드가 없으면 추가 중지

    (*root)->working_flag = true;
    // 현재 노드의 작업 플래그를 설정하여 작업을 진행합니다.

    Push_Down(*root);
    // 현재 노드를 푸시 다운하여 작업을 준비합니다.

    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return;
    // 상자 포인트가 현재 노드의 X 범위를 벗어나면 추가 중지

    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return;
    // 상자 포인트가 현재 노드의 Y 범위를 벗어나면 추가 중지

    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return;
    // 상자 포인트가 현재 노드의 Z 범위를 벗어나면 추가 중지

    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] &&
        boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1]) {
        // 상자 포인트가 현재 노드의 범위를 완전히 포함하는 경우

        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;
        // 현재 노드를 삭제되지 않음 및 포인트 삭제되지 않음 상태로 설정하고, 필요한 푸시 다운을 지정합니다.
    }
    return;
}

if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x &&
    boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y &&
    boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z) {
    // 현재 노드의 포인트가 상자 포인트의 범위 내에 있는 경우

    (*root)->point_deleted = (*root)->point_downsample_deleted;
    // 현재 노드의 포인트 삭제 상태를 현재 노드의 다운샘플 삭제 상태로 설정합니다.
}

Operation_Logger_Type add_box_log;
struct timespec Timeout;    
add_box_log.op = ADD_BOX;
add_box_log.boxpoint = boxpoint;
// 상자 포인트 추가 작업을 로그에 기록하기 위한 로그 구조체를 생성합니다.

if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr) {
    // Rebuild_Ptr가 nullptr이거나 현재 노드의 왼쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
    // 왼쪽 자식 노드로 재귀적으로 상자 포인트 추가 작업을 수행합니다.
    Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
} else {
    // Rebuild_Ptr가 있고 현재 노드의 왼쪽 자식 노드가 Rebuild_Ptr와 같은 경우
    // 상자 포인트 추가 작업 중에 다른 스레드가 노드를 재구성할 수 있으므로 뮤텍스로 동기화합니다.
    pthread_mutex_lock(&working_flag_mutex);
    Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
    if (rebuild_flag) {
        pthread_mutex_lock(&rebuild_logger_mutex_lock);
        Rebuild_Logger.push(add_box_log);
        pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
    }
    pthread_mutex_unlock(&working_flag_mutex);
}

  if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr) {
    // Rebuild_Ptr가 nullptr이거나 현재 노드의 오른쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
    // 오른쪽 자식 노드로 재귀적으로 상자 포인트 추가 작업을 수행합니다.
    Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
} else {
    // Rebuild_Ptr가 있고 현재 노드의 오른쪽 자식 노드가 Rebuild_Ptr와 같은 경우
    // 상자 포인트 추가 작업 중에 다른 스레드가 노드를 재구성할 수 있으므로 뮤텍스로 동기화합니다.
    pthread_mutex_lock(&working_flag_mutex);
    Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
    if (rebuild_flag) {
        pthread_mutex_lock(&rebuild_logger_mutex_lock);
        Rebuild_Logger.push(add_box_log);
        pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
    }
    pthread_mutex_unlock(&working_flag_mutex);
}

Update(*root);
// 현재 노드를 업데이트합니다.

if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
// Rebuild_Ptr가 현재 노드를 가리키고 현재 노드의 포인트 수가 Multi_Thread_Rebuild_Point_Num보다 작으면 Rebuild_Ptr를 nullptr로 설정합니다.

bool need_rebuild = allow_rebuild & Criterion_Check((*root));
// 재귀적으로 노드를 추가한 후에 재구성이 필요한지 여부를 판단합니다.

if (need_rebuild) Rebuild(root);
// 필요하면 노드를 재구성합니다.

if ((*root) != nullptr) (*root)->working_flag = false;   
// 현재 노드의 작업 플래그를 false로 설정하여 작업이 완료되었음을 나타냅니다.

return;
}
template <typename PointType>
void KD_TREE<PointType>::Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild, int father_axis){     
    // 만약 루트 노드가 없다면 새로운 노드를 생성하고 초기화합니다.
    if (*root == nullptr){
        *root = new KD_TREE_NODE;
        InitTreeNode(*root); // 노드를 초기화합니다.
        (*root)->point = point; // 노드에 포인트를 할당합니다.
        (*root)->division_axis = (father_axis + 1) % 3; // 분할 축을 설정합니다.
        Update(*root); // 노드 정보를 업데이트합니다.
        return;
    }
    (*root)->working_flag = true; // 현재 노드를 작업 중으로 표시합니다.
    Operation_Logger_Type add_log;
    struct timespec Timeout;
    add_log.op = ADD_POINT; // 로그 작업 유형을 "포인트 추가"로 설정합니다.
    add_log.point = point; // 로그에 추가할 포인트를 설정합니다.
    Push_Down(*root); // 현재 노드를 내려가면서 작업 플래그를 설정합니다.
    
    // 현재 노드의 분할 축에 따라 포인트를 왼쪽 또는 오른쪽 서브트리로 추가합니다.
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){          
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild, (*root)->division_axis);
        } else {
            // 재귀적으로 왼쪽 서브트리로 이동하면서 Rebuild_Ptr와 노드의 left_son_ptr을 비교하고, 필요한 경우 재구축 작업을 로깅합니다.
            pthread_mutex_lock(&working_flag_mutex); // 스레드 뮤텍스를 사용하여 작업 플래그를 보호합니다.
            Add_by_point(&(*root)->left_son_ptr, point, false, (*root)->division_axis); // 재귀 호출
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock); // 스레드 뮤텍스를 사용하여 재구축 로그를 보호합니다.
                Rebuild_Logger.push(add_log); // 재구축 로그를 큐에 추가합니다.
                pthread_mutex_unlock(&rebuild_logger_mutex_lock); // 스레드 뮤텍스 잠금 해제
            }
            pthread_mutex_unlock(&working_flag_mutex); // 스레드 뮤텍스 잠금 해제
        }
        
    } else {  
    // 현재 노드의 분할 축에 따라 포인트를 오른쪽 또는 왼쪽 서브트리로 추가합니다.
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
        Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild, (*root)->division_axis);
    } else {
        // 재귀적으로 오른쪽 서브트리로 이동하면서 Rebuild_Ptr와 노드의 right_son_ptr을 비교하고, 필요한 경우 재구축 작업을 로깅합니다.
        pthread_mutex_lock(&working_flag_mutex); // 스레드 뮤텍스를 사용하여 작업 플래그를 보호합니다.
        Add_by_point(&(*root)->right_son_ptr, point, false, (*root)->division_axis); // 재귀 호출
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock); // 스레드 뮤텍스를 사용하여 재구축 로그를 보호합니다.
            Rebuild_Logger.push(add_log); // 재구축 로그를 큐에 추가합니다.
            pthread_mutex_unlock(&rebuild_logger_mutex_lock); // 스레드 뮤텍스 잠금 해제
        }
        pthread_mutex_unlock(&working_flag_mutex); // 스레드 뮤텍스 잠금 해제
    }
}

Update(*root); // 현재 노드를 업데이트합니다.

// Rebuild_Ptr가 현재 노드와 일치하고 현재 노드의 TreeSize가 Multi_Thread_Rebuild_Point_Num보다 작으면 Rebuild_Ptr를 nullptr로 설정합니다.
if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr;

// 필요한 경우 재구축 작업을 수행하기 위한 조건을 확인합니다.
bool need_rebuild = allow_rebuild & Criterion_Check((*root));
if (need_rebuild) Rebuild(root); // 재구축 함수를 호출하여 트리를 재구축합니다.

if ((*root) != nullptr) (*root)->working_flag = false; // 현재 노드의 작업 플래그를 false로 설정합니다.
return;
}

template <typename PointType>
void KD_TREE<PointType>::Search(KD_TREE_NODE * root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist){
    // 루트 노드가 없거나 삭제된 노드인 경우 함수를 종료합니다.
    if (root == nullptr || root->tree_deleted) return;   

    // 현재 노드와 목표 포인트 간의 거리를 계산합니다.
    double cur_dist = calc_box_dist(root, point);
    double max_dist_sqr = max_dist * max_dist;

    // 현재 노드와 목표 포인트 간의 거리가 최대 거리보다 크면 함수를 종료합니다.
    if (cur_dist > max_dist_sqr) return;    

    int retval; 
    if (root->need_push_down_to_left || root->need_push_down_to_right) {
        // 현재 노드가 왼쪽 또는 오른쪽 서브트리로 push_down이 필요한 경우,
        // 스레드 뮤텍스를 사용하여 push_down을 수행합니다.
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0){
            Push_Down(root);
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        } else {
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }

    if (!root->point_deleted){
        // 현재 노드에 삭제되지 않은 포인트가 있으면,
        // 목표 포인트와의 거리를 계산하고 k_nearest보다 작은 거리인 경우
        // MANUAL_HEAP에 해당 포인트를 추가합니다.
        float dist = calc_dist(point, root->point);
        if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist)){
            if (q.size() >= k_nearest) q.pop(); // MANUAL_HEAP의 크기가 k_nearest를 초과하면 가장 먼 포인트를 제거합니다.
            PointType_CMP current_point{root->point, dist};                    
            q.push(current_point); // 새로운 포인트를 MANUAL_HEAP에 추가합니다.
        }
    }  
    
   int cur_search_counter; // 현재 검색 횟수를 추적하는 변수

float dist_left_node = calc_box_dist(root->left_son_ptr, point); // 왼쪽 자식 노드와 입력 지점 간의 거리를 계산
float dist_right_node = calc_box_dist(root->right_son_ptr, point); // 오른쪽 자식 노드와 입력 지점 간의 거리를 계산

// 우선순위 큐 'q'의 크기가 'k_nearest'보다 작거나, 
// 왼쪽 자식 노드와 오른쪽 자식 노드 간의 거리가 현재 우선순위 큐 'q'의 가장 높은 우선순위 항목의 거리보다 작을 경우
if (q.size() < k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist) {
    // 왼쪽 자식 노드와 오른쪽 자식 노드 간의 거리를 비교하여 어떤 노드를 먼저 검색할지 결정
    if (dist_left_node <= dist_right_node) {
        // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 왼쪽 자식을 가리키지 않거나 다르면,
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr) {
            // 검색을 수행하는 함수 'Search'를 호출하여 왼쪽 자식 노드를 검색
            Search(root->left_son_ptr, k_nearest, point, q, max_dist);
        } else {
            // 검색을 수행하는 동안 다른 스레드가 이미 'Search' 함수를 실행 중이므로
            // 이 스레드는 'search_flag_mutex'를 사용하여 동기화 작업 수행
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter == -1) {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            // 검색 카운터를 증가시키고 뮤텍스 잠금 해제
            search_mutex_counter += 1;
            pthread_mutex_unlock(&search_flag_mutex);
            
            // 왼쪽 자식 노드를 검색하는 함수 'Search'를 호출
            Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            
            // 검색이 완료되면 검색 카운터를 다시 감소시키고 뮤텍스 잠금 해제
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter -= 1;
            pthread_mutex_unlock(&search_flag_mutex);
        }
            // 우선순위 큐 'q'의 크기가 'k_nearest'보다 작거나, 
            // 오른쪽 자식 노드와 현재 우선순위 큐 'q'의 가장 높은 우선순위 항목의 거리보다 작을 경우
            if (q.size() < k_nearest || dist_right_node < q.top().dist) {
                // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 오른쪽 자식을 가리키지 않거나 다르면,
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr) {
                    // 검색을 수행하는 함수 'Search'를 호출하여 오른쪽 자식 노드를 검색
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                } else {
                    // 검색을 수행하는 동안 다른 스레드가 이미 'Search' 함수를 실행 중이므로
                    // 이 스레드는 'search_flag_mutex'를 사용하여 동기화 작업 수행
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1) {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    // 검색 카운터를 증가시키고 뮤텍스 잠금 해제
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    
                    // 오른쪽 자식 노드를 검색하는 함수 'Search'를 호출
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                    
                    // 검색이 완료되면 검색 카운터를 다시 감소시키고 뮤텍스 잠금 해제
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }                
            }

        } else {
        // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 오른쪽 자식을 가리키지 않거나 다르면,
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr) {
            // 검색을 수행하는 함수 'Search'를 호출하여 오른쪽 자식 노드를 검색
            Search(root->right_son_ptr, k_nearest, point, q, max_dist);
        } else {
            // 검색을 수행하는 동안 다른 스레드가 이미 'Search' 함수를 실행 중이므로
            // 이 스레드는 'search_flag_mutex'를 사용하여 동기화 작업 수행
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter == -1) {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            // 검색 카운터를 증가시키고 뮤텍스 잠금 해제
            search_mutex_counter += 1;
            pthread_mutex_unlock(&search_flag_mutex);
            
            // 오른쪽 자식 노드를 검색하는 함수 'Search'를 호출
            Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            
            // 검색이 완료되면 검색 카운터를 다시 감소시키고 뮤텍스 잠금 해제
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter -= 1;
            pthread_mutex_unlock(&search_flag_mutex);
        }

        
            // 우선순위 큐 'q'의 크기가 'k_nearest'보다 작거나, 
            // 왼쪽 자식 노드와 현재 우선순위 큐 'q'의 가장 높은 우선순위 항목의 거리보다 작을 경우
            if (q.size() < k_nearest || dist_left_node < q.top().dist) {
                // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 왼쪽 자식을 가리키지 않거나 다르면,
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr) {
                    // 검색을 수행하는 함수 'Search'를 호출하여 왼쪽 자식 노드를 검색
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                } else {
                    // 검색을 수행하는 동안 다른 스레드가 이미 'Search' 함수를 실행 중이므로
                    // 이 스레드는 'search_flag_mutex'를 사용하여 동기화 작업 수행
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1) {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    // 검색 카운터를 증가시키고 뮤텍스 잠금 해제
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    
                    // 왼쪽 자식 노드를 검색하는 함수 'Search'를 호출
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                    
                    // 검색이 완료되면 검색 카운터를 다시 감소시키고 뮤텍스 잠금 해제
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }

        }
    } else {
      // 이전 코드에서 오른쪽 자식 노드를 검색하지 않는 경우에 해당하는 'else' 블록입니다.
    // 현재 노드의 왼쪽 자식과 입력 지점 간의 거리가 현재 우선순위 큐 'q'의 가장 높은 우선순위 항목의 거리보다 작을 경우
    if (dist_left_node < q.top().dist) {
        // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 왼쪽 자식을 가리키지 않거나 다르면,
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr) {
            // 검색을 수행하는 함수 'Search'를 호출하여 왼쪽 자식 노드를 검색
            Search(root->left_son_ptr, k_nearest, point, q, max_dist);
        } else {
            // 검색을 수행하는 동안 다른 스레드가 이미 'Search' 함수를 실행 중이므로
            // 이 스레드는 'search_flag_mutex'를 사용하여 동기화 작업 수행
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter == -1) {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            // 검색 카운터를 증가시키고 뮤텍스 잠금 해제
            search_mutex_counter += 1;
            pthread_mutex_unlock(&search_flag_mutex);
            
            // 왼쪽 자식 노드를 검색하는 함수 'Search'를 호출
            Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            
            // 검색이 완료되면 검색 카운터를 다시 감소시키고 뮤텍스 잠금 해제
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter -= 1;
            pthread_mutex_unlock(&search_flag_mutex);
        }
    }

        // 현재 노드의 오른쪽 자식과 입력 지점 간의 거리가 현재 우선순위 큐 'q'의 가장 높은 우선순위 항목의 거리보다 작을 경우
        if (dist_right_node < q.top().dist) {
            // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 오른쪽 자식을 가리키지 않거나 다르면,
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr) {
                // 검색을 수행하는 함수 'Search'를 호출하여 오른쪽 자식 노드를 검색
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            } else {
                // 검색을 수행하는 동안 다른 스레드가 이미 'Search' 함수를 실행 중이므로
                // 이 스레드는 'search_flag_mutex'를 사용하여 동기화 작업 수행
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1) {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                // 검색 카운터를 증가시키고 뮤텍스 잠금 해제
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                
                // 오른쪽 자식 노드를 검색하는 함수 'Search'를 호출
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                
                // 검색이 완료되면 검색 카운터를 다시 감소시키고 뮤텍스 잠금 해제
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector & Storage){
    if (root == nullptr) return; // 루트 노드가 없으면 함수를 종료합니다.
    Push_Down(root); // 현재 노드를 내려가면서 작업 플래그를 설정합니다.

    // 박스 범위와 현재 노드의 X, Y, Z 범위를 비교하여 겹치지 않으면 함수를 종료합니다.
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1]) return;

    // 박스가 현재 노드를 완전히 포함하면서 노드를 평면화하여 결과를 저장합니다.
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] && boxpoint.vertex_max[0] > root->node_range_x[1] && boxpoint.vertex_min[1] <= root->node_range_y[0] && boxpoint.vertex_max[1] > root->node_range_y[1] && boxpoint.vertex_min[2] <= root->node_range_z[0] && boxpoint.vertex_max[2] > root->node_range_z[1]){
        flatten(root, Storage, NOT_RECORD);
        return;
    }

    // 현재 노드의 포인트가 박스 범위 내에 있고 포인트가 삭제되지 않았다면 결과 벡터에 추가합니다.
    if (boxpoint.vertex_min[0] <= root->point.x && boxpoint.vertex_max[0] > root->point.x && boxpoint.vertex_min[1] <= root->point.y && boxpoint.vertex_max[1] > root->point.y && boxpoint.vertex_min[2] <= root->point.z && boxpoint.vertex_max[2] > root->point.z){
        if (!root->point_deleted) Storage.push_back(root->point);
    }

    // 왼쪽 서브트리와 오른쪽 서브트리를 재귀적으로 탐색합니다.
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }

    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }

    return;    
}


template <typename PointType>
void KD_TREE<PointType>::Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage)
{
    // 주어진 반경 내에서 KD 트리를 검색하는 함수입니다.

    if (root == nullptr)
        return;
    // 루트 노드가 없으면 검색 중지

    Push_Down(root);
    // 현재 노드를 푸시 다운하여 작업을 준비합니다.

    PointType range_center;
    // 현재 노드의 범위 중심점을 계산합니다.
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
    range_center.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;
    
    float dist = sqrt(calc_dist(range_center, point));
    // 현재 노드의 범위 중심점과 대상 포인트 간의 거리를 계산합니다.

    if (dist > radius + sqrt(root->radius_sq))
        return;
    // 현재 노드와 대상 포인트 간의 거리가 반경 + 현재 노드의 반지름의 제곱근보다 크면 검색 중지

    if (dist <= radius - sqrt(root->radius_sq)) 
    {
        // 현재 노드와 대상 포인트 간의 거리가 반경 - 현재 노드의 반지름의 제곱근보다 작거나 같으면
        // 현재 노드의 모든 포인트를 저장하고 검색을 종료합니다.
        flatten(root, Storage, NOT_RECORD);
        return;
    }

    if (!root->point_deleted && calc_dist(root->point, point) <= radius * radius){
        // 현재 노드의 포인트가 삭제되지 않았으며 대상 포인트와의 거리가 반경 내에 있는 경우
        // 해당 포인트를 결과 벡터에 추가합니다.
        Storage.push_back(root->point);
    }

    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        // Rebuild_Ptr가 nullptr이거나 현재 노드의 왼쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
        // 왼쪽 자식 노드로 재귀적으로 검색을 수행합니다.
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
    }
    else
    {
        // Rebuild_Ptr가 있고 현재 노드의 왼쪽 자식 노드가 Rebuild_Ptr와 같은 경우
        // 검색 중에 다른 스레드가 노드를 재구성할 수 있으므로 뮤텍스로 동기화합니다.
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }

    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        // Rebuild_Ptr가 nullptr이거나 현재 노드의 오른쪽 자식 노드가 Rebuild_Ptr와 같지 않으면
        // 오른쪽 자식 노드로 재귀적으로 검색을 수행합니다.
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
    }
    else
    {
        // Rebuild_Ptr가 있고 현재 노드의 오른쪽 자식 노드가 Rebuild_Ptr와 같은 경우
        // 검색 중에 다른 스레드가 노드를 재구성할 수 있으므로 뮤텍스로 동기화합니다.
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }    
    return;
}

template <typename PointType>
bool KD_TREE<PointType>::Criterion_Check(KD_TREE_NODE * root) {
    // KD 트리 재구성 조건을 검사하는 함수입니다.

    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size) {
        // 트리 크기가 최소 불균형 트리 크기보다 작으면 재구성 조건을 충족하지 않습니다.
        return false;
    }
    
    float balance_evaluation = 0.0f;
    float delete_evaluation = 0.0f;
    
    KD_TREE_NODE * son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
    // 왼쪽 자식 노드가 없을 경우 오른쪽 자식 노드를 사용합니다.

    delete_evaluation = float(root->invalid_point_num) / root->TreeSize;
    // 삭제된 포인트 수와 전체 포인트 수를 비교하여 삭제 평가를 수행합니다.

    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize - 1);
    // 자식 노드의 크기와 부모 노드에서 현재 노드를 제외한 크기를 비교하여 균형 평가를 수행합니다.

    if (delete_evaluation > delete_criterion_param) {
        // 삭제 평가가 삭제 재구성 기준을 초과하는 경우, 재구성이 필요합니다.
        return true;
    }

    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1 - balance_criterion_param) {
        // 균형 평가가 균형 재구성 기준을 벗어나는 경우, 재구성이 필요합니다.
        return true;
    } 

    // 모든 재구성 조건을 충족하지 않는 경우, 재구성이 필요하지 않습니다.
    return false;
}


template <typename PointType>
void KD_TREE<PointType>::Push_Down(KD_TREE_NODE *root) {
    if (root == nullptr) return;
    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;

    // 현재 노드가 왼쪽 자식으로 'push_down'을 해야하고, 왼쪽 자식 노드가 존재하는 경우
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr) {
        // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 왼쪽 자식을 가리키지 않거나 다르면,
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr) {
            // 왼쪽 자식 노드에 대한 플래그와 속성 업데이트 수행
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;

            // 필요한 경우 노드의 삭제된 요소 수와 무효화된 포인트 수 업데이트
            if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
            else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;

            // 왼쪽 자식 노드의 'need_push_down_to_left' 및 'need_push_down_to_right' 플래그 업데이트
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;

            // 현재 노드의 'need_push_down_to_left' 플래그를 비활성화
            root->need_push_down_to_left = false;
  
        } else {
            
        pthread_mutex_lock(&working_flag_mutex); // 작업 플래그 뮤텍스를 잠그기 시작
    
        // 왼쪽 자식 노드의 'tree_downsample_deleted' 및 'point_downsample_deleted' 플래그 업데이트
        root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
        root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
        
        // 왼쪽 자식 노드의 'tree_deleted' 및 'point_deleted' 속성 업데이트
        root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
        root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
        
        // 'tree_downsample_deleted' 플래그에 따라 왼쪽 자식 노드의 'down_del_num' 업데이트
        if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
        
        // 'tree_deleted' 플래그에 따라 왼쪽 자식 노드의 'invalid_point_num' 업데이트
        if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
        else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
        
        // 왼쪽 자식 노드의 'need_push_down_to_left' 및 'need_push_down_to_right' 플래그 업데이트
        root->left_son_ptr->need_push_down_to_left = true;
        root->left_son_ptr->need_push_down_to_right = true;
        
        // 'rebuild_flag'가 true인 경우 작업 로그를 'Rebuild_Logger'에 추가
        if (rebuild_flag) {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(operation);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        
        // 현재 노드의 'need_push_down_to_left' 플래그를 비활성화
        root->need_push_down_to_left = false;
        
        pthread_mutex_unlock(&working_flag_mutex); // 작업 플래그 뮤텍스 잠금 해제
        }
    }
    
    // 현재 노드가 오른쪽 자식으로 'push_down'을 해야하고, 오른쪽 자식 노드가 존재하는 경우
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr) {
        // 'Rebuild_Ptr'이 'nullptr'이거나 'Rebuild_Ptr'이 현재 노드의 오른쪽 자식을 가리키지 않거나 다르면,
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr) {
            // 오른쪽 자식 노드에 대한 플래그와 속성 업데이트 수행
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
    
            // 필요한 경우 노드의 삭제된 요소 수와 무효화된 포인트 수 업데이트
            if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
            else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
    
            // 오른쪽 자식 노드의 'need_push_down_to_left' 및 'need_push_down_to_right' 플래그 업데이트
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
    
            // 현재 노드의 'need_push_down_to_right' 플래그를 비활성화
            root->need_push_down_to_right = false;
            
        } 

        else {
    pthread_mutex_lock(&working_flag_mutex); // 작업 플래그 뮤텍스를 잠그기 시작

    // 오른쪽 자식 노드의 'tree_downsample_deleted' 및 'point_downsample_deleted' 플래그 업데이트
    root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
    root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;

    // 오른쪽 자식 노드의 'tree_deleted' 및 'point_deleted' 속성 업데이트
    root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
    root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;

    // 'tree_downsample_deleted' 플래그에 따라 오른쪽 자식 노드의 'down_del_num' 업데이트
    if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;

    // 'tree_deleted' 플래그에 따라 오른쪽 자식 노드의 'invalid_point_num' 업데이트
    if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
    else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;

    // 오른쪽 자식 노드의 'need_push_down_to_left' 및 'need_push_down_to_right' 플래그 업데이트
    root->right_son_ptr->need_push_down_to_left = true;
    root->right_son_ptr->need_push_down_to_right = true;

    // 'rebuild_flag'가 true인 경우 작업 로그를 'Rebuild_Logger'에 추가
    if (rebuild_flag) {
        pthread_mutex_lock(&rebuild_logger_mutex_lock);
        Rebuild_Logger.push(operation);
        pthread_mutex_unlock(&rebuild_logger_mutex_lock);
    }

    // 현재 노드의 'need_push_down_to_right' 플래그를 비활성화
    root->need_push_down_to_right = false;

    pthread_mutex_unlock(&working_flag_mutex); // 작업 플래그 뮤텍스 잠금 해제
}
return; // 함수 종료
}

template <typename PointType>
void KD_TREE<PointType>::Update(KD_TREE_NODE * root) {
    // 현재 노드의 왼쪽 자식 및 오른쪽 자식 포인터를 복사
    KD_TREE_NODE * left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE * right_son_ptr = root->right_son_ptr;

    // 임시 변수를 사용하여 초기 범위를 설정합니다. (x, y, z 각각 최솟값과 최댓값을 초기화)
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};

    // 트리 크기 업데이트
if (left_son_ptr != nullptr && right_son_ptr != nullptr) {
    // 현재 노드의 트리 크기 업데이트: 왼쪽 자식의 트리 크기 + 오른쪽 자식의 트리 크기 + 1 (현재 노드 포함)
    root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;

    // 현재 노드의 무효화된 포인트 수 업데이트:
    // 왼쪽 자식의 무효화된 포인트 수 + 오른쪽 자식의 무효화된 포인트 수 + (현재 노드가 삭제된 경우 1, 그렇지 않으면 0)
    root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);

    // 현재 노드의 다운샘플 삭제된 포인트 수 업데이트:
    // 왼쪽 자식의 다운샘플 삭제된 포인트 수 + 오른쪽 자식의 다운샘플 삭제된 포인트 수 + (현재 노드가 다운샘플 삭제된 경우 1, 그렇지 않으면 0)
    root->down_del_num = left_son_ptr->down_del_num + right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);

    // 현재 노드의 트리 다운샘플 삭제된 플래그 업데이트:
    // 왼쪽 자식의 트리 다운샘플 삭제된 플래그와 오른쪽 자식의 트리 다운샘플 삭제된 플래그와 현재 노드의 포인트 다운샘플 삭제된 플래그의 논리 AND
    root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;

    // 현재 노드의 트리 삭제된 플래그 업데이트:
    // 왼쪽 자식의 트리 삭제된 플래그와 오른쪽 자식의 트리 삭제된 플래그와 현재 노드의 포인트 삭제된 플래그의 논리 AND
    root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;

    // 현재 노드의 임시 범위 업데이트:
    // x, y 및 z 좌표에 대한 최솟값 및 최댓값을 업데이트하여 임시 범위 설정
    tmp_range_x[0] = min(min(left_son_ptr->node_range_x[0], right_son_ptr->node_range_x[0]), root->point.x);
    tmp_range_x[1] = max(max(left_son_ptr->node_range_x[1], right_son_ptr->node_range_x[1]), root->point.x);
    tmp_range_y[0] = min(min(left_son_ptr->node_range_y[0], right_son_ptr->node_range_y[0]), root->point.y);
    tmp_range_y[1] = max(max(left_son_ptr->node_range_y[1], right_son_ptr->node_range_y[1]), root->point.y);
    tmp_range_z[0] = min(min(left_son_ptr->node_range_z[0], right_son_ptr->node_range_z[0]), root->point.z);
    tmp_range_z[1] = max(max(left_son_ptr->node_range_z[1], right_son_ptr->node_range_z[1]), root->point.z);

        } else {
            // 왼쪽 자식 노드가 삭제되지 않은 경우에만 다음을 수행
            if (!left_son_ptr->tree_deleted) {
                // 임시 범위의 x, y 및 z 좌표에 대한 최솟값 및 최댓값을 업데이트
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }

           if (!right_son_ptr->tree_deleted) {
            // 오른쪽 자식 노드가 삭제되지 않은 경우에만 다음을 수행
            // 임시 범위의 x, y 및 z 좌표에 대한 최솟값 및 최댓값을 업데이트
            tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
            tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
            tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
            tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
            tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
            tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
        }
            if (!root->point_deleted) {
            // 현재 노드의 포인트가 삭제되지 않은 경우에만 다음을 수행
            // 임시 범위의 x, y 및 z 좌표에 대한 최솟값 및 최댓값을 업데이트
            tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
            tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
            tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
            tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
            tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
            tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
        }
        }
        } else if (left_son_ptr != nullptr) {
        // 왼쪽 자식 노드만 있는 경우 다음을 수행
        // 현재 노드의 트리 크기 업데이트: 왼쪽 자식의 트리 크기 + 1 (현재 노드 포함)
        root->TreeSize = left_son_ptr->TreeSize + 1;
    
        // 현재 노드의 무효화된 포인트 수 업데이트:
        // 왼쪽 자식의 무효화된 포인트 수 + (현재 노드가 삭제된 경우 1, 그렇지 않으면 0)
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
    
        // 현재 노드의 다운샘플 삭제된 포인트 수 업데이트:
        // 왼쪽 자식의 다운샘플 삭제된 포인트 수 + (현재 노드가 다운샘플 삭제된 경우 1, 그렇지 않으면 0)
        root->down_del_num = left_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
    
        // 현재 노드의 트리 다운샘플 삭제된 플래그 업데이트:
        // 왼쪽 자식의 트리 다운샘플 삭제된 플래그와 현재 노드의 포인트 다운샘플 삭제된 플래그의 논리 AND
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
    
        // 현재 노드의 트리 삭제된 플래그 업데이트:
        // 왼쪽 자식의 트리 삭제된 플래그와 현재 노드의 포인트 삭제된 플래그의 논리 AND
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;


    
      if (root->tree_deleted || (!left_son_ptr->tree_deleted && !root->point_deleted)) {
        // 현재 노드의 트리가 삭제되었거나
        // 왼쪽 자식 노드의 트리와 현재 노드의 포인트가 모두 삭제되지 않은 경우
        // 다음을 수행
    
        // 임시 범위의 x, y 및 z 좌표를 업데이트:
        // 왼쪽 자식 노드의 x, y, z 범위와 현재 노드의 포인트를 고려하여 최솟값과 최댓값을 결정
        tmp_range_x[0] = min(left_son_ptr->node_range_x[0], root->point.x);
        tmp_range_x[1] = max(left_son_ptr->node_range_x[1], root->point.x);
        tmp_range_y[0] = min(left_son_ptr->node_range_y[0], root->point.y);
        tmp_range_y[1] = max(left_son_ptr->node_range_y[1], root->point.y);
        tmp_range_z[0] = min(left_son_ptr->node_range_z[0], root->point.z);
        tmp_range_z[1] = max(left_son_ptr->node_range_z[1], root->point.z);
    }

        
        else {
        // 현재 노드의 트리가 삭제되지 않은 경우
        // 왼쪽 자식 노드의 트리가 삭제되지 않은 경우에만 다음을 수행
    
        // 임시 범위의 x, y 및 z 좌표를 업데이트:
        // 왼쪽 자식 노드의 x, y, z 범위를 고려하여 최솟값과 최댓값을 결정
        tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
        tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
        tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
        tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
        tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
        tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
    }

            if (!root->point_deleted) {
            // 현재 노드의 포인트가 삭제되지 않은 경우에만 다음을 수행
            // 임시 범위의 x, y 및 z 좌표에 대한 최솟값 및 최댓값을 업데이트
            tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
            tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
            tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
            tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
            tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
            tmp_range_z[1] = max(tmp_range_z[1], root->point.z);     
            }            
        }

    } else if (right_son_ptr != nullptr) {
    // 오른쪽 자식 노드만 있는 경우 다음을 수행
    // 현재 노드의 트리 크기 업데이트: 오른쪽 자식의 트리 크기 + 1 (현재 노드 포함)
    root->TreeSize = right_son_ptr->TreeSize + 1;

    // 현재 노드의 무효화된 포인트 수 업데이트:
    // 오른쪽 자식의 무효화된 포인트 수 + (현재 노드가 삭제된 경우 1, 그렇지 않으면 0)
    root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);

    // 현재 노드의 다운샘플 삭제된 포인트 수 업데이트:
    // 오른쪽 자식의 다운샘플 삭제된 포인트 수 + (현재 노드가 다운샘플 삭제된 경우 1, 그렇지 않으면 0)
    root->down_del_num = right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);

    // 현재 노드의 트리 다운샘플 삭제된 플래그 업데이트:
    // 오른쪽 자식의 트리 다운샘플 삭제된 플래그와 현재 노드의 포인트 다운샘플 삭제된 플래그의 논리 AND
    root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;

    // 현재 노드의 트리 삭제된 플래그 업데이트:
    // 오른쪽 자식의 트리 삭제된 플래그와 현재 노드의 포인트 삭제된 플래그의 논리 AND
    root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;

    if (root->tree_deleted || (!right_son_ptr->tree_deleted && !root->point_deleted)) {
        // 현재 노드의 트리가 삭제되었거나
        // 오른쪽 자식 노드의 트리와 현재 노드의 포인트가 모두 삭제되지 않은 경우
        // 다음을 수행

        // 임시 범위의 x, y 및 z 좌표를 업데이트:
        // 오른쪽 자식 노드의 x, y, z 범위와 현재 노드의 포인트를 고려하여 최솟값과 최댓값을 결정
        tmp_range_x[0] = min(right_son_ptr->node_range_x[0], root->point.x);
        tmp_range_x[1] = max(right_son_ptr->node_range_x[1], root->point.x);
        tmp_range_y[0] = min(right_son_ptr->node_range_y[0], root->point.y);
        tmp_range_y[1] = max(right_son_ptr->node_range_y[1], root->point.y);
        tmp_range_z[0] = min(right_son_ptr->node_range_z[0], root->point.z);
        tmp_range_z[1] = max(right_son_ptr->node_range_z[1], root->point.z);
        
        }
    else {
    // 오른쪽 자식 노드의 트리가 삭제되지 않은 경우, 다음을 수행
    if (!right_son_ptr->tree_deleted) {
        // 임시 범위의 x, y 및 z 좌표에 대한 최솟값 및 최댓값을 업데이트:
        // 오른쪽 자식 노드의 x, y, z 범위를 고려하여 최솟값과 최댓값을 결정
        tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
        tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
        tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
        tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
        tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
        tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
    }
    
    // 현재 노드의 포인트가 삭제되지 않은 경우, 다음을 수행
    if (!root->point_deleted) {
        // 임시 범위의 x, y 및 z 좌표에 대한 최솟값 및 최댓값을 업데이트:
        // 현재 노드의 포인트의 x, y, z 좌표를 고려하여 최솟값과 최댓값을 결정
        tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
        tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
        tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
        tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
        tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
        tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
    }            
}

    } else {
    // 이 경우는 오른쪽 자식 노드가 없을 때 해당됩니다.

    // 현재 노드의 트리 크기를 1로 설정합니다.
    root->TreeSize = 1;

    // 현재 노드의 무효화된 포인트 수를 설정합니다:
    // 현재 노드가 삭제된 경우 1, 그렇지 않으면 0
    root->invalid_point_num = (root->point_deleted ? 1 : 0);

    // 현재 노드의 다운샘플 삭제된 포인트 수를 설정합니다:
    // 현재 노드가 다운샘플 삭제된 경우 1, 그렇지 않으면 0
    root->down_del_num = (root->point_downsample_deleted ? 1 : 0);

    // 현재 노드의 트리 다운샘플 삭제된 플래그를 설정합니다.
    // 현재 노드의 포인트 다운샘플 삭제된 플래그와 동일합니다.
    root->tree_downsample_deleted = root->point_downsample_deleted;

    // 현재 노드의 트리 삭제된 플래그를 설정합니다.
    // 현재 노드의 포인트 삭제된 플래그와 동일합니다.
    root->tree_deleted = root->point_deleted;

    // 임시 범위의 x, y, z 좌표를 현재 노드의 포인트의 좌표로 설정합니다.
    tmp_range_x[0] = root->point.x;
    tmp_range_x[1] = root->point.x;
    tmp_range_y[0] = root->point.y;
    tmp_range_y[1] = root->point.y;
    tmp_range_z[0] = root->point.z;
    tmp_range_z[1] = root->point.z;
}


   // 임시 범위 정보를 루트 노드의 범위 정보에 복사합니다.
    memcpy(root->node_range_x, tmp_range_x, sizeof(tmp_range_x));
    memcpy(root->node_range_y, tmp_range_y, sizeof(tmp_range_y));
    memcpy(root->node_range_z, tmp_range_z, sizeof(tmp_range_z));
    
    // x, y, z 축의 길이의 반을 계산하여 루트 노드의 반지름 제곱을 계산합니다.
    float x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    float y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
    float z_L = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
    root->radius_sq = x_L * x_L + y_L * y_L + z_L * z_L;
    
    // 왼쪽 자식 노드와 오른쪽 자식 노드의 부모 포인터를 현재 루트 노드로 설정합니다.
    if (left_son_ptr != nullptr) left_son_ptr->father_ptr = root;
    if (right_son_ptr != nullptr) right_son_ptr->father_ptr = root;
    
    // 루트 노드가 최상위 루트 노드이고 트리 크기가 3 이상인 경우 다음을 수행합니다.
    if (root == Root_Node && root->TreeSize > 3) {
        // 왼쪽 또는 오른쪽 자식 노드 중 하나를 선택합니다.
        KD_TREE_NODE *son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
    
        // 루트 노드의 삭제 비율을 계산합니다.
        root->alpha_del = float(root->invalid_point_num) / root->TreeSize;
    
        // 루트 노드의 균형 비율을 계산합니다.
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize - 1);
        root->alpha_bal = (tmp_bal >= 0.5 - EPSS) ? tmp_bal : 1 - tmp_bal;
    }
    
    // 함수 종료
    return;
}

template <typename PointType>
void KD_TREE<PointType>::flatten(KD_TREE_NODE * root, PointVector &Storage, delete_point_storage_set storage_type){
    // KD 트리를 펼쳐서 포인트를 저장하는 함수입니다. 이 함수는 재귀적으로 호출됩니다.
    if (root == nullptr) return;
    // 만약 현재 노드가 널 포인터인 경우, 함수를 종료하고 아무 작업도 수행하지 않습니다.
    Push_Down(root);
    // 현재 노드의 자식 노드들을 푸시 다운합니다. (Push_Down 함수는 여기에 포함되어 있지 않아서 자세한 내용은 알 수 없습니다.)

    if (!root->point_deleted) {
        // 현재 노드의 포인트가 삭제되지 않은 경우,
        Storage.push_back(root->point);
        // 현재 노드의 포인트를 Storage 벡터에 추가합니다.
    }

    flatten(root->left_son_ptr, Storage, storage_type);
    // 현재 노드의 왼쪽 자식 노드를 펼치기 위해 재귀적으로 호출합니다.
    flatten(root->right_son_ptr, Storage, storage_type);
    // 현재 노드의 오른쪽 자식 노드를 펼치기 위해 재귀적으로 호출합니다.

    switch (storage_type)
    {
    case NOT_RECORD:
        // 저장 타입이 NOT_RECORD인 경우 아무 작업도 하지 않습니다.
        break;
    case DELETE_POINTS_REC:
        if (root->point_deleted && !root->point_downsample_deleted) {
            // 현재 노드의 포인트가 삭제되었지만 다운샘플링 삭제되지 않은 경우,
            Points_deleted.push_back(root->point);
            // Points_deleted 벡터에 현재 노드의 포인트를 추가합니다.
        }       
        break;
    case MULTI_THREAD_REC:
        if (root->point_deleted  && !root->point_downsample_deleted) {
            // 현재 노드의 포인트가 삭제되었지만 다운샘플링 삭제되지 않은 경우,
            Multithread_Points_deleted.push_back(root->point);
            // Multithread_Points_deleted 벡터에 현재 노드의 포인트를 추가합니다.
        }
        break;
    default:
        break;
    }     
    return;
}

template <typename PointType>
void KD_TREE<PointType>::delete_tree_nodes(KD_TREE_NODE ** root){ 
    // KD 트리의 노드들을 삭제하는 함수입니다. 이 함수는 재귀적으로 호출됩니다.
    if (*root == nullptr) return;
    // 만약 현재 노드가 널 포인터인 경우, 함수를 종료하고 아무 작업도 수행하지 않습니다.
    Push_Down(*root);
    // 현재 노드의 자식 노드들을 푸시 다운합니다. (Push_Down 함수는 여기에 포함되어 있지 않아서 자세한 내용은 알 수 없습니다.)
    delete_tree_nodes(&(*root)->left_son_ptr);
    // 현재 노드의 왼쪽 자식 노드를 삭제하기 위해 재귀적으로 호출합니다.
    delete_tree_nodes(&(*root)->right_son_ptr);
    // 현재 노드의 오른쪽 자식 노드를 삭제하기 위해 재귀적으로 호출합니다.
    pthread_mutex_destroy( &(*root)->push_down_mutex_lock);
    // 현재 노드의 푸시 다운 뮤텍스 락을 파괴합니다. (뮤텍스는 다중 스레드 환경에서 상호 배제를 제공하는데 사용됩니다.)
    delete *root;
    // 현재 노드를 삭제합니다.
    *root = nullptr;
    // 현재 노드 포인터를 널 포인터로 설정하여 노드의 메모리를 해제합니다.
    return;
}


template <typename PointType>
bool KD_TREE<PointType>::same_point(PointType a, PointType b){
    // 두 포인트 a와 b가 같은지 확인하는 함수입니다.

    return (fabs(a.x - b.x) < EPSS && fabs(a.y - b.y) < EPSS && fabs(a.z - b.z) < EPSS );
    // 두 포인트의 X, Y, Z 좌표가 각각 아주 작은 오차 (EPSS) 내에 있으면 true를 반환합니다.
}

template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a, PointType b){
    // 두 포인트 a와 b 사이의 거리를 계산하는 함수입니다.

    float dist = 0.0f;
    // 거리를 저장할 변수를 초기화합니다.
    
    dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    // 두 포인트 간의 제곱 거리를 계산합니다.

    return dist;
    // 계산된 거리를 반환합니다.
}

template <typename PointType>
float KD_TREE<PointType>::calc_box_dist(KD_TREE_NODE * node, PointType point){
    if (node == nullptr) return INFINITY;
    // 노드가 널 포인터인 경우, 무한대(INFINITY)를 반환합니다.

    float min_dist = 0.0;
    // 최소 거리를 저장하는 변수를 초기화합니다.

    if (point.x < node->node_range_x[0]) min_dist += (point.x - node->node_range_x[0]) * (point.x - node->node_range_x[0]);
    // 포인트의 X 좌표가 노드의 X 범위의 최소값보다 작은 경우, 해당 차이의 제곱을 최소 거리에 더합니다.
    if (point.x > node->node_range_x[1]) min_dist += (point.x - node->node_range_x[1]) * (point.x - node->node_range_x[1]);
    // 포인트의 X 좌표가 노드의 X 범위의 최대값보다 큰 경우, 해당 차이의 제곱을 최소 거리에 더합니다.

    if (point.y < node->node_range_y[0]) min_dist += (point.y - node->node_range_y[0]) * (point.y - node->node_range_y[0]);
    // 포인트의 Y 좌표가 노드의 Y 범위의 최소값보다 작은 경우, 해당 차이의 제곱을 최소 거리에 더합니다.
    if (point.y > node->node_range_y[1]) min_dist += (point.y - node->node_range_y[1]) * (point.y - node->node_range_y[1]);
    // 포인트의 Y 좌표가 노드의 Y 범위의 최대값보다 큰 경우, 해당 차이의 제곱을 최소 거리에 더합니다.

    if (point.z < node->node_range_z[0]) min_dist += (point.z - node->node_range_z[0]) * (point.z - node->node_range_z[0]);
    // 포인트의 Z 좌표가 노드의 Z 범위의 최소값보다 작은 경우, 해당 차이의 제곱을 최소 거리에 더합니다.
    if (point.z > node->node_range_z[1]) min_dist += (point.z - node->node_range_z[1]) * (point.z - node->node_range_z[1]);
    // 포인트의 Z 좌표가 노드의 Z 범위의 최대값보다 큰 경우, 해당 차이의 제곱을 최소 거리에 더합니다.

    return min_dist;
    // 최소 거리를 반환합니다.
}

template <typename PointType>
bool KD_TREE<PointType>::point_cmp_x(PointType a, PointType b) { return a.x < b.x; }
// X 좌표를 기준으로 포인트 a와 포인트 b를 비교하는 함수입니다.

template <typename PointType>
bool KD_TREE<PointType>::point_cmp_y(PointType a, PointType b) { return a.y < b.y; }
// Y 좌표를 기준으로 포인트 a와 포인트 b를 비교하는 함수입니다.

template <typename PointType>
bool KD_TREE<PointType>::point_cmp_z(PointType a, PointType b) { return a.z < b.z; }
// Z 좌표를 기준으로 포인트 a와 포인트 b를 비교하는 함수입니다.

template <typename T>
void MANUAL_Q<T>::clear(){
    head = 0;
    // 헤드 포인터를 0으로 설정합니다.
    tail = 0;
    // 테일 포인터를 0으로 설정합니다.
    counter = 0;
    // 큐의 요소 수를 0으로 설정합니다.
    is_empty = true;
    // 큐가 비어있음을 나타내는 플래그를 true로 설정합니다.
    return;
}

template <typename T>
void MANUAL_Q<T>::pop(){
    if (counter == 0) return;
    // 큐가 비어있으면 아무 작업도 하지 않고 함수를 종료합니다.

    head ++;
    // 헤드 포인터를 증가시킵니다.
    head %= Q_LEN;
    // 헤드 포인터가 큐 길이를 초과할 경우 원형 큐로 인덱스를 조정합니다.
    counter --;
    // 큐의 요소 수를 하나 감소시킵니다.

    if (counter == 0) is_empty = true;
    // 큐가 비어있으면 is_empty 플래그를 설정합니다.
    return;
}

template <typename T>
T MANUAL_Q<T>::front(){
    return q[head];
    // 큐의 맨 앞 요소를 반환합니다.
}

template <typename T>
T MANUAL_Q<T>::back(){
    return q[tail];
    // 큐의 맨 뒤 요소를 반환합니다.
}

template <typename T>
void MANUAL_Q<T>::push(T op){
    q[tail] = op;
    // 테일 포인터가 가리키는 위치에 요소를 추가합니다.
    counter ++;
    // 큐의 요소 수를 증가시킵니다.

    if (is_empty) is_empty = false;
    // 큐가 비어있는 경우 is_empty 플래그를 설정해줍니다.
    tail ++;
    // 테일 포인터를 증가시킵니다.
    tail %= Q_LEN;
    // 테일 포인터가 큐 길이를 초과할 경우 원형 큐로 인덱스를 조정합니다.
}



template <typename T>
bool MANUAL_Q<T>::empty(){
    return is_empty;
    // 큐가 비어있는지 여부를 반환합니다.
}

template <typename T>
int MANUAL_Q<T>::size(){
    return counter;
    // 큐에 저장된 요소의 수를 반환합니다.
}

template class KD_TREE<ikdTree_PointType>;
// ikdTree_PointType을 사용하여 KD_TREE 클래스를 인스턴스화합니다.

template class KD_TREE<pcl::PointXYZ>;
// pcl::PointXYZ를 사용하여 KD_TREE 클래스를 인스턴스화합니다.

template class KD_TREE<pcl::PointXYZI>;
// pcl::PointXYZI를 사용하여 KD_TREE 클래스를 인스턴스화합니다.

template class KD_TREE<pcl::PointXYZINormal>;
// pcl::PointXYZINormal을 사용하여 KD_TREE 클래스를 인스턴스화합니다.
