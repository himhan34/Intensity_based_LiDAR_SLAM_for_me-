// Modified from https://github.com/irapkaist/scancontext
// 위의 주석은 코드의 원본 출처를 나타냅니다. 코드를 수정한 버전임을 알려줍니다.

#include "Scancontext.h"
// "Scancontext.h" 헤더 파일을 포함하는 명령입니다. 이 헤더 파일은 필요한 라이브러리나 함수를 선언합니다.

void coreImportTest(void)
{
    cout << "scancontext lib is successfully imported." << endl;
    // "coreImportTest" 함수 정의. 이 함수는 라이브러리가 성공적으로 임포트되었음을 확인하는 메시지를 출력합니다.
} // coreImportTest

float rad2deg(float radians)
{
    return radians * 180.0 / M_PI;
    // 라디안 값을 받아서 해당 값을 도 단위로 변환하여 반환하는 함수입니다.
}

float deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
    // 도 단위 값을 받아서 해당 값을 라디안으로 변환하여 반환하는 함수입니다.
}

float xy2theta(const float &_x, const float &_y)
{
    if (_x >= 0 & _y >= 0)
        return (180 / M_PI) * atan(_y / _x);
    // x와 y가 양수인 경우, 아크탄젠트 값을 계산하고 이를 도 단위로 변환하여 반환합니다.

    if (_x < 0 & _y >= 0)
        return 180 - ((180 / M_PI) * atan(_y / (-_x)));
    // x는 음수이고 y는 양수인 경우, 아크탄젠트 값을 계산하고 이를 도 단위로 변환하여 180에서 빼서 반환합니다.

    if (_x < 0 & _y < 0)
        return 180 + ((180 / M_PI) * atan(_y / _x));
    // x와 y가 모두 음수인 경우, 아크탄젠트 값을 계산하고 이를 도 단위로 변환하여 180을 더해서 반환합니다.

    if (_x >= 0 & _y < 0)
        return 360 - ((180 / M_PI) * atan((-_y) / _x));
    // x는 양수이고 y는 음수인 경우, 아크탄젠트 값을 계산하고 이를 도 단위로 변환하여 360에서 빼서 반환합니다.
} // xy2theta

MatrixXd circshift(MatrixXd &_mat, int _num_shift)
{
    // _mat 행렬을 _num_shift 만큼 오른쪽으로 순환 이동시키는 함수입니다.
    assert(_num_shift >= 0);

    if (_num_shift == 0)
    {
        MatrixXd shifted_mat(_mat);
        return shifted_mat;
        // 만약 _num_shift가 0이면 _mat 행렬을 그대로 반환합니다.
    }

    MatrixXd shifted_mat = MatrixXd::Zero(_mat.rows(), _mat.cols());
    // 모든 요소가 0인 행렬을 생성합니다.

    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++)
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
        // _mat의 열을 _num_shift만큼 오른쪽으로 이동한 위치에 복사합니다.
    }

    return shifted_mat;
    // 순환 이동이 끝난 행렬을 반환합니다.
} // circshift


std::vector<float> eig2stdvec(MatrixXd _eigmat)
{
    // MatrixXd를 std::vector<float>으로 변환하는 함수입니다.
    std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
    return vec;
} // eig2stdvec


double SCManager::distDirectSC(MatrixXd &_sc1, MatrixXd &_sc2)
{
    int num_eff_cols = 0; // 0이 아닌 섹터의 개수를 저장할 변수를 초기화합니다.
    double sum_sector_similarity = 0; // 섹터 유사도의 합을 저장할 변수를 초기화합니다.

    for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++)
    {
        VectorXd col_sc1 = _sc1.col(col_idx); // _sc1의 열을 VectorXd로 가져옵니다.
        VectorXd col_sc2 = _sc2.col(col_idx); // _sc2의 열을 VectorXd로 가져옵니다.
        
        if (col_sc1.norm() == 0 || col_sc2.norm() == 0)
            continue; // 섹터의 크기가 0이면 이 섹터 쌍을 무시합니다.

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
        // 두 섹터의 유사도를 계산합니다.

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1; // 0이 아닌 섹터의 개수를 증가시킵니다.
    }

    double sc_sim = sum_sector_similarity / num_eff_cols; // 섹터 유사도의 평균을 계산합니다.
    return 1.0 - sc_sim; // 1에서 섹터 유사도 평균을 뺀 값을 반환합니다.
} // distDirectSC


int SCManager::fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2)
{
    int argmin_vkey_shift = 0; // 최소 벡터 시프트의 위치를 저장할 변수를 초기화합니다.
    double min_veky_diff_norm = 10000000; // 최소 벡터 차이의 노름을 저장할 변수를 초기화합니다.

    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx); // _vkey2를 시프트합니다.

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted; // 두 벡터의 차이를 계산합니다.

        double cur_diff_norm = vkey_diff.norm(); // 현재 벡터 차이의 노름을 계산합니다.
        if (cur_diff_norm < min_veky_diff_norm)
        {
            argmin_vkey_shift = shift_idx; // 최소 벡터 시프트 위치를 업데이트합니다.
            min_veky_diff_norm = cur_diff_norm; // 최소 벡터 차이의 노름을 업데이트합니다.
        }
    }

    return argmin_vkey_shift; // 최소 벡터 시프트 위치를 반환합니다.
} // fastAlignUsingVkey

std::pair<double, int> SCManager::distanceBtnScanContext(MatrixXd &_sc1, MatrixXd &_sc2)
{
    // 1. 빠른 벡터 키(Variant Key)를 사용한 정렬 (원래 IROS18에 없는 부분)
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1); // 첫 번째 스캔 콘텍스트의 벡터 키 생성
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2); // 두 번째 스캔 콘텍스트의 벡터 키 생성
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2); // 벡터 키 정렬을 수행

    const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols()); // 검색 범위의 반
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
    {
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. 빠른 열 단위 차이 계산
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for (int num_shift : shift_idx_search_space)
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift); // _sc2를 주어진 위치로 시프트
        double cur_sc_dist = distDirectSC(_sc1, sc2_shifted); // 스캔 콘텍스트 간 직접 거리 계산
        if (cur_sc_dist < min_sc_dist)
        {
            argmin_shift = num_shift; // 최소 거리를 가진 시프트 위치 업데이트
            min_sc_dist = cur_sc_dist; // 최소 스캔 콘텍스트 거리 업데이트
        }
    }

    return std::make_pair(min_sc_dist, argmin_shift);
} // distanceBtnScanContext


MatrixXd SCManager::makeScancontext(pcl::PointCloud<SCPointType> & _scan_down)
{
    TicToc_SC t_making_desc;

    int num_pts_scan_down = _scan_down.points.size();

    // 주요 부분
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // 2D 평면 내에서
    int ring_idx, sector_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // 간단한 덧셈 (모든 포인트는 0보다 커야 함).

        // x, y 좌표를 각도와 반지름으로 변환
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // 범위가 ROI를 벗어나면 건너뜁니다.
        if (azim_range > PC_MAX_RADIUS)
            continue;

        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        sector_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

        // 최대 z 값을 취합니다.
        if (desc(ring_idx - 1, sector_idx - 1) < pt.z) // -1은 C++이 0부터 시작하기 때문입니다.
            desc(ring_idx - 1, sector_idx - 1) = pt.z; // 해당 버킷에서 최대 값을 업데이트합니다.
    }

    // 포인트가 없는 곳을 0으로 재설정 (나중에 코사인 거리를 위해)
    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
            if (desc(row_idx, col_idx) == NO_POINT)
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext 만들기");

    return desc;
} // SCManager::makeScancontext

MatrixXd SCManager::makeRingkeyFromScancontext(Eigen::MatrixXd &_desc)
{
    /* 
     * 요약: 행별 평균 벡터 생성
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for (int row_idx = 0; row_idx < _desc.rows(); row_idx++)
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx); // 현재 행 가져오기
        invariant_key(row_idx, 0) = curr_row.mean(); // 현재 행의 평균을 인베리언트 키에 저장
    }

    return invariant_key; // 인베리언트 키 반환
} // SCManager::makeRingkeyFromScancontext


MatrixXd SCManager::makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc)
{
    /* 
     * 요약: 열별 평균 벡터 생성
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++)
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx); // 현재 열 가져오기
        variant_key(0, col_idx) = curr_col.mean(); // 현재 열의 평균을 베리언트 키에 저장
    }

    return variant_key; // 베리언트 키 반환
} // SCManager::makeSectorkeyFromScancontext

void SCManager::makeAndSaveScancontextAndKeys(pcl::PointCloud<SCPointType> &_scan_down)
{
    Eigen::MatrixXd sc = makeScancontext(_scan_down); // 스캔 콘텍스트 생성
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc); // 스캔 콘텍스트로부터 링 키 생성
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc); // 스캔 콘텍스트로부터 섹터 키 생성
    std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey); // 링 키를 벡터로 변환

    polarcontexts_.push_back(sc); // 스캔 콘텍스트 저장
    polarcontext_invkeys_.push_back(ringkey); // 링 키 저장
    polarcontext_vkeys_.push_back(sectorkey); // 섹터 키 저장
    polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec); // 링 키를 벡터로 저장

    // cout << polarcontext_vkeys_.size() << endl;

} // SCManager::makeAndSaveScancontextAndKeys

std::pair<int, float> SCManager::detectLoopClosureID(void)
{
    int loop_id { -1 }; // 루프 클로저 ID를 초기화합니다. -1은 루프가 없음을 나타냅니다. (LeGO-LOAM의 "closestHistoryFrameID" 변수와 동일)

    auto curr_key = polarcontext_invkeys_mat_.back(); // 현재 관측 (쿼리)
    auto curr_desc = polarcontexts_.back(); // 현재 관측 (쿼리)

    /* 
     * 단계 1: 링 키 트리에서 후보를 선택합니다.
     */
    if (polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result { loop_id, 0.0 };
        return result; // 이전 관측 데이터가 부족하면 루프 클로저가 없음을 반환합니다.
    }

    // 트리 재구성 (필수적으로 매번 수행할 필요는 없습니다)
    if (tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // 연산 비용을 절약하기 위한 조건
    {
        TicToc_SC t_tree_construction;

        polarcontext_invkeys_to_search_.clear();
        polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT);

        polarcontext_tree_.reset();
        polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* 차원 */, polarcontext_invkeys_to_search_, 10 /* 최대 리프 노드 개수 */);
        // tree_ptr_->index->buildIndex(); // 내부적으로 InvKeyTree 생성자에서 호출됨 (자세한 내용은 nanoflann 및 KDtreeVectorOfVectorsAdaptor 참조)
        t_tree_construction.toc("tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;
        
    double min_dist = 10000000; // 큰 값으로 초기화
    int nn_align = 0;
    int nn_idx = 0;

    // knn 검색
    std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE); 
    std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);

    TicToc_SC t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
    knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0] /* 쿼리 */, nanoflann::SearchParams(10)); 
    t_tree_search.toc("tree search");

/* 
 * 단계 2: 쌍별 거리 계산 (코사인 거리를 사용하여 최적의 열별 맞춤을 찾음)
 */
TicToc_SC t_calc_dist;   
for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
{
    MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]]; // 후보 스캔 콘텍스트 가져오기
    std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate); // 스캔 콘텍스트 간 거리 계산

    double candidate_dist = sc_dist_result.first; // 후보 거리
    int candidate_align = sc_dist_result.second; // 후보 정렬

    if (candidate_dist < min_dist)
    {
        min_dist = candidate_dist; // 최소 거리 업데이트
        nn_align = candidate_align; // 최적 정렬 업데이트

        nn_idx = candidate_indexes[candidate_iter_idx]; // 최적 후보 인덱스 업데이트
    }
}
t_calc_dist.toc("거리 계산");

/* 
 * 루프 임계값 확인
 */
if (min_dist < SC_DIST_THRES)
{
    loop_id = nn_idx; // 루프 클로저 ID 업데이트

    // std::cout.precision(3); 
    cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
    cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
}
else
{
    std::cout.precision(3); 
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
}

// To do: nn_align (즉, Yaw 차이)도 반환합니다.
float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE); // Yaw 차이를 라디안으로 변환
std::pair<int, float> result {loop_id, yaw_diff_rad}; // 루프 클로저 ID와 Yaw 차이를 반환하는 결과 생성

return result; // 결과 반환

} // SCManager::detectLoopClosureID

// } // namespace SC2


