// 중복 포함을 방지하기 위한 전처리기 지시문
#pragma once

// Ceres 라이브러리를 위한 헤더 파일 포함. 비선형 최적화 문제를 해결하는 데 사용됩니다.
#include <ceres/ceres.h>

// Ceres에서 제공하는 회전에 대한 헤더 파일
#include <ceres/rotation.h>

// Eigen 라이브러리를 위한 헤더 파일 포함. 행렬 및 벡터 연산을 위해 사용됩니다.
#include <eigen3/Eigen/Dense>

// PCL 라이브러리를 위한 헤더 파일 포함. 포인트 클라우드 처리를 위해 사용됩니다.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL과 ROS 간의 변환을 위한 헤더 파일 포함.
#include <pcl_conversions/pcl_conversions.h>

// 매치된 특징점에 대한 프론트 엔드 최적화를 위한 구조체
struct front_end_residual
{
    // 생성자: 소스 포인트와 목적지 포인트를 초기화
    front_end_residual(Eigen::Vector3d src_point_, Eigen::Vector3d dst_point_) 
						        : src_point(src_point_), dst_point(dst_point_){}

    // 연산자 오버로딩을 통한 최적화 함수
    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// 소스에서 목적지까지의 회전을 나타내는 쿼터니언 생성
		Eigen::Quaternion<T> q_src2dest{q[3], q[0], q[1], q[2]};
		// 소스에서 목적지까지의 변환을 나타내는 벡터 생성
		Eigen::Matrix<T, 3, 1> t_src2dest{t[0], t[1], t[2]};
		// 소스 포인트를 T 타입으로 변환
		Eigen::Matrix<T, 3, 1> cp{T(src_point.x()), T(src_point.y()), T(src_point.z())};
		// 소스 포인트를 목적지 좌표계로 변환
		Eigen::Matrix<T, 3, 1> point_in_dist_coordinate_sys;
		point_in_dist_coordinate_sys = q_src2dest * cp + t_src2dest;

		// 잔차 계산
		residual[0] = point_in_dist_coordinate_sys.x() - T(dst_point.x());
		residual[1] = point_in_dist_coordinate_sys.y() - T(dst_point.y());
		residual[2] = point_in_dist_coordinate_sys.z() - T(dst_point.z());
		return true;
	}

    // Cost Function 생성을 위한 정적 메소드
    static ceres::CostFunction *Create(const Eigen::Vector3d src_point_, const Eigen::Vector3d dst_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				front_end_residual, 3, 4, 3>(
			new front_end_residual(src_point_, dst_point_)));
	}

    // 소스 포인트와 목적지 포인트를 저장하는 변수
    Eigen::Vector3d src_point, dst_point;
};

// 특징점 매칭에 대한 잔차 구조체 정의
struct FeatureMatchingResidual
{
    // 생성자: 현재 포인트와 이전 포인트를 초기화
    FeatureMatchingResidual(Eigen::Vector3d curr_point_, Eigen::Vector3d prev_point_) 
						        : curr_point(curr_point_), prev_point(prev_point_){}

    // 연산자 오버로딩을 통한 최적화 함수
    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// 현재 좌표계에서 세계 좌표계로의 회전을 나타내는 쿼터니언 생성
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		// 현재 좌표계에서 세계 좌표계로의 변환을 나타내는 벡터 생성
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		// 현재 포인트를 T 타입으로 변환
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		// 세계 좌표계에서의 포인트 위치 계산
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		// 잔차 계산
		residual[0] = point_w.x() - T(prev_point.x());
		residual[1] = point_w.y() - T(prev_point.y());
		residual[2] = point_w.z() - T(prev_point.z());
		return true;
	}

    // Cost Function 생성을 위한 정적 메소드
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d prev_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				FeatureMatchingResidual, 3, 4, 3>(
			new FeatureMatchingResidual(curr_point_, prev_point_)));
	}

    // 현재 포인트와 이전 포인트를 저장하는 변수
    Eigen::Vector3d curr_point, prev_point;
};

// LiDAR 지상 평면의 법선에 대한 요소 구조체 정의
struct LidarGroundPlaneNormFactor
{
    // 생성자: 현재 포인트, 평면 단위 법선, OA의 음수 점곱 초기화
    LidarGroundPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
                               double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
                                                               negative_OA_dot_norm(negative_OA_dot_norm_) {}

    // 연산자 오버로딩을 통한 최적화 함수
    template <typename T>
    bool operator()(const T *q, T *residual) const
    {
        // 현재 좌표계에서 세계 좌표계로의 회전을 나타내는 쿼터니언 생성
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        // 현재 포인트를 T 타입으로 변환
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        // 세계 좌표계에서의 포인트 위치 계산
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_w_curr * cp;

        // 평면 단위 법선을 T 타입으로 변환
        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
        // 잔차 계산
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
        return true;
    }

    // Cost Function 생성을 위한 정적 메소드
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarGroundPlaneNormFactor, 1, 4>(
            new LidarGroundPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
    }

    // 현재 포인트, 평면 단위 법선, OA의 음수 점곱을 저장하는 변수
    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
};

// LiDAR 평면 요소 구조체 정의
struct LidarPlaneFactor
{
    // 생성자: 현재 포인트와 이전 평면의 포인트들을 초기화, 평면의 법선 계산
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

    // 연산자 오버로딩을 통한 최적화 함수
	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
	    // 현재 포인트와 이전 평면의 포인트를 T 타입으로 변환
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};

		// 평면의 법선을 T 타입으로 변환
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		// 쿼터니언과 변환 벡터를 사용하여 현재 포인트를 이전 좌표계로 변환
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		// 변환된 포인트 위치 계산
		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		// 잔차 계산
		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

    // Cost Function 생성을 위한 정적 메소드
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

    // 현재 포인트, 이전 평면의 포인트, 평면의 법선, 비율을 저장하는 변수
	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

// LiDAR 평면 법선 요소 구조체 정의
struct LidarPlaneNormFactor
{
    // 생성자: 현재 포인트, 평면 단위 법선, OA의 음수 점곱 초기화
	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

    // 연산자 오버로딩을 통한 최적화 함수
	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// 현재 좌표계에서 세계 좌표계로의 회전을 나타내는 쿼터니언 생성
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		// 현재 좌표계에서 세계 좌표계로의 변환을 나타내는 벡터 생성
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		// 현재 포인트를 T 타입으로 변환
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		// 세계 좌표계에서의 포인트 위치 계산
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		// 평면 단위 법선을 T 타입으로 변환
		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		// 잔차 계산
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

    // Cost Function 생성을 위한 정적 메소드
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

    // 현재 포인트, 평면 단위 법선, OA의 음수 점곱을 저장하는 변수
	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};

// LiDAR 엣지(모서리) 요소 구조체 정의
struct LidarEdgeFactor
{
    // 생성자: 현재 포인트, 이전 두 포인트 및 비율 초기화
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

    // 연산자 오버로딩을 통한 최적화 함수
	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
        // 현재 포인트와 이전 두 포인트를 T 타입으로 변환
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

        // 쿼터니언과 변환 벡터를 사용하여 현재 포인트를 이전 좌표계로 변환
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

        // 변환된 포인트 위치 계산
		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

        // 두 포인트에 대한 벡터곱을 사용한 엣지의 법선 계산
		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

        // 잔차 계산
		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

    // Cost Function 생성을 위한 정적 메소드
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

    // 현재 포인트, 이전 두 포인트, 비율을 저장하는 변수
	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};
