// from https://github.com/HKUST-Aerial-Robotics/VINS-Mono
#pragma once
// 한 번만 포함되도록 지시

#include <ctime>
// C 표준 라이브러리의 시간 기능을 포함시킴
#include <cstdlib>
// C 표준 라이브러리의 일반 유틸리티를 포함시킴
#include <chrono>
// C++ 표준 라이브러리의 시간 관련 기능을 포함시킴

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }
    // 생성자, tic 함수를 호출하여 시간 측정 시작

    void tic()
    {
        start = std::chrono::system_clock::now();
    }
    // 현재 시간을 기록하여 시간 측정 시작

    double toc()
    {
        end = std::chrono::system_clock::now();
        // 현재 시간을 기록하여 시간 측정 종료
        std::chrono::duration<double> elapsed_seconds = end - start;
        // 시작 시간과 종료 시간의 차이를 계산
        return elapsed_seconds.count() * 1000;
        // 경과 시간을 밀리초 단위로 반환
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    // 시간 측정을 위한 시작점과 종료점을 저장하는 변수
};
