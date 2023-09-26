#include "core.h"

class LoopTimer{
private:;
  double period_us;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
public:
  LoopTimer(int rate_hz){
    this->period_us = 1000000.0 / rate_hz;
  }
  void Start(){
    this->start_time = std::chrono::high_resolution_clock::now();
  }
  void PrintElapsed(){
    auto curr_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(curr_time - start_time).count();
    std::cout << "elased_time: " << elapsed_time_us << std::endl;
  }
  void Sleep() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    
    //std::cout << "loop elased_time: " << elapsed_time_us << std::endl;
    //std::cout << "sleep _time: " << this->period_us- elapsed_time_us << std::endl;
    if (elapsed_time_us < this->period_us) {
        std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<int>(this->period_us - elapsed_time_us))
        );
    }
  }
};