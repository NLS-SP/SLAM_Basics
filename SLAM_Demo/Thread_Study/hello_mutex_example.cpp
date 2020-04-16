//
// Created by Robotics_qi on 2020/3/21.
//

#include <iostream>
#include <thread>
#include <mutex>

volatile int counter(0);    // non-atomic counter
std::mutex mtx;

void attempt_10K_increases(){
    for(int i = 0; i < 10000; ++i){
        if(mtx.try_lock()){
            ++counter;
            mtx.unlock();
        }
    }
}

int main(int argc, const char* argv[])
{
    std::thread threads[10];
    for(int i = 0; i < 10; ++i)
        threads[i] = std::thread(attempt_10K_increases);

    for(auto& th : threads) th.join();
    std::cout << counter << "successful increases of the counter. " << std::endl;

    return 0;
}
