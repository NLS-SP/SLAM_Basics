//
// Created by Robotics_qi on 2020/3/21.
//

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

std::timed_mutex mtx;

void fireworks(){
    // waiting to get a lock: each thread prints "-" every 200ms.
    while(!mtx.try_lock_for(std::chrono::milliseconds(200))){
        std::cout << "-";
    }

    // got a lock - wait for 1s, then this thread prints "*"
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "*\n";
    mtx.unlock();
}

int main()
{
    std::thread threads[10];

    // spawn 10 threads.
    for(int i = 0; i < 10; ++i)
        threads[i] = std::thread(fireworks);

    for(auto& th : threads) th.join();

    return 0;
}
