//
// Created by Robotics_qi on 2019/12/31.
//

#include <iostream>
#include <thread>

void threadFunction(){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Finish work 1 in thread" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Finish work 2 in thread" << std::endl;
}

int main()
{
    // create thread.
    std::thread t(threadFunction);
    t.join();

    // do something in main()
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // wait for thread to finish.
    std::cout << "Finish work 1 in main" << std::endl;


    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // wait for thread to finish.
    std::cout << "Finish work 2 in main" << std::endl;


    return 0;
}