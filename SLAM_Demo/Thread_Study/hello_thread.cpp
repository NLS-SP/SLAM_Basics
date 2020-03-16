//
// Created by Robotics_qi on 2020/2/2.
//

#include <iostream>
#include <thread>

void hello(){
    std::cout << "Hello Concurrent World!" << std::endl;
}

int main()
{
    std::thread t(hello);
    t.join();
}