//
// Created by Robotics_qi on 2020/1/3.
//

#include <iostream>
#include <thread>

void threadFunctionEven(){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::cout << "Even thread!" << std::endl;
}

void threadFunctionOdd(){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::cout << "Odd thread!" << std::endl;
}


int main()
{
    for(int i = 0; i < 6; ++i){
        if(i % 2 == 0){
            std::thread t(threadFunctionEven);
            t.detach();
        }else{
            std::thread t(threadFunctionOdd);
            t.detach();
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::cout << "End of main is reached!" << std::endl;
    return 0;
}