//
// Created by Robotics_qi on 2020/1/3.
//

#include <iostream>
#include <thread>

class Vehicle{
public:
    Vehicle(int id){
        _id = id;
    }

    void operator()(){
        std::cout << "Vehicle #" << _id <<  " has been created!" << std::endl;
    }

private:
    int _id;
};

int main()
{
    // Create thread.
    std::thread t2 = std::thread(Vehicle(1));    // Use copy initialization, Please pay attention after C++11 standard can use this.

    // do something in main.
    std::cout << "Finish work in main!" << std::endl;

    // Wait for thread to finish.
    t2.join();
}