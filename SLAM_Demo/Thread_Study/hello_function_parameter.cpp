//
// Created by Robotics_qi on 2020/1/3.
//

#include <iostream>
#include <string>
#include <thread>

void printID(int id){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "ID = " << id << std::endl;
}

void printIdAndName(int id, std::string name){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "ID = " << id << ", name = " << name << std::endl;
}

int main()
{
    int id = 0;         // Define an integer variable.

    // Starting threads using variadic templates.
    std::thread t1(printID, id);
    std::thread t2(printIdAndName, ++id, "MyString");

    // Wait for threads before returning
    t1.join();
    t2.join();

    return 0;
}