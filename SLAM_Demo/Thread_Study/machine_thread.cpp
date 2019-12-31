//
// Created by gatsby on 2019-05-13.
//

#include <iostream>
#include <thread>

int main()
{
    std::cout << "Hello concurrent world from main! Thread id = "
              << std::this_thread::get_id() << std::endl;

    unsigned int nCores = std::thread::hardware_concurrency();
    std::cout << "This machine supports concurrency with " << nCores << " cores available." << std::endl;
}