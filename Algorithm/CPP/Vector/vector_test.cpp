//
// Created by Robotics_qi on 2019-11-15.
//
#include <iostream>
#include "Vector.h"
#include <random>

int main()
{
    Vector<int> myVector;
    myVector.insert(myVector.size(), 1);myVector.insert(myVector.size(), 9);myVector.insert(myVector.size(), 5);
    myVector.insert(myVector.size(), 3);myVector.insert(myVector.size(), 27);myVector.insert(myVector.size(), 13);
    myVector.insert(myVector.size(), 23);

    for(int i = 0; i < myVector.size(); i++) std::cout << "Before, The vector is: " << myVector[i] << std::endl;
    std::cout << "Now my Vector size is: " << myVector.size() << std::endl;
    myVector.sort();
    for(int i = 0; i < myVector.size(); i++) std::cout << "Now, The vector is: " << myVector[i] << std::endl ;

}