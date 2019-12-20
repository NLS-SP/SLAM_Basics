//
// Created by gatsby on 2019-05-13.
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