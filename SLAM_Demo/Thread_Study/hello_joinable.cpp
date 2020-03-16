//
// Created by Robotics_qi on 2020/2/2.
//

#include <iostream>
#include <thread>

void do_something(int &i){ ++i; }

struct func{
    int& i_;
    func(int& i) : i_(i){}

    void operator()(){
        for(unsigned j = 0; j < 10000000000; ++j) {
            do_something(i_);
            std::cout << "In Concurrent, The i is: " << i_ << std::endl;
        }
    }
};

void oops(){
    int some_local_state = 0;
    func my_func(some_local_state);
    std::thread my_thread(my_func);
    my_thread.detach();
}

int main()
{
    oops();
    for(int i = 0; i < 100000; i++);
}