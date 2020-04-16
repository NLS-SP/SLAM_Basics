//
// Created by Robotics_qi on 2020/3/17.
//

#include <future>
#include <iostream>

int find_the_answer_to_ltuae(){
    int i = 0;
    while(i < 990)
        i++;
    return i;
}

void do_other_stuff(int i){
    std::cout << "Now, the number i is: " << i << std::endl;
}

int main()
{
    std::future<int> the_answer = std::async(find_the_answer_to_ltuae);
    for(int i = 0; i < 99; ++i) {
        do_other_stuff(i);
    }
    std::cout << "The answer is: " << the_answer.get() << std::endl;
}