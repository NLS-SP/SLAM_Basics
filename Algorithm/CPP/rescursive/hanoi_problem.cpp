//
// Created by Robotics_qi on 2020/4/1.
//

#include <iostream>
using namespace std;

void hanoi(int N, char source, char relay, char destination){
    if(N == 1) std::cout << source << " --> " << destination << std::endl;
    else{
        hanoi(N-1, source, destination, relay);
        std::cout << source << " --> " << destination << std::endl;
        hanoi(N-1, relay, source, destination);
    }

}

int main()
{
    std::cout << "汉诺塔问题求解" << std::endl;
    hanoi(3, 'A', 'B', 'C');
    return 0;
}