//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_B_H
#define BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_B_H

template<typename T>
static Rank binSearch(T* A, T const& e, Rank lo, Rank hi){
    std::cout << "The B version of Binary Search" << std::endl;
    while(hi - lo > 1){
        Rank mi = (lo + hi) >> 1;               // 以中点为轴点
        (A[mi] > e) ? hi = mi : lo = mi;
    }
    return (A[lo] == e) ? lo : -1;              // 查找成功返回对应的秩，否则统一返回-1
}

#endif //BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_B_H
