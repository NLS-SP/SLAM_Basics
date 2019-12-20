//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_A_H
#define BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_A_H

template<typename T>
static Rank binSearch(T* A, T const& e, Rank lo, Rank hi){
    std::cout << "This is the first binary search algorithm" << std::endl;
    while(lo < hi){
        Rank mi = (lo + hi) >> 1;
        if     (e < A[mi]) hi = mi;                          // 深入前半段[lo, mi)继续查找
        else if( A[mi] < e ) lo = mi + 1;                    // 深入后半段(mi, hi)继续查找
        else    return mi;                                   // 在mi处命中
    }
    return -1;
}

#endif //BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_A_H
