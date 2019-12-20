//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_C_H
#define BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_C_H
template<typename T>
static Rank binSearch(T* A, T const& e, Rank lo, Rank hi){
    std::cout << "The C version of binary search " << std::endl;
    while(hi > lo){
        Rank mi = (lo + hi) >> 1;
        (A[mi] > e) ? hi = mi : lo = mi+1;          // 经比较后确定深入[lo, mi)或(mi, hi)
    }
    return --lo;
}
#endif //BASIC_ALGORITHM_VECTOR_SEARCH_BINARY_C_H
