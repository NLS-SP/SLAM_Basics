//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_FIND_H
#define BASIC_ALGORITHM_VECTOR_FIND_H

template<typename T>
Rank Vector<T>::find(T const& e, Rank lo, Rank hi) const{
    while((lo < hi--) && (e != _elem[hi]));                         // 从后向前，顺序查找
    return hi;
}

#endif //BASIC_ALGORITHM_VECTOR_FIND_H
