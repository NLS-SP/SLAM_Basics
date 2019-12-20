//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_BUBBLE_H
#define BASIC_ALGORITHM_VECTOR_BUBBLE_H

template<typename T>
bool Vector<T>::bubble(Rank lo, Rank hi){
    bool sorted = true;                                     // 整体有序标志
    while(++lo < hi)                                        // 自左向右,逐一检查各对相邻元素
        if(_elem[lo-1] > _elem[lo]){
            sorted = false;
            swap(_elem[lo-1], _elem[lo]);
        }
    return sorted;
}

#endif //BASIC_ALGORITHM_VECTOR_BUBBLE_H
