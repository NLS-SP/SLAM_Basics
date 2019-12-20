//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_SELECTIONSORT_H
#define BASIC_ALGORITHM_VECTOR_SELECTIONSORT_H

template<typename T>
void Vector<T>::selectionSort(Rank lo, Rank hi){
    while(lo < --hi)
        swap(_elem[max(lo, hi)], _elem[hi]);
}

template<typename T>
Rank Vector<T>::max(Rank lo, Rank hi){
    Rank mx = hi;
    while(lo < hi--)
        if(_elem[hi] > _elem[mx])
            mx = hi;
    return mx;
}

#endif //BASIC_ALGORITHM_VECTOR_SELECTIONSORT_H
