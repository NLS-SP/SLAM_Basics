//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_BUBBLESORT_H
#define BASIC_ALGORITHM_VECTOR_BUBBLESORT_H

template<typename T>
void Vector<T>::bubbleSort(Rank lo, Rank hi){
    while(!bubble(lo, hi--));
}

#endif //BASIC_ALGORITHM_VECTOR_BUBBLESORT_H
