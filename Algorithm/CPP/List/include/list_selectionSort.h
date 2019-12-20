//
// Created by Robotics_qi on 2019-11-23.
//

#ifndef BASIC_ALGORITHM_LIST_SELECTIONSORT_H
#define BASIC_ALGORITHM_LIST_SELECTIONSORT_H

template<typename T>
void List<T>::selectionSort(listnodePosi(T) p, int n){
    listnodePosi(T) header = p->pred; listnodePosi(T) trail = p;
    for(int i = 0; i < n; i++) tail = tail->succ;
    while(1 < n){
        listnodePosi(T) max = selectMax(header->succ, n);
        insertB(tail, remove(max));
        tail = tail->pred;
        n--;
    }
}

#endif //BASIC_ALGORITHM_LIST_SELECTIONSORT_H
