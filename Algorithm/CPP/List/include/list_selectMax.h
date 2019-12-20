//
// Created by Robotics_qi on 2019-11-23.
//

#ifndef BASIC_ALGORITHM_LIST_SELECTMAX_H
#define BASIC_ALGORITHM_LIST_SELECTMAX_H

template<typename T>
listnodePosi(T) List<T>::selectMax(listnodePosi(T) p, int n){
    listnodePosi(T) max = p;
    for(listnodePosi(T) cur = p; 1 < n; n--)
        if((cur = cur->succ)->data > max->data)
            max = cur;

    return max;
}

#endif //BASIC_ALGORITHM_LIST_SELECTMAX_H
