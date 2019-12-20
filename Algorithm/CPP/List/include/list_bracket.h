//
// Created by Robotics_qi on 2019-11-21.
//

#ifndef BASIC_ALGORITHM_LIST_BRACKET_H
#define BASIC_ALGORITHM_LIST_BRACKET_H

template<typename T>
T& List<T>::operator[](Rank r) const{
    listnodePosi(T) p = first();
    while(0 < r--) p = p->succ;
    return p->data;
}

#endif //BASIC_ALGORITHM_LIST_BRACKET_H
