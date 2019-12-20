//
// Created by Robotics_qi on 2019-11-21.
//

#ifndef BASIC_ALGORITHM_LIST_CLEAR_H
#define BASIC_ALGORITHM_LIST_CLEAR_H

template<typename T>
int List<T>::clear(){
    int oldSize = _size;
    while(0 < _size) remove(header->succ);
    return oldSize;
}

#endif //BASIC_ALGORITHM_LIST_CLEAR_H
