//
// Created by Robotics_qi on 2019-11-15.
//

#ifndef BASIC_ALGORITHM_VECTOR_EXPAND_H
#define BASIC_ALGORITHM_VECTOR_EXPAND_H

template<typename T>
void Vector<T>::expand(){
    if(_size < _capacity) return;                                   // 尚未满员时候，不必扩容
    if(_capacity < DEFAULT_CAPACITY) _capacity = DEFAULT_CAPACITY;  // 不低于最小容量
    T* oldElem = _elem; _elem = new T[_capacity <<= 1];             // 容量加倍
    for(int i = 0; i < _size; i++)
        _elem[i] = oldElem[i];
    delete []oldElem;
}

#endif //BASIC_ALGORITHM_VECTOR_EXPAND_H
