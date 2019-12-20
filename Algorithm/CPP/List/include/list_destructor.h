//
// Created by Robotics_qi on 2019-11-21.
//

#ifndef BASIC_ALGORITHM_LIST_DESTRUCTOR_H
#define BASIC_ALGORITHM_LIST_DESTRUCTOR_H

template<typename T>
List<T>::~List(){
    clear();
    delete header;
    delete trailer;
}

#endif //BASIC_ALGORITHM_LIST_DESTRUCTOR_H
