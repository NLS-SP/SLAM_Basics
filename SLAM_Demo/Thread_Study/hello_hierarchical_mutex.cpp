//
// Created by Robotics_qi on 2020/2/5.
//

#include <mutex>
#include <stdexcept>

class hierarchical_mutex{
    std::mutex internal_mutex;
    unsigned long const hierarchy_value;
    unsigned long previous_hierarchical_value;
    static thread_local unsigned long this_thread_hierarchy_value;
};