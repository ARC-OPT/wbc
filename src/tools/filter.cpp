#include "filter.hpp"
#include <iostream>

namespace wbc {

double MovingMedianFilter::apply(double new_value) 
{
    if (values_.size() < window_size_)
        values_.conservativeResize(values_.size()+1);
    else
        values_.segment(0,window_size_-1) = values_.segment(1,window_size_-1);
    values_[values_.size()-1] = new_value;
    sorted_values_ = values_;

    std::sort(std::begin(sorted_values_), std::end(sorted_values_));
    size_t mid = sorted_values_.size() / 2;

    if (sorted_values_.size() % 2 == 0) {
        return (sorted_values_[mid - 1] + sorted_values_[mid]) / 2.0;
    } else {
        return sorted_values_[mid];
    }
}

double MovingAverageFilter::apply(double new_value) {    
    if (values_.size() < window_size_)
        values_.conservativeResize(values_.size()+1);
    else
        values_.segment(0,window_size_-1) = values_.segment(1,window_size_-1);
    values_[values_.size()-1] = new_value;
    return values_.mean();
}


}  // namespace wbc