#ifndef WBC_TOOLS_FILTER_HPP
#define WBC_TOOLS_FILTER_HPP  

#include <vector>
#include <Eigen/Core>

namespace wbc {

class MovingMedianFilter{
protected:
    size_t window_size_;
    Eigen::VectorXd values_;
    Eigen::VectorXd sorted_values_;
public:
    MovingMedianFilter(){}
    MovingMedianFilter(size_t window_size) : window_size_(window_size) {}

    double apply(double new_value);

};

class MovingAverageFilter{
protected:
    size_t window_size_;
    Eigen::VectorXd values_;
public:
    MovingAverageFilter(){}
    MovingAverageFilter(size_t window_size) : window_size_(window_size) {}

    double apply(double new_value);

};

}

#endif  // WBC_TOOLS_FILTER_HPP