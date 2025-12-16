#ifndef WBC_TOOLS_FILTER_HPP
#define WBC_TOOLS_FILTER_HPP  

#include <vector>
#include <Eigen/Core>

namespace wbc {

class Filter{
protected:
    size_t window_size_;
public:
    Filter(size_t window_size) : window_size_(window_size) {}
    virtual ~Filter() {}
    virtual double apply(double new_value) = 0;
};

class MovingMedianFilter : public Filter {
protected:
    Eigen::VectorXd values_;
    Eigen::VectorXd sorted_values_;
public:
    MovingMedianFilter(size_t window_size) : Filter(window_size) {}

    virtual double apply(double new_value);

};

class MovingAverageFilter : public Filter {
protected:
    Eigen::VectorXd values_;
public:
    MovingAverageFilter(size_t window_size) : Filter(window_size) {}

    virtual double apply(double new_value);

};

}

#endif  // WBC_TOOLS_FILTER_HPP