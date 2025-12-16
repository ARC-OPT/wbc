#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <tools/filter.hpp>
#include <tools/Logger.hpp>
#include <chrono>
#include <fstream>

loglevel_e loglevel = logWARNING;

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(test_moving_median_filter){
    MovingMedianFilter filter(5);
    BOOST_CHECK_EQUAL(filter.apply(5.0), 5.0);
    BOOST_CHECK_EQUAL(filter.apply(9.0), 7.0);
    BOOST_CHECK_EQUAL(filter.apply(3.0), 5.0);
    BOOST_CHECK_EQUAL(filter.apply(4.0), 4.5);
    BOOST_CHECK_EQUAL(filter.apply(1.0), 4.0);
}

BOOST_AUTO_TEST_CASE(test_multi_dim){

    const int N_JOINT = 20;
    const int WINDOWSIZE = 50;

    std::vector<MovingAverageFilter> filters;
    for(int i = 0; i < N_JOINT; i++)
        filters.push_back(MovingAverageFilter(WINDOWSIZE));
    
    ofstream myfile;
    myfile.open ("filter_benchmark.txt");

    double avg_time = 0;
    int count = 0;
    for(double t = 0; t < 2*M_PI; t+=0.001){
        Eigen::VectorXd input = Eigen::VectorXd::Random(N_JOINT) / 10.0;
        for(int j = 0; j < N_JOINT; j++)
            input[j]+=sin(t);
        if(count % 1000 == 0)
            input += Eigen::VectorXd::Random(N_JOINT);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        for(int j = 0; j < N_JOINT; j++){
            double res = filters[j].apply(input[j]);
            myfile <<input[j]<<" "<<res<<" ";
        }
        myfile << sin(t);
        myfile << std::endl;        
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        avg_time += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        count++;
    }
    avg_time /= count;
    std::cout<<"Avg time: "<<avg_time<<" [us]"<<std::endl;
}
