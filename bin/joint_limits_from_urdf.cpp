#include <base-logging/Logging.hpp>
#include <tools/URDFTools.hpp>
#include <base/JointLimits.hpp>
#include <boost/program_options.hpp>

using namespace wbc;
using namespace std;
namespace po = boost::program_options;

int main(int argc, char *argv[]){
    argc--;
    argv++;

    bool as_yaml = false;

    // Declare the supported options.
    po::options_description desc("Usage: joint_limits_from_urdf <urdf_file> <options>. Possible options are");
    desc.add_options()
        ("help", "produce help message")
        ("yml", "print in yaml format")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }
    if(vm.count("yml"))
        as_yaml = true;

    if(argc < 1){
        cout << desc << "\n";
        return -1;
    }

    std::string filename = argv[0];
    base::JointLimits joint_limits;
    URDFTools::jointLimitsFromURDF(filename, joint_limits);

    if(as_yaml){
        printf("joint_limits:\n");
        printf("  names:\n");
        for(auto n : joint_limits.names) printf("    - %s\n", n.c_str());
        printf("  elements:\n");
        printf("    [\n");
        for(auto n : joint_limits.names){
            base::JointLimitRange r = joint_limits[n];
            printf("    {max: {position: %2.6f, speed: %2.6f, effort: %2.6f}, min: {position %2.6f}},\n", r.max.position, r.max.speed, r.max.effort, r.min.position);
        }
        printf("    ]\n");
    }
    else{
        for(auto n : joint_limits.names){
            printf("%25s: \tMax Pos: %2.6f \tMin Pos: %2.6f \tMax Vel: %2.6f \tMax Eff: %2.6f\n",
                   n.c_str(), joint_limits[n].max.position, joint_limits[n].min.position, joint_limits[n].max.speed, joint_limits[n].max.effort);
        }
    }

    return 0;
}
