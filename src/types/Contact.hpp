#ifndef WBC_TYPES_CONTACT_HPP
#define WBC_TYPES_CONTACT_HPP

#include <string>

namespace wbc{ namespace types{

/**
 * @brief Describes a rigid contact between a robot link and the environment. This can be either a point or surface contact
 */
class Contact{
public:
    Contact(){

    }
    Contact(std::string frame_id, int active, double mu) : frame_id(frame_id), active(active), mu(mu){

    }
    Contact(std::string frame_id, int active, double mu, double wx, double wy) : frame_id(frame_id), active(active), mu(mu), wx(wx), wy(wy){

    }
    std::string frame_id; /** ID of the link in contact */
    int active;           /** Is the link with ID frame_id currently in contact?*/
    double mu;            /** Friction coeffcient*/
    double wx;            /** x-dimension of the contact surface (only for surface contacts)*/
    double wy;            /** y-dimension of the contact surface (only for surface contacts)*/
};
}
}
#endif