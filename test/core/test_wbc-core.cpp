#include <boost/test/unit_test.hpp>
#include "core/Jacobian.hpp"
#include <kdl/jacobian.hpp>
#include <iostream>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(test_jacobian)
{
    cout<<"Testing Jacobian ...."<<endl<<endl;

    srand(time(NULL));

    cout<<"Testing change Ref point ..."<<endl<<endl;

    Jacobian jac(7);
    jac.setIdentity();
    jac.changeRefPoint(base::Vector3d(0.1,0.2,0.3));

    KDL::Jacobian jac_kdl(7);
    jac_kdl.data.setIdentity();
    jac_kdl.changeRefPoint(KDL::Vector(0.1,0.2,0.3));

    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 7; j++)
            BOOST_CHECK_EQUAL(jac(i,j), jac_kdl.data(i,j));

    cout<<"Jacobian from WBC is " <<endl;
    cout<<jac<<endl;

    cout<<"Jacobian from KDL is " <<endl;
    cout<<jac_kdl.data<<endl<<endl;

    cout<<"Testing change Ref Frame ..."<<endl<<endl;

    base::Affine3d a;
    a.setIdentity();
    a.translate(base::Vector3d(1,2,3));
    jac.changeRefFrame(a);

    KDL::Frame a_kdl;
    a_kdl.p = KDL::Vector(1,2,3);
    jac_kdl.changeRefFrame(a_kdl);

    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 7; j++)
            BOOST_CHECK_EQUAL(jac(i,j), jac_kdl.data(i,j));

    cout<<"Jacobian from WBC is " <<endl;
    cout<<jac<<endl;

    cout<<"Jacobian from KDL is " <<endl;
    cout<<jac_kdl.data<<endl<<endl;

    cout<<"...done"<<endl<<endl;
}
