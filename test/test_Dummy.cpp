#include <boost/test/unit_test.hpp>
#include <wbc/Dummy.hpp>

using namespace wbc;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    wbc::DummyClass dummy;
    dummy.welcome();
}
