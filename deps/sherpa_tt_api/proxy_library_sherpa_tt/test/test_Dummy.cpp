#include <boost/test/unit_test.hpp>
#include <proxy_library_sherpa_tt/Dummy.hpp>

using namespace proxy_library_sherpa_tt;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    proxy_library_sherpa_tt::DummyClass dummy;
    dummy.welcome();
}
