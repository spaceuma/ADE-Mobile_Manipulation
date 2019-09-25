#define BOOST_TEST_MODULE BaseTypes
#include <boost/test/unit_test.hpp>

#define BASE_LOG_DEBUG
#include <base-logging/Logging.hpp>

BOOST_AUTO_TEST_CASE( logging_test )
{
#ifdef BASE_LONG_NAMES
        FILE* s = fopen("test.out", "w");

#ifdef WIN32
        BASE_LOG_CONFIGURE(INFO_P, s);
#else
        BASE_LOG_CONFIGURE(INFO, s);
#endif
        BASE_LOG_INFO("info-message")
#else
        FILE* s = fopen("test.out", "w");
#ifdef WIN32
        LOG_CONFIGURE(INFO_P, s);
#else 
	LOG_CONFIGURE(INFO, s);
#endif

        LOG_INFO("info-message")
#endif

        std::string test("additional-argument");

        int number = 1000000;
        time_t start,stop;
        time(&start);
        for(int i = 0; i < number; i++)
        {
#ifdef BASE_LONG_NAMES
            BASE_LOG_FATAL("test fatal log %s", test.c_str())
#else
            LOG_FATAL("test fatal log %s", test.c_str())
#endif
        }
        time(&stop);
        double seconds = difftime(stop, start)/(number*1.0);
        printf("Estimated time per log msg %f seconds", seconds);
}

