rock_testsuite(test_suite suite.cpp
   udpTest.cpp
   DEPS udp)

TARGET_LINK_LIBRARIES(test_suite
   pthread)