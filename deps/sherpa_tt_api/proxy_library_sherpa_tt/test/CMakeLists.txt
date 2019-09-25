#rock_testsuite(test_suite suite.cpp
#   test_Dummy.cpp
#   DEPS proxy_library_sherpa_tt)

rock_executable(proxy_library_sherpa_tt_test proxy_library_sherpa_tt_test.cpp
    DEPS proxy_library_sherpa_tt)

rock_executable(proxy_library_sherpa_tt_sim_test proxy_library_sherpa_tt_sim_test.cpp
    DEPS proxy_library_sherpa_tt)
