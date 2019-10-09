
rock_executable(sherpa_tt_recv sherpa_tt_recv.cpp
    DEPS proxy_library_sherpa_tt proxy_library_sherpa_tt_types
    DEPS_PKGCONFIG opencv)

rock_executable(sherpa_tt_send sherpa_tt_send.cpp
    DEPS proxy_library_sherpa_tt)

rock_executable(sherpa_tt_init_arm sherpa_tt_init_arm.cpp
    DEPS proxy_library_sherpa_tt)

rock_executable(load_dem load_dem.cpp
    DEPS proxy_library_sherpa_tt proxy_library_sherpa_tt_types
    DEPS_PKGCONFIG opencv)

rock_executable(sherpa_tt_hz sherpa_tt_hz.cpp
    DEPS proxy_library_sherpa_tt
    DEPS_PKGCONFIG base-types)

rock_executable(sherpa_tt_ratio sherpa_tt_ratio.cpp
    DEPS proxy_library_sherpa_tt
    DEPS_PKGCONFIG base-types)

rock_executable(sherpa_tt_sim_recv_camera_images sherpa_tt_sim_recv_camera_images.cpp
    DEPS proxy_library_sherpa_tt proxy_library_sherpa_tt_types
    DEPS_PKGCONFIG opencv)

rock_executable(sherpa_tt_sim_recv_ptu_joints_state sherpa_tt_sim_recv_ptu_joints_state.cpp
    DEPS proxy_library_sherpa_tt proxy_library_sherpa_tt_types)

rock_executable(sherpa_tt_sim_send_ptu_joints_cmd sherpa_tt_sim_send_ptu_joints_cmd.cpp
    DEPS proxy_library_sherpa_tt proxy_library_sherpa_tt_types)

