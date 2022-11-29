get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)


file(GLOB srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${srcs})
set_target_properties(${example_name}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
