# SPDX-License-Identifier: MIT

# Integration tests. The library's own C++ standard requirement is PRIVATE, so it is not
# inherited by consumers; every test target therefore sets its own instead.

add_executable(
    ${PROJECT_NAME}_integration
        controller_interface_test.cxx
)

target_link_libraries(${PROJECT_NAME}_integration PRIVATE ba7lya::abb::robot::egm GTest::gtest_main)

target_compile_features(${PROJECT_NAME}_integration PRIVATE cxx_std_23)

if(MSVC)
    target_compile_options(${PROJECT_NAME}_integration PRIVATE /utf-8)
endif()

gtest_discover_tests(${PROJECT_NAME}_integration)
