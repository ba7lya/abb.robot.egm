# SPDX-License-Identifier: MIT

# Micro benchmarks for the interpolation classes. The library's own C++ standard
# requirement is PRIVATE, so it is not inherited by consumers; the target sets its
# own instead. benchmark::benchmark_main supplies main().

add_executable(
    ${PROJECT_NAME}_benchmark
        common_auxiliary_benchmark.cxx
        interpolator_benchmark.cxx
)

target_link_libraries(
    ${PROJECT_NAME}_benchmark PRIVATE
        ba7lya::abb::robot::egm
        benchmark::benchmark
        benchmark::benchmark_main
)

target_compile_features(${PROJECT_NAME}_benchmark PRIVATE cxx_std_23)

if(MSVC)
    target_compile_options(${PROJECT_NAME}_benchmark PRIVATE /utf-8)
endif()
