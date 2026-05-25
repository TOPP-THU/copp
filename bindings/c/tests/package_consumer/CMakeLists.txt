cmake_minimum_required(VERSION 3.16)
project(copp_package_consumer C)

find_package(copp CONFIG REQUIRED)

add_executable(copp_package_consumer main.c)
target_link_libraries(copp_package_consumer PRIVATE copp::copp)

if(WIN32)
    add_custom_command(
        TARGET copp_package_consumer POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "$<TARGET_FILE:copp::copp>"
                "$<TARGET_FILE_DIR:copp_package_consumer>"
    )
endif()

enable_testing()
add_test(NAME copp_package_consumer_runs COMMAND copp_package_consumer)
