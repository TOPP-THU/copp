cmake_minimum_required(VERSION 3.16)

set(COPP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
file(READ "${COPP_ROOT}/Cargo.toml" COPP_CARGO_TOML)
string(REGEX MATCH "version[ \t]*=[ \t]*\"([^\"]+)\"" _copp_version_match "${COPP_CARGO_TOML}")
if(CMAKE_MATCH_1)
    set(COPP_PACKAGE_VERSION "${CMAKE_MATCH_1}")
else()
    set(COPP_PACKAGE_VERSION "0.0.0")
endif()

project(copp_c_smoke VERSION "${COPP_PACKAGE_VERSION}" LANGUAGES C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(COPP_LINK_STATIC "Link COPP as a static library" OFF)
option(COPP_BUILD_TESTS "Build COPP C smoke tests" ON)
option(COPP_BUILD_EXAMPLES "Build COPP C examples" ON)
option(COPP_TEST_EXAMPLES "Register COPP C examples as CTest tests" OFF)
option(COPP_INSTALL "Install COPP C headers, native library, and CMake package config" ON)
option(COPP_BUILD_INSTALL_TESTS "Build CTest smoke tests for the installed CMake package" OFF)

set(COPP_TARGET_DIR "${COPP_ROOT}/target/release" CACHE PATH "Cargo release directory containing the COPP native library")
set(COPP_INSTALL_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/copp" CACHE STRING "Install directory for coppConfig.cmake")
set(COPP_STATIC_LINK_LIBRARIES "" CACHE STRING "Extra native libraries required by consumers of the static COPP library")

if(COPP_LINK_STATIC)
    set(COPP_IMPORTED_LIBRARY_TYPE STATIC)
else()
    set(COPP_IMPORTED_LIBRARY_TYPE SHARED)
endif()

if(WIN32)
    if(MINGW)
        if(COPP_LINK_STATIC)
            set(COPP_LIBRARY_CANDIDATES
                "${COPP_TARGET_DIR}/libcopp.a"
            )
        else()
            set(COPP_LIBRARY_CANDIDATES
                "${COPP_TARGET_DIR}/libcopp.dll.a"
            )
            set(COPP_RUNTIME_DLL "${COPP_TARGET_DIR}/copp.dll")
        endif()
    else()
        if(COPP_LINK_STATIC)
            set(COPP_LIBRARY_CANDIDATES
                "${COPP_TARGET_DIR}/copp.lib"
            )
        else()
            set(COPP_LIBRARY_CANDIDATES
                "${COPP_TARGET_DIR}/copp.dll.lib"
            )
            set(COPP_RUNTIME_DLL "${COPP_TARGET_DIR}/copp.dll")
        endif()
    endif()
elseif(APPLE)
    if(COPP_LINK_STATIC)
        set(COPP_LIBRARY_CANDIDATES
            "${COPP_TARGET_DIR}/libcopp.a"
        )
    else()
        set(COPP_LIBRARY_CANDIDATES
            "${COPP_TARGET_DIR}/libcopp.dylib"
        )
    endif()
else()
    if(COPP_LINK_STATIC)
        set(COPP_LIBRARY_CANDIDATES
            "${COPP_TARGET_DIR}/libcopp.a"
        )
    else()
        set(COPP_LIBRARY_CANDIDATES
            "${COPP_TARGET_DIR}/libcopp.so"
        )
    endif()
endif()

set(COPP_LIBRARY "")
foreach(candidate IN LISTS COPP_LIBRARY_CANDIDATES)
    if(EXISTS "${candidate}")
        set(COPP_LIBRARY "${candidate}")
        break()
    endif()
endforeach()

if(NOT COPP_LIBRARY)
    message(FATAL_ERROR "Could not find COPP library in ${COPP_TARGET_DIR}. Run `cargo build --release --lib --features c` first, or pass -DCOPP_TARGET_DIR for target-specific builds.")
endif()

if(WIN32 AND NOT COPP_LINK_STATIC AND NOT EXISTS "${COPP_RUNTIME_DLL}")
    message(FATAL_ERROR "Could not find COPP runtime DLL at ${COPP_RUNTIME_DLL}. Run `cargo build --release --lib --features c` first, or pass -DCOPP_TARGET_DIR for target-specific builds.")
endif()

add_library(copp_ffi ${COPP_IMPORTED_LIBRARY_TYPE} IMPORTED GLOBAL)
add_library(copp::copp ALIAS copp_ffi)

if(WIN32 AND NOT COPP_LINK_STATIC)
    set_target_properties(copp_ffi PROPERTIES
        IMPORTED_IMPLIB "${COPP_LIBRARY}"
        IMPORTED_LOCATION "${COPP_RUNTIME_DLL}"
    )
else()
    set_target_properties(copp_ffi PROPERTIES IMPORTED_LOCATION "${COPP_LIBRARY}")
endif()
target_include_directories(copp_ffi INTERFACE "${CMAKE_CURRENT_LIST_DIR}/include")

set(COPP_STATIC_PLATFORM_LINK_LIBRARIES "")
set(COPP_STATIC_PLATFORM_LINK_OPTIONS "")
if(COPP_LINK_STATIC)
    find_package(Threads QUIET)
    if(Threads_FOUND)
        list(APPEND COPP_STATIC_PLATFORM_LINK_LIBRARIES Threads::Threads)
    endif()

    if(WIN32)
        list(APPEND COPP_STATIC_PLATFORM_LINK_LIBRARIES ws2_32 userenv bcrypt advapi32 ntdll)
    elseif(APPLE)
        list(APPEND COPP_STATIC_PLATFORM_LINK_OPTIONS
            "SHELL:-framework CoreFoundation"
            "SHELL:-framework Security"
            "SHELL:-framework SystemConfiguration"
        )
        list(APPEND COPP_STATIC_PLATFORM_LINK_LIBRARIES m)
    elseif(UNIX)
        list(APPEND COPP_STATIC_PLATFORM_LINK_LIBRARIES ${CMAKE_DL_LIBS} m)
    endif()

    list(APPEND COPP_STATIC_PLATFORM_LINK_LIBRARIES ${COPP_STATIC_LINK_LIBRARIES})
    if(COPP_STATIC_PLATFORM_LINK_LIBRARIES)
        set_property(TARGET copp_ffi APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COPP_STATIC_PLATFORM_LINK_LIBRARIES})
    endif()
    if(COPP_STATIC_PLATFORM_LINK_OPTIONS)
        set_property(TARGET copp_ffi APPEND PROPERTY INTERFACE_LINK_OPTIONS ${COPP_STATIC_PLATFORM_LINK_OPTIONS})
    endif()
endif()

function(add_copp_c_executable target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE copp_ffi)
    if(UNIX AND NOT APPLE)
        target_link_libraries(${target} PRIVATE m)
    endif()

    if(WIN32 AND NOT COPP_LINK_STATIC AND EXISTS "${COPP_RUNTIME_DLL}")
        add_custom_command(
            TARGET ${target} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "${COPP_RUNTIME_DLL}"
                    "$<TARGET_FILE_DIR:${target}>"
        )
    endif()
endfunction()

function(add_copp_c_test target source)
    add_copp_c_executable(${target} ${source})
    add_test(NAME ${target} COMMAND ${target})
endfunction()

function(add_copp_c_example target source)
    add_copp_c_executable(${target} ${source})
    if(COPP_TEST_EXAMPLES)
        add_test(NAME ${target} COMMAND ${target})
    endif()
endfunction()

if(COPP_BUILD_TESTS OR COPP_TEST_EXAMPLES OR COPP_BUILD_INSTALL_TESTS)
    enable_testing()
endif()

if(COPP_BUILD_TESTS)
    add_copp_c_test(test_version tests/test_version.c)
    add_copp_c_test(test_error_handling tests/test_error_handling.c)
    add_copp_c_test(test_headers tests/test_headers.c)
    add_copp_c_test(test_topp2_interpolation tests/test_topp2_interpolation.c)
    add_copp_c_test(test_topp3_interpolation tests/test_topp3_interpolation.c)
    add_copp_c_test(test_reach_set2 tests/test_reach_set2.c)
    add_copp_c_test(test_topp2_ra tests/test_topp2_ra.c)
    add_copp_c_test(test_copp2_socp tests/test_copp2_socp.c)
    add_copp_c_test(test_topp3_lp tests/test_topp3_lp.c)
    add_copp_c_test(test_topp3_socp tests/test_topp3_socp.c)
    add_copp_c_test(test_copp3_socp tests/test_copp3_socp.c)
    add_copp_c_test(test_path_evaluation tests/test_path_evaluation.c)
    add_copp_c_test(test_robot_raw_constraints tests/test_robot_raw_constraints.c)
endif()

if(COPP_BUILD_EXAMPLES)
    add_copp_c_example(example_topp2_ra examples/topp2_ra.c)
    add_copp_c_example(example_reach_set2 examples/reach_set2.c)
    add_copp_c_example(example_copp2_socp examples/copp2_socp.c)
    add_copp_c_example(example_topp3_lp examples/topp3_lp.c)
    add_copp_c_example(example_topp3_socp examples/topp3_socp.c)
    add_copp_c_example(example_copp3_socp examples/copp3_socp.c)
endif()

if(COPP_INSTALL)
    install(DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/include/copp"
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

    if(WIN32 AND NOT COPP_LINK_STATIC)
        install(FILES "${COPP_LIBRARY}"
                DESTINATION "${CMAKE_INSTALL_LIBDIR}")
        install(FILES "${COPP_RUNTIME_DLL}"
                DESTINATION "${CMAKE_INSTALL_BINDIR}")
    else()
        install(FILES "${COPP_LIBRARY}"
                DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    endif()

    get_filename_component(COPP_CONFIG_LIBRARY_NAME "${COPP_LIBRARY}" NAME)
    if(WIN32 AND NOT COPP_LINK_STATIC)
        get_filename_component(COPP_CONFIG_RUNTIME_NAME "${COPP_RUNTIME_DLL}" NAME)
        set(COPP_CONFIG_IMPLIB_RELPATH "${CMAKE_INSTALL_LIBDIR}/${COPP_CONFIG_LIBRARY_NAME}")
        set(COPP_CONFIG_LOCATION_RELPATH "${CMAKE_INSTALL_BINDIR}/${COPP_CONFIG_RUNTIME_NAME}")
    else()
        set(COPP_CONFIG_IMPLIB_RELPATH "")
        set(COPP_CONFIG_LOCATION_RELPATH "${CMAKE_INSTALL_LIBDIR}/${COPP_CONFIG_LIBRARY_NAME}")
    endif()

    if(COPP_LINK_STATIC)
        set(COPP_CONFIG_TARGET_TYPE STATIC)
        set(COPP_CONFIG_IS_STATIC ON)
        set(COPP_CONFIG_STATIC_LINK_LIBRARIES "${COPP_STATIC_PLATFORM_LINK_LIBRARIES}")
        set(COPP_CONFIG_STATIC_LINK_OPTIONS "${COPP_STATIC_PLATFORM_LINK_OPTIONS}")
    else()
        set(COPP_CONFIG_TARGET_TYPE SHARED)
        set(COPP_CONFIG_IS_STATIC OFF)
        set(COPP_CONFIG_STATIC_LINK_LIBRARIES "")
        set(COPP_CONFIG_STATIC_LINK_OPTIONS "")
    endif()

    configure_package_config_file(
        "${CMAKE_CURRENT_LIST_DIR}/cmake/coppConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/coppConfig.cmake"
        INSTALL_DESTINATION "${COPP_INSTALL_CMAKE_DIR}"
        PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR CMAKE_INSTALL_BINDIR
    )
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/coppConfigVersion.cmake"
        VERSION "${PROJECT_VERSION}"
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/coppConfig.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/coppConfigVersion.cmake"
            DESTINATION "${COPP_INSTALL_CMAKE_DIR}")
endif()

if(COPP_BUILD_INSTALL_TESTS)
    set(COPP_INSTALL_TEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/package-install" CACHE PATH "Install prefix used by COPP package smoke tests")
    set(COPP_INSTALL_TEST_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/package-consumer-build")

    add_test(
        NAME test_install_copp_package
        COMMAND "${CMAKE_COMMAND}" --install "${CMAKE_BINARY_DIR}" --prefix "${COPP_INSTALL_TEST_PREFIX}" --config "$<CONFIG>"
    )
    add_test(
        NAME test_find_package_copp_configure
        COMMAND "${CMAKE_COMMAND}"
                -S "${CMAKE_CURRENT_LIST_DIR}/tests/package_consumer"
                -B "${COPP_INSTALL_TEST_BUILD_DIR}"
                -DCMAKE_PREFIX_PATH=${COPP_INSTALL_TEST_PREFIX}
                "-DCMAKE_BUILD_TYPE=$<CONFIG>"
    )
    add_test(
        NAME test_find_package_copp_build
        COMMAND "${CMAKE_COMMAND}" --build "${COPP_INSTALL_TEST_BUILD_DIR}" --config "$<CONFIG>"
    )
    add_test(
        NAME test_find_package_copp_run
        COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${COPP_INSTALL_TEST_BUILD_DIR}" --output-on-failure -C "$<CONFIG>"
    )

    set_tests_properties(test_find_package_copp_configure PROPERTIES DEPENDS test_install_copp_package)
    set_tests_properties(test_find_package_copp_build PROPERTIES DEPENDS test_find_package_copp_configure)
    set_tests_properties(test_find_package_copp_run PROPERTIES DEPENDS test_find_package_copp_build)
endif()
