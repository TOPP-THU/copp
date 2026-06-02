cmake_minimum_required(VERSION 3.16)

set(COPP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
file(READ "${COPP_ROOT}/Cargo.toml" COPP_CARGO_TOML)
string(REGEX MATCH "version[ \t]*=[ \t]*\"([^\"]+)\"" _copp_version_match "${COPP_CARGO_TOML}")
if(CMAKE_MATCH_1)
    set(COPP_PACKAGE_VERSION "${CMAKE_MATCH_1}")
else()
    set(COPP_PACKAGE_VERSION "0.0.0")
endif()

project(copp_cpp VERSION "${COPP_PACKAGE_VERSION}" LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(COPP_LINK_STATIC "Link COPP as a static library" ON)
option(COPP_CPP_WITH_EIGEN "Enable optional Eigen adapters for the C++ facade" ON)
option(COPP_BUILD_CPP_TESTS "Build COPP C++ smoke tests" OFF)
option(COPP_BUILD_CPP_EXAMPLES "Build COPP C++ examples" OFF)
option(COPP_TEST_CPP_EXAMPLES "Register COPP C++ examples as CTest tests" OFF)
option(COPP_INSTALL "Install COPP C++ headers, native library, and CMake package config" ON)
option(COPP_INSTALL_C_ABI_TARGET "Install a separate C ABI target from the same package" ON)

set(COPP_TARGET_DIR "${COPP_ROOT}/target/release" CACHE PATH "Cargo release directory containing the COPP native library")
set(COPP_INSTALL_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/copp" CACHE STRING "Install directory for coppConfig.cmake")
set(COPP_STATIC_LINK_LIBRARIES "" CACHE STRING "Extra native libraries required by consumers of the static COPP library")

set(COPP_CPP_FACADE_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/src/interpolation.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/core.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/path.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/robot.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/topp2_ra.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/clarabel.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/copp2_socp.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/topp3.cpp"
)

if(COPP_INSTALL_C_ABI_TARGET)
    set(COPP_CARGO_FEATURE_HINT "cpp,c")
else()
    set(COPP_CARGO_FEATURE_HINT "cpp")
endif()

if(COPP_LINK_STATIC)
    set(COPP_IMPORTED_LIBRARY_TYPE STATIC)
    set(COPP_CONFIG_CPP_SHARED 0)
else()
    set(COPP_IMPORTED_LIBRARY_TYPE SHARED)
    set(COPP_CONFIG_CPP_SHARED 1)
endif()

if(WIN32)
    if(MINGW)
        set(COPP_STATIC_LIBRARY_CANDIDATES "${COPP_TARGET_DIR}/libcopp.a")
    else()
        set(COPP_STATIC_LIBRARY_CANDIDATES "${COPP_TARGET_DIR}/copp.lib")
    endif()
elseif(APPLE)
    set(COPP_STATIC_LIBRARY_CANDIDATES "${COPP_TARGET_DIR}/libcopp.a")
else()
    set(COPP_STATIC_LIBRARY_CANDIDATES "${COPP_TARGET_DIR}/libcopp.a")
endif()

set(COPP_STATIC_LIBRARY "")
foreach(candidate IN LISTS COPP_STATIC_LIBRARY_CANDIDATES)
    if(EXISTS "${candidate}")
        set(COPP_STATIC_LIBRARY "${candidate}")
        break()
    endif()
endforeach()

if(NOT COPP_STATIC_LIBRARY)
    message(FATAL_ERROR "Could not find COPP static library in ${COPP_TARGET_DIR}. The C++ CMake flow is Cargo-first and imports an existing Rust/C++ native artifact; it does not rebuild Rust automatically. Run `cargo build --release --lib --features ${COPP_CARGO_FEATURE_HINT}` after any Rust or C++ bridge change, or pass -DCOPP_TARGET_DIR for target-specific builds.")
endif()

if(NOT COPP_LINK_STATIC)
    file(GLOB COPP_CXXBRIDGE_INCLUDE_DIRS CONFIGURE_DEPENDS
        "${COPP_TARGET_DIR}/build/copp-*/out/cxxbridge/include")
    file(GLOB COPP_CXXBRIDGE_CRATE_DIRS CONFIGURE_DEPENDS
        "${COPP_TARGET_DIR}/build/copp-*/out/cxxbridge/crate")

    if(NOT COPP_CXXBRIDGE_INCLUDE_DIRS OR NOT COPP_CXXBRIDGE_CRATE_DIRS)
        message(FATAL_ERROR "Could not find generated cxx bridge headers under ${COPP_TARGET_DIR}/build. Run `cargo build --release --lib --features ${COPP_CARGO_FEATURE_HINT}` before configuring the C++ shared facade.")
    endif()

    if(COPP_INSTALL_C_ABI_TARGET)
        message(FATAL_ERROR "COPP_INSTALL_C_ABI_TARGET=ON is currently supported by the C++ CMake package only when COPP_LINK_STATIC=ON. For a shared C++ SDK, pass -DCOPP_INSTALL_C_ABI_TARGET=OFF; use the C ABI package/workflow for a shared C target.")
    endif()
endif()

if(COPP_LINK_STATIC)
    add_library(copp_cpp ${COPP_IMPORTED_LIBRARY_TYPE} IMPORTED GLOBAL)
    set_target_properties(copp_cpp PROPERTIES
        IMPORTED_LOCATION "${COPP_STATIC_LIBRARY}"
        IMPORTED_LINK_INTERFACE_LANGUAGES CXX
    )
else()
    add_library(copp_cpp SHARED ${COPP_CPP_FACADE_SOURCES})
    set_target_properties(copp_cpp PROPERTIES OUTPUT_NAME copp_cpp)
    target_include_directories(copp_cpp PRIVATE
        ${COPP_CXXBRIDGE_INCLUDE_DIRS}
        ${COPP_CXXBRIDGE_CRATE_DIRS}
    )
    target_compile_definitions(copp_cpp PRIVATE COPP_CPP_BUILDING_LIBRARY)
    target_link_libraries(copp_cpp PRIVATE "${COPP_STATIC_LIBRARY}")
endif()
add_library(copp::copp ALIAS copp_cpp)

if(COPP_LINK_STATIC)
    target_include_directories(copp_cpp INTERFACE "${CMAKE_CURRENT_LIST_DIR}/include")
    target_compile_features(copp_cpp INTERFACE cxx_std_17)
else()
    target_include_directories(copp_cpp PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
    target_compile_features(copp_cpp PUBLIC cxx_std_17)
endif()

if(NOT COPP_LINK_STATIC)
    target_compile_definitions(copp_cpp INTERFACE COPP_CPP_SHARED=1)
endif()

if(COPP_CPP_WITH_EIGEN)
    set(COPP_CONFIG_CPP_WITH_EIGEN 1)
    find_package(Eigen3 CONFIG REQUIRED)
    if(COPP_LINK_STATIC)
        target_compile_definitions(copp_cpp INTERFACE COPP_CPP_WITH_EIGEN=1)
        target_link_libraries(copp_cpp INTERFACE Eigen3::Eigen)
    else()
        target_compile_definitions(copp_cpp PUBLIC COPP_CPP_WITH_EIGEN=1)
        target_link_libraries(copp_cpp PUBLIC Eigen3::Eigen)
    endif()
else()
    set(COPP_CONFIG_CPP_WITH_EIGEN 0)
    if(COPP_LINK_STATIC)
        target_compile_definitions(copp_cpp INTERFACE COPP_CPP_WITH_EIGEN=0)
    else()
        target_compile_definitions(copp_cpp PUBLIC COPP_CPP_WITH_EIGEN=0)
    endif()
endif()

if(COPP_INSTALL_C_ABI_TARGET)
    add_library(copp_c_abi ${COPP_IMPORTED_LIBRARY_TYPE} IMPORTED GLOBAL)
    add_library(copp::c_abi ALIAS copp_c_abi)
    set_target_properties(copp_c_abi PROPERTIES
        IMPORTED_LOCATION "${COPP_STATIC_LIBRARY}"
        IMPORTED_LINK_INTERFACE_LANGUAGES CXX
    )
    target_include_directories(copp_c_abi INTERFACE "${COPP_ROOT}/bindings/c/include")
endif()

set(COPP_STATIC_PLATFORM_LINK_LIBRARIES "")
set(COPP_STATIC_PLATFORM_LINK_OPTIONS "")
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

if(COPP_LINK_STATIC)
    if(COPP_STATIC_PLATFORM_LINK_LIBRARIES)
        set_property(TARGET copp_cpp APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COPP_STATIC_PLATFORM_LINK_LIBRARIES})
        if(TARGET copp_c_abi)
            set_property(TARGET copp_c_abi APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COPP_STATIC_PLATFORM_LINK_LIBRARIES})
        endif()
    endif()
    if(COPP_STATIC_PLATFORM_LINK_OPTIONS)
        set_property(TARGET copp_cpp APPEND PROPERTY INTERFACE_LINK_OPTIONS ${COPP_STATIC_PLATFORM_LINK_OPTIONS})
        if(TARGET copp_c_abi)
            set_property(TARGET copp_c_abi APPEND PROPERTY INTERFACE_LINK_OPTIONS ${COPP_STATIC_PLATFORM_LINK_OPTIONS})
        endif()
    endif()
else()
    if(COPP_STATIC_PLATFORM_LINK_LIBRARIES)
        target_link_libraries(copp_cpp PRIVATE ${COPP_STATIC_PLATFORM_LINK_LIBRARIES})
    endif()
    if(COPP_STATIC_PLATFORM_LINK_OPTIONS)
        target_link_options(copp_cpp PRIVATE ${COPP_STATIC_PLATFORM_LINK_OPTIONS})
    endif()
endif()

function(add_copp_cpp_executable target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE copp_cpp)
    if(WIN32 AND NOT COPP_LINK_STATIC)
        add_custom_command(
            TARGET ${target} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "$<TARGET_FILE:copp_cpp>"
                    "$<TARGET_FILE_DIR:${target}>"
        )
    endif()
endfunction()

function(add_copp_cpp_test target source)
    add_copp_cpp_executable(${target} ${source})
    add_test(NAME ${target} COMMAND ${target})
endfunction()

function(add_copp_cpp_example target source)
    add_copp_cpp_executable(${target} ${source})
    if(COPP_TEST_CPP_EXAMPLES)
        add_test(NAME ${target} COMMAND ${target})
    endif()
endfunction()

if(COPP_BUILD_CPP_TESTS OR COPP_TEST_CPP_EXAMPLES)
    enable_testing()
endif()

if(COPP_BUILD_CPP_TESTS)
    add_copp_cpp_test(test_version tests/test_version.cpp)
    add_copp_cpp_test(test_headers tests/test_headers.cpp)
    add_copp_cpp_test(test_eigen_adapter tests/test_eigen_adapter.cpp)
    add_copp_cpp_test(test_error_handling tests/test_error_handling.cpp)
    add_copp_cpp_test(test_no_throw tests/test_no_throw.cpp)
    add_copp_cpp_test(test_topp2_interpolation tests/test_topp2_interpolation.cpp)
    add_copp_cpp_test(test_topp3_interpolation tests/test_topp3_interpolation.cpp)
    add_copp_cpp_test(test_reach_set2 tests/test_reach_set2.cpp)
    add_copp_cpp_test(test_topp2_ra tests/test_topp2_ra.cpp)
    add_copp_cpp_test(test_copp2_socp tests/test_copp2_socp.cpp)
    add_copp_cpp_test(test_topp3_problem tests/test_topp3_problem.cpp)
    add_copp_cpp_test(test_topp3_lp tests/test_topp3_lp.cpp)
    add_copp_cpp_test(test_topp3_socp tests/test_topp3_socp.cpp)
    add_copp_cpp_test(test_copp3_socp tests/test_copp3_socp.cpp)
    add_copp_cpp_test(test_path tests/test_path.cpp)
    add_copp_cpp_test(test_robot_constraints tests/test_robot_constraints.cpp)
    add_copp_cpp_test(test_robot_inverse_dynamics tests/test_robot_inverse_dynamics.cpp)
endif()

if(COPP_BUILD_CPP_EXAMPLES)
    add_copp_cpp_example(example_path_from_waypoints examples/path_from_waypoints.cpp)
    add_copp_cpp_example(example_path_from_parametric examples/path_from_parametric.cpp)
    add_copp_cpp_example(example_path_from_evaluator_2nd examples/path_from_evaluator_2nd.cpp)
    add_copp_cpp_example(example_path_from_evaluator_3rd examples/path_from_evaluator_3rd.cpp)
    add_copp_cpp_example(example_topp2_ra examples/topp2_ra.cpp)
    add_copp_cpp_example(example_reach_set2 examples/reach_set2.cpp)
    add_copp_cpp_example(example_copp2_socp examples/copp2_socp.cpp)
    add_copp_cpp_example(example_robot_inverse_dynamics examples/robot_inverse_dynamics.cpp)
    add_copp_cpp_example(example_topp3_lp examples/topp3_lp.cpp)
    add_copp_cpp_example(example_topp3_socp examples/topp3_socp.cpp)
    add_copp_cpp_example(example_copp3_socp examples/copp3_socp.cpp)
endif()

if(COPP_INSTALL)
    install(DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/include/copp"
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

    if(COPP_INSTALL_C_ABI_TARGET)
        install(DIRECTORY "${COPP_ROOT}/bindings/c/include/copp"
                DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
    endif()

    if(COPP_LINK_STATIC)
        install(FILES "${COPP_STATIC_LIBRARY}" DESTINATION "${CMAKE_INSTALL_LIBDIR}")

        get_filename_component(COPP_CONFIG_LIBRARY_NAME "${COPP_STATIC_LIBRARY}" NAME)
        set(COPP_CONFIG_IMPLIB_RELPATH "")
        set(COPP_CONFIG_LOCATION_RELPATH "${CMAKE_INSTALL_LIBDIR}/${COPP_CONFIG_LIBRARY_NAME}")
    else()
        install(TARGETS copp_cpp
                RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
                LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
                ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

        if(WIN32)
            if(MINGW)
                set(COPP_CONFIG_IMPLIB_RELPATH "${CMAKE_INSTALL_LIBDIR}/libcopp_cpp.dll.a")
            else()
                set(COPP_CONFIG_IMPLIB_RELPATH "${CMAKE_INSTALL_LIBDIR}/copp_cpp.lib")
            endif()
            set(COPP_CONFIG_LOCATION_RELPATH "${CMAKE_INSTALL_BINDIR}/copp_cpp.dll")
        elseif(APPLE)
            set(COPP_CONFIG_IMPLIB_RELPATH "")
            set(COPP_CONFIG_LOCATION_RELPATH "${CMAKE_INSTALL_LIBDIR}/libcopp_cpp.dylib")
        else()
            set(COPP_CONFIG_IMPLIB_RELPATH "")
            set(COPP_CONFIG_LOCATION_RELPATH "${CMAKE_INSTALL_LIBDIR}/libcopp_cpp.so")
        endif()
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
