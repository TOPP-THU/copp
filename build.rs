fn main() {
    #[cfg(feature = "cpp")]
    build_cpp_bridge();
}

#[cfg(feature = "cpp")]
fn build_cpp_bridge() {
    cxx_build::bridge("src/ffi/cpp/mod.rs")
        .file("bindings/cpp/src/interpolation.cpp")
        .file("bindings/cpp/src/core.cpp")
        .file("bindings/cpp/src/path.cpp")
        .file("bindings/cpp/src/robot.cpp")
        .file("bindings/cpp/src/topp2_ra.cpp")
        .file("bindings/cpp/src/clarabel.cpp")
        .file("bindings/cpp/src/copp2_socp.cpp")
        .file("bindings/cpp/src/topp3.cpp")
        .include("bindings/cpp/include")
        .define("COPP_CPP_BUILDING_LIBRARY", None)
        .std("c++17")
        .flag_if_supported("/EHsc")
        .compile("copp_cpp_bridge");

    println!("cargo:rerun-if-changed=src/ffi/cpp/mod.rs");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/core.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/eigen.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/clarabel.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/detail/path_bridge.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/detail/robot_bridge.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/interpolation.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/objective.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/path.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/robot.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/solver/copp2_socp.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/solver/copp3_socp.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/solver/reach_set2.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/solver/topp2_ra.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/solver/topp3.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/solver/topp3_lp.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/include/copp/solver/topp3_socp.hpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/clarabel.cpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/core.cpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/copp2_socp.cpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/interpolation.cpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/path.cpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/robot.cpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/topp2_ra.cpp");
    println!("cargo:rerun-if-changed=bindings/cpp/src/topp3.cpp");
}
