fn main() {
    let dst = cmake::build("sbgECom").join("build");

    println!("cargo:rustc-link-search={}", dst.display());
    println!("cargo:rustc-link-lib=static=sbgECom");

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=sbgECom");
}
