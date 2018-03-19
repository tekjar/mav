extern crate mav;

use mav::Mav2Rust;
use std::path::Path;

fn main() {
    let spec_path = Path::new("common.xml");
    let code_path = Path::new(("src/mavlink.rs"));

    let mav2rust = Mav2Rust::new(&spec_path);
    mav2rust.generate(code_path);
}
