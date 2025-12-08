#[macro_use]
extern crate ispc;

// Functions exported from ispc will be callable under spmd::*
ispc_module!(spmd);

mod fits;

fn main() {
    println!("Testing FITSWebQL v4 Rust-x265 interface.");

    // first get the home directory as String
    let home_dir = match std::env::var("HOME") {
        Ok(path) => path,
        Err(_) => {
            eprintln!("Could not get HOME environment variable.");
            return;
        }
    };
    println!("Home directory: {}", home_dir);

    let test_file_path = format!(
        "{}/NAO/NRO/FUGIN/FGN_21250-0050_1x1_12CO_v1.00_cube.fits",
        home_dir
    );
    println!("Test file path: {}", test_file_path);
}
