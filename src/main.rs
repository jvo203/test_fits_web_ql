#![recursion_limit = "1024"]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

#[macro_use]
extern crate ispc;

// Functions exported from ispc will be callable under spmd::*
ispc_module!(spmd);

#[macro_use]
extern crate scan_fmt;

#[macro_use]
extern crate lazy_static;

#[macro_use]
extern crate serde_json;

use std::{env, path, ptr};
use vpx_sys::*;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;

use actix::Actor;

//FITS datasets
lazy_static! {
    static ref DATASETS: Arc<RwLock<HashMap<String, Arc<RwLock<Box<fits::FITS>>>>>> =
        Arc::new(RwLock::new(HashMap::new()));
}

pub struct UserParams {
    pmin: f32,
    pmax: f32,
    lmin: f32,
    lmax: f32,
    black: f32,
    white: f32,
    median: f32,
    sensitivity: f32,
    ratio_sensitivity: f32,
    flux: String,
    start: usize,
    end: usize,
    mask: Vec<u8>,
    pixels: Vec<f32>,
}

mod fits;
mod server;

#[actix_web::main]
async fn main() {
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

    let filepath = std::path::PathBuf::from(test_file_path);
    if !filepath.exists() {
        eprintln!("Test FITS file does not exist: {:?}", filepath);
        return;
    }

    let dataid = "test_fits".to_string();
    let server = server::SessionServer::default().start();

    let fits = fits::FITS::from_path(
        &dataid,
        &"".to_owned(),
        filepath.as_path(),
        &"".to_owned(),
        &server,
    );
}

fn vpx_codec_enc_config_init() -> vpx_codec_enc_cfg_t {
    vpx_codec_enc_cfg_t {
        g_usage: 0,
        g_threads: 0,
        g_profile: 0,
        g_w: 0,
        g_h: 0,
        g_bit_depth: vpx_bit_depth::VPX_BITS_8,
        g_input_bit_depth: 8,
        g_timebase: vpx_rational { num: 0, den: 0 },
        g_error_resilient: 0,
        g_pass: vpx_enc_pass::VPX_RC_ONE_PASS,
        g_lag_in_frames: 0,
        rc_dropframe_thresh: 0,
        rc_resize_allowed: 0,
        rc_scaled_width: 0,
        rc_scaled_height: 0,
        rc_resize_up_thresh: 0,
        rc_resize_down_thresh: 0,
        rc_end_usage: vpx_rc_mode::VPX_VBR,
        rc_twopass_stats_in: vpx_fixed_buf {
            buf: ptr::null_mut(),
            sz: 0,
        },
        rc_firstpass_mb_stats_in: vpx_fixed_buf {
            buf: ptr::null_mut(),
            sz: 0,
        },
        rc_target_bitrate: 0,
        rc_min_quantizer: 0,
        rc_max_quantizer: 0,
        rc_undershoot_pct: 0,
        rc_overshoot_pct: 0,
        rc_buf_sz: 0,
        rc_buf_initial_sz: 0,
        rc_buf_optimal_sz: 0,
        rc_2pass_vbr_bias_pct: 0,
        rc_2pass_vbr_minsection_pct: 0,
        rc_2pass_vbr_maxsection_pct: 0,
        rc_2pass_vbr_corpus_complexity: 0,
        kf_mode: vpx_kf_mode::VPX_KF_AUTO,
        kf_min_dist: 0,
        kf_max_dist: 0,
        ss_number_layers: 0,
        ss_enable_auto_alt_ref: [0; 5],
        ss_target_bitrate: [0; 5],
        ts_number_layers: 0,
        ts_target_bitrate: [0; 5],
        ts_rate_decimator: [0; 5],
        ts_periodicity: 0,
        ts_layer_id: [0; 16],
        layer_target_bitrate: [0; 12],
        temporal_layering_mode: 0,
        use_vizier_rc_params: 0,
        active_wq_factor: vpx_rational { num: 0, den: 0 },
        err_per_mb_factor: vpx_rational { num: 0, den: 0 },
        sr_default_decay_limit: vpx_rational { num: 0, den: 0 },
        sr_diff_factor: vpx_rational { num: 0, den: 0 },
        kf_err_per_mb_factor: vpx_rational { num: 0, den: 0 },
        kf_frame_min_boost_factor: vpx_rational { num: 0, den: 0 },
        kf_frame_max_boost_first_factor: vpx_rational { num: 0, den: 0 },
        kf_frame_max_boost_subs_factor: vpx_rational { num: 0, den: 0 },
        kf_max_total_boost_factor: vpx_rational { num: 0, den: 0 },
        gf_max_total_boost_factor: vpx_rational { num: 0, den: 0 },
        gf_frame_max_boost_factor: vpx_rational { num: 0, den: 0 },
        zm_factor: vpx_rational { num: 0, den: 0 },
        rd_mult_inter_qp_fac: vpx_rational { num: 0, den: 0 },
        rd_mult_arf_qp_fac: vpx_rational { num: 0, den: 0 },
        rd_mult_key_qp_fac: vpx_rational { num: 0, den: 0 },
    }
}
