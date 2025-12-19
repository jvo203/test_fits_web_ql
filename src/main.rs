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
use std::ffi::CString;
use std::fs::File;
use std::io::Write;
use std::sync::Arc;
use std::time::Instant;
use std::time::SystemTime;
use uuid::Uuid;

use actix::{Actor, Addr};
use bincode::{Encode, config, encode_to_vec};

use ffmpeg_next as ffmpeg;

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
mod kalman;
mod server;

use crate::kalman::KalmanFilter;

const PROGRESS_INTERVAL: u64 = 250; //[ms]

static LOG_DIRECTORY: &'static str = "/tmp";

#[derive(Encode, Debug)]
pub struct WsFrame {
    pub ts: f32,
    pub seq_id: u32,
    pub msg_type: u32,
    pub elapsed: f32,
    pub frame: Vec<u8>,
}

#[actix_web::main]
async fn main() -> Result<(), ffmpeg::Error> {
    // Register all components (codecs, formats, etc.)
    ffmpeg::init()?;

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

    let fits = Arc::new(RwLock::new(Box::new(fits)));
    DATASETS.write().insert(dataid.clone(), fits.clone());

    if fits.read().has_data {
        fits.read().make_data_histogram();
    } else {
        eprintln!("FITS file has no data: {:?}", filepath);
        return;
    }

    // id: a Vector of String
    let id = vec![dataid.to_string()];

    hevc_test(server.clone(), id.clone());

    /*let no_threads = 1;

    let handles: Vec<std::thread::JoinHandle<()>> = (0..no_threads)
        .map(|_| {
            let server_clone = server.clone();
            let id_clone = id.clone();
            std::thread::spawn(move || {
                hevc_test(server_clone, id_clone);
            })
        })
        .collect();

    for handle in handles {
        handle.join().unwrap();
    }*/

    println!("HEVC test completed.");

    // drop dataset references so Drop impls run (x265/vpx cleanup in UserSession::drop)
    DATASETS.write().clear();

    // give a short grace period for background drops/IO to finish
    //tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    // stop actix runtime / system
    actix::System::current().stop();

     Ok(())
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

struct UserSession {
    addr: Addr<server::SessionServer>,
    dataset_id: Vec<String>,
    session_id: Uuid,
    pool: Option<rayon::ThreadPool>,
    user: Option<UserParams>,
    timestamp: std::time::Instant,          //inactivity timeout
    progress_timestamp: std::time::Instant, //WebSocket progress timestamp
    log: std::io::Result<File>,
    wasm: bool,
    hevc: std::io::Result<File>,
    cfg: vpx_codec_enc_cfg_t, //VP9 encoder config
    ctx: vpx_codec_ctx_t,     //VP9 encoder context
    param: *mut x265_param,   //HEVC param
    enc: *mut x265_encoder,   //HEVC context
    pic: *mut x265_picture,   //HEVC picture
    //config: EncoderConfig,
    width: u32,
    height: u32,
    streaming: bool,
    last_video_frame: i32,
    video_frame: f64,
    video_ref_freq: f64,
    video_fps: f64,
    video_seq_id: i32,
    video_timestamp: std::time::Instant,
    bitrate: i32,
    kf: KalmanFilter,
}

impl UserSession {
    pub fn new(addr: Addr<server::SessionServer>, id: &Vec<String>) -> UserSession {
        let uuid = Uuid::new_v4();

        #[cfg(not(feature = "jvo"))]
        let filename = format!("/dev/null");

        #[cfg(feature = "jvo")]
        let filename = format!("{}/{}_{}.log", LOG_DIRECTORY, id[0].replace("/", "_"), uuid);

        let log = File::create(filename);

        //let filename = format!("/dev/null");
        let filename = format!(
            "{}/{}_{}.hevc",
            LOG_DIRECTORY,
            id[0].replace("/", "_"),
            uuid
        );

        let hevc = File::create(filename);

        let num_threads = num_cpus::get_physical();
        let pool = match rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .build()
        {
            Ok(pool) => Some(pool),
            Err(err) => {
                println!("{:?}, switching to a global rayon pool", err);
                None
            }
        };

        let session = UserSession {
            addr: addr.clone(),
            dataset_id: id.clone(),
            session_id: uuid,
            pool: pool,
            user: None,
            timestamp: std::time::Instant::now(), //SpawnHandle::default(),
            progress_timestamp: std::time::Instant::now()
                - std::time::Duration::from_millis(PROGRESS_INTERVAL),
            log: log,
            wasm: false,
            hevc: hevc,
            //cfg: vpx_codec_enc_cfg::default(),
            cfg: vpx_codec_enc_config_init(),
            ctx: vpx_codec_ctx_t {
                name: ptr::null(),
                iface: ptr::null_mut(),
                err: VPX_CODEC_OK,
                err_detail: ptr::null(),
                init_flags: 0,
                config: vpx_codec_ctx__bindgen_ty_1 { enc: ptr::null() },
                priv_: ptr::null_mut(),
            },
            param: ptr::null_mut(),
            enc: ptr::null_mut(),
            pic: ptr::null_mut(),
            //config: EncoderConfig::default(),
            width: 0,
            height: 0,
            streaming: false,
            last_video_frame: -1,
            video_frame: 0.0,
            video_ref_freq: 0.0,
            video_fps: 10.0,
            video_seq_id: 0,
            video_timestamp: std::time::Instant::now(),
            bitrate: 1000,
            kf: KalmanFilter::default(),
        };

        println!("allocating a new websocket session for {}", id[0]);

        session
    }
}

impl Drop for UserSession {
    fn drop(&mut self) {
        println!("dropping a websocket session for {}", self.dataset_id[0]);

        unsafe { vpx_codec_destroy(&mut self.ctx) };

        unsafe {
            if !self.param.is_null() {
                x265_param_free(self.param);
            }

            if !self.enc.is_null() {
                x265_encoder_close(self.enc);
            }

            if !self.pic.is_null() {
                x265_picture_free(self.pic);
            }
        }
    }
}

// create a separate function to stress-test the Rust-x265 interface (pass the cloned server and id)
#[allow(dead_code)]
fn hevc_test(server: Addr<server::SessionServer>, id: Vec<String>) {
    println!("Starting a separate HEVC test thread...");

    // borrow the fits dataset
    let datasets = DATASETS.read();
    let fits = match datasets.get(&id[0]) {
        Some(x) => x,
        None => {
            eprintln!("FITS dataset not found: {}", id[0]);
            return;
        }
    };

    let fits = match fits.try_read() {
        Some(x) => x,
        None => {
            eprintln!("FITS dataset is busy: {}", id[0]);
            return;
        }
    };

    {
        *fits.timestamp.write() = SystemTime::now();
    }

    // set up the dimensions
    let width = 1024; // a dummy width
    let height = 768; // a dummy height
    // force downsizing
    //let width = (fits.width / 2) as u32;
    //let height = (fits.height / 2) as u32;
    let depth = fits.depth; // the number of FITS planes (frames)

    println!(
        "HEVC width: {}, height: {}, depth (number of frames): {}",
        width, height, depth
    );

    // create a user session
    let mut session = UserSession::new(server, &id);
    session.width = width;
    session.height = height;
    let flux = "".to_string();
    let target_bitrate = 2000; //kbps
    let fps = 60;
    let mut seq_id: i32 = 0;
    let is_composite = false;

    //HEVC config
    //alloc HEVC params
    if session.param.is_null() {
        session.param = unsafe { x265_param_alloc() };
        unsafe {
            //x265_param_default_preset(session.param, CString::new("ultrafast").unwrap().as_ptr(), CString::new("fastdecode").unwrap().as_ptr());

            let tune = CString::new("zerolatency").unwrap();

            /*let tune = if fits.telescope.contains("kiso") {
                CString::new("grain").unwrap()
            } else {
                CString::new("zerolatency").unwrap()
            };*/

            if session.dataset_id.len() == 1 || !is_composite {
                let preset = CString::new("superfast").unwrap();
                x265_param_default_preset(session.param, preset.as_ptr(), tune.as_ptr());
            } else {
                let preset = CString::new("ultrafast").unwrap();
                x265_param_default_preset(session.param, preset.as_ptr(), tune.as_ptr());
            }

            (*session.param).fpsNum = fps as u32;
            (*session.param).fpsDenom = 1;
        };
    }

    unsafe {
        (*session.param).bRepeatHeaders = 1;

        if session.dataset_id.len() > 1 && is_composite {
            (*session.param).internalCsp = X265_CSP_I444 as i32;
        } else {
            (*session.param).internalCsp = X265_CSP_I400 as i32;
        }

        (*session.param).internalBitDepth = 8;
        (*session.param).sourceWidth = width as i32;
        (*session.param).sourceHeight = height as i32;

        //constant bitrate
        (*session.param).rc.rateControlMode = X265_RC_METHODS_X265_RC_CRF as i32;
        (*session.param).rc.bitrate = target_bitrate; //1000;
    };

    if session.pic.is_null() {
        session.pic = unsafe { x265_picture_alloc() };
    }

    if session.enc.is_null() {
        session.enc = unsafe { x265_encoder_open(session.param) }; //x265_encoder_open_160 for x265 2.8
        unsafe { x265_picture_init(session.param, session.pic) };
    }

    //start a video creation event loop
    session.streaming = true;

    //HEVC (x265) encoding test
    //for frame_idx in 0..depth {
        for frame_idx in depth / 2..depth / 2 + 1 {
        println!("Encoding frame {}/{}", frame_idx + 1, depth);

        let watch = Instant::now();

        match fits.get_video_frame(
            frame_idx,
            session.width,
            session.height,
            &flux,
            &session.pool,
        ) {
            Some(mut y) => {
                unsafe {
                    (*session.pic).stride[0] = session.width as i32;
                    (*session.pic).planes[0] = y.as_mut_ptr() as *mut std::os::raw::c_void;
                    //adaptive bitrate
                    (*session.param).rc.bitrate = target_bitrate;
                }

                let ret = unsafe { x265_encoder_reconfig(session.enc, session.param) };

                if ret < 0 {
                    println!("x265: error changing the bitrate");
                }

                let mut nal_count: u32 = 0;
                let mut p_nal: *mut x265_nal = ptr::null_mut();
                let p_out: *mut x265_picture = ptr::null_mut();

                //encode
                let ret = unsafe {
                    x265_encoder_encode(session.enc, &mut p_nal, &mut nal_count, session.pic, p_out)
                };

                println!(
                    "x265 hevc video frame prepare/encode time: {:?}, speed {} frames per second, ret = {}, nal_count = {}",
                    watch.elapsed(),
                    1000000000 / watch.elapsed().as_nanos(),
                    ret,
                    nal_count
                );

                //y falls out of scope
                unsafe {
                    (*session.pic).stride[0] = 0 as i32;
                    (*session.pic).planes[0] = ptr::null_mut();
                }

                //process all NAL units one by one
                if nal_count > 0 {
                    let nal_units =
                        unsafe { std::slice::from_raw_parts(p_nal, nal_count as usize) };

                    for unit in nal_units {
                        println!("NAL unit type: {}, size: {}", unit.type_, unit.sizeBytes);

                        let payload = unsafe {
                            std::slice::from_raw_parts(unit.payload, unit.sizeBytes as usize)
                        };

                        let timestamp = std::time::Instant::now();
                        seq_id += 1;

                        let ws_frame = WsFrame {
                            ts: timestamp.elapsed().as_millis() as f32,
                            seq_id: seq_id as u32,
                            msg_type: 5, //an hevc video frame
                            //length: video_frame.len() as u32,
                            elapsed: watch.elapsed().as_millis() as f32,
                            frame: payload.to_vec(),
                        };

                        match encode_to_vec(&ws_frame, config::legacy()) {
                            Ok(bin) => {
                                println!("WsFrame binary length: {}", bin.len());
                                //println!("{}", bin);
                                //ctx.binary(bin);
                            }
                            Err(err) => println!(
                                "error serializing a WebSocket video frame response: {}",
                                err
                            ),
                        }

                        match session.hevc {
                            Ok(ref mut file) => {
                                let _ = file.write_all(payload);
                            }
                            Err(_) => {}
                        }
                    }
                }
            }
            None => {}
        }
    }
}
