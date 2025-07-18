use wasm_bindgen::prelude::*;
use web_sys::console;
use serde::{Deserialize, Serialize};

// This is like the `extern` block in C
#[wasm_bindgen]
extern "C" {
    // Bind the `alert` function from the browser
    fn alert(s: &str);
}

// Define a macro to provide `println!(..)` style syntax for `console.log` logging
macro_rules! log {
    ( $( $t:tt )* ) => {
        console::log_1(&format!( $( $t )* ).into());
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub confidence: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CameraParams {
    pub focal_length: f64,
    pub principal_point_x: f64,
    pub principal_point_y: f64,
    pub distortion_coeffs: Vec<f64>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PhotoFrame {
    pub id: u32,
    pub timestamp: f64,
    pub camera_position: Point3D,
    pub camera_rotation: Vec<f64>, // quaternion [w, x, y, z]
    pub keypoints: Vec<Point3D>,
    pub is_processed: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ProcessingProgress {
    pub current_frame: u32,
    pub total_frames: u32,
    pub points_detected: u32,
    pub triangulated_points: u32,
    pub stage: String,
    pub progress_percent: f64,
}

#[wasm_bindgen]
pub struct PhotogrammetryProcessor {
    frames: Vec<PhotoFrame>,
    point_cloud: Vec<Point3D>,
    camera_params: CameraParams,
    processing_stage: String,
    progress: f64,
}

#[wasm_bindgen]
impl PhotogrammetryProcessor {
    #[wasm_bindgen(constructor)]
    pub fn new() -> PhotogrammetryProcessor {
        log!("Initializing PhotogrammetryProcessor");
        PhotogrammetryProcessor {
            frames: Vec::new(),
            point_cloud: Vec::new(),
            camera_params: CameraParams {
                focal_length: 1000.0,
                principal_point_x: 320.0,
                principal_point_y: 240.0,
                distortion_coeffs: vec![0.0, 0.0, 0.0, 0.0],
            },
            processing_stage: "initialized".to_string(),
            progress: 0.0,
        }
    }

    #[wasm_bindgen]
    pub fn set_camera_params(&mut self, focal_length: f64, pp_x: f64, pp_y: f64) {
        self.camera_params.focal_length = focal_length;
        self.camera_params.principal_point_x = pp_x;
        self.camera_params.principal_point_y = pp_y;
        log!("Camera params updated: f={}, pp=({}, {})", focal_length, pp_x, pp_y);
    }

    #[wasm_bindgen]
    pub fn process_frame(&mut self, frame_id: u32, timestamp: f64, width: u32, height: u32) -> JsValue {
        log!("Processing frame {}", frame_id);
        
        // Simulate feature detection
        let num_keypoints = 100 + (frame_id % 50) as usize;
        let mut keypoints = Vec::new();
        
        for i in 0..num_keypoints {
            let x = (i as f64 * 3.14159 / num_keypoints as f64).sin() * 100.0 + width as f64 / 2.0;
            let y = (i as f64 * 3.14159 / num_keypoints as f64).cos() * 100.0 + height as f64 / 2.0;
            keypoints.push(Point3D {
                x,
                y,
                z: 0.0,
                confidence: 0.8 + (i as f64 / num_keypoints as f64) * 0.2,
            });
        }

        // Simulate camera pose estimation
        let camera_position = Point3D {
            x: (frame_id as f64 * 0.1).sin() * 5.0,
            y: (frame_id as f64 * 0.1).cos() * 5.0,
            z: frame_id as f64 * 0.1,
            confidence: 0.95,
        };

        let frame = PhotoFrame {
            id: frame_id,
            timestamp,
            camera_position,
            camera_rotation: vec![1.0, 0.0, 0.0, 0.0], // identity quaternion
            keypoints,
            is_processed: true,
        };

        self.frames.push(frame.clone());
        
        // Update progress
        self.progress = (frame_id as f64 / 100.0) * 100.0;
        self.processing_stage = "feature_detection".to_string();

        serde_wasm_bindgen::to_value(&frame).unwrap()
    }

    #[wasm_bindgen]
    pub fn triangulate_points(&mut self) -> JsValue {
        log!("Triangulating points from {} frames", self.frames.len());
        
        self.processing_stage = "triangulation".to_string();
        self.point_cloud.clear();

        // Simple triangulation simulation
        for i in 0..1000 {
            let x = (i as f64 * 0.01).sin() * 10.0;
            let y = (i as f64 * 0.01).cos() * 10.0;
            let z = (i as f64 * 0.005).sin() * 5.0;
            
            self.point_cloud.push(Point3D {
                x,
                y,
                z,
                confidence: 0.7 + (i as f64 / 1000.0) * 0.3,
            });
        }

        self.progress = 100.0;
        self.processing_stage = "completed".to_string();

        serde_wasm_bindgen::to_value(&self.point_cloud).unwrap()
    }

    #[wasm_bindgen]
    pub fn get_progress(&self) -> JsValue {
        let progress = ProcessingProgress {
            current_frame: self.frames.len() as u32,
            total_frames: 100,
            points_detected: self.frames.iter().map(|f| f.keypoints.len() as u32).sum(),
            triangulated_points: self.point_cloud.len() as u32,
            stage: self.processing_stage.clone(),
            progress_percent: self.progress,
        };

        serde_wasm_bindgen::to_value(&progress).unwrap()
    }

    #[wasm_bindgen]
    pub fn get_point_cloud(&self) -> JsValue {
        serde_wasm_bindgen::to_value(&self.point_cloud).unwrap()
    }

    #[wasm_bindgen]
    pub fn get_camera_positions(&self) -> JsValue {
        let positions: Vec<Point3D> = self.frames.iter()
            .map(|f| f.camera_position.clone())
            .collect();
        
        serde_wasm_bindgen::to_value(&positions).unwrap()
    }

    #[wasm_bindgen]
    pub fn export_ply(&self) -> String {
        let mut ply_content = String::new();
        ply_content.push_str("ply\n");
        ply_content.push_str("format ascii 1.0\n");
        ply_content.push_str(&format!("element vertex {}\n", self.point_cloud.len()));
        ply_content.push_str("property float x\n");
        ply_content.push_str("property float y\n");
        ply_content.push_str("property float z\n");
        ply_content.push_str("property float confidence\n");
        ply_content.push_str("end_header\n");

        for point in &self.point_cloud {
            ply_content.push_str(&format!("{} {} {} {}\n", 
                point.x, point.y, point.z, point.confidence));
        }

        ply_content
    }
}

// Legacy functions for backward compatibility
#[wasm_bindgen]
pub fn greet(name: &str) -> String {
    log!("Hello from Rust! Greeting {}", name);
    format!("Hello, {}! This message is from Rust compiled to WebAssembly.", name)
}

#[wasm_bindgen]
pub fn fibonacci(n: u32) -> u32 {
    log!("Computing fibonacci({}) in Rust", n);
    match n {
        0 => 0,
        1 => 1,
        _ => fibonacci(n - 1) + fibonacci(n - 2),
    }
}

#[wasm_bindgen]
pub fn process_array(numbers: &[f64]) -> Vec<f64> {
    log!("Processing array of {} numbers in Rust", numbers.len());
    numbers.iter().map(|x| x * x).collect()
}

// Called when the wasm module is instantiated
#[wasm_bindgen(start)]
pub fn main() {
    log!("Photogrammetry WASM module loaded!");
}
