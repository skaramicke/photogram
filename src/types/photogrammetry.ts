export interface Point3D {
  x: number;
  y: number;
  z: number;
  confidence?: number;
  r?: number;
  g?: number;
  b?: number;
  // Camera rotation data (new format)
  rotation?: number[]; // 3x3 rotation matrix as flat array [m00,m01,m02,m10,m11,m12,m20,m21,m22]
  viewDirection?: number[]; // Camera viewing direction [x,y,z]
  // Legacy format (for backward compatibility)
  rx?: number;
  ry?: number;
  rz?: number;
}

export interface CameraParams {
  focal_length: number;
  principal_point_x: number;
  principal_point_y: number;
  distortion_coeffs: number[];
}

export interface PhotoFrame {
  id: number;
  timestamp: number;
  camera_position: Point3D;
  camera_rotation: number[]; // quaternion [w, x, y, z]
  keypoints: Point3D[];
  is_processed: boolean;
}

export interface ProcessingProgress {
  current_frame?: number;
  total_frames?: number;
  points_detected?: number;
  triangulated_points?: number;
  stage?: string;
  progress_percent?: number;
}

export interface ProcessingSettings {
  maxFrames: number;
  keyframeInterval: number;
  featureDetectionThreshold: number;
  triangulationThreshold: number;
  quality: 'low' | 'medium' | 'high' | 'ultra';
}

export interface ExportFormat {
  type: 'ply' | 'obj' | 'json';
  includeColors: boolean;
  includeCameras: boolean;
  decimation: number;
}

export type ProcessingStage = 
  | 'idle'
  | 'loading'
  | 'frame_extraction'
  | 'feature_detection'
  | 'matching'
  | 'triangulation'
  | 'bundle_adjustment'
  | 'meshing'
  | 'completed'
  | 'error';

export interface VideoMetadata {
  duration: number;
  width: number;
  height: number;
  fps: number;
  size: number;
  filename: string;
}

export interface SceneView {
  showPointCloud: boolean;
  showCameras: boolean;
  showMesh: boolean;
  pointSize: number;
  colorMode: 'rgb' | 'confidence' | 'height';
}
