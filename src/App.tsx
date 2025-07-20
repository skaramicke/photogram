import React, {
  useState,
  useEffect,
  useCallback,
  useRef,
  useMemo,
} from "react";
import init, { PhotogrammetryProcessor } from "./wasm/photogram_wasm.js";
import { VideoUploader } from "./components/VideoUploader";
import { ProcessingPanel } from "./components/ProcessingPanel";
import { SceneViewer } from "./components/SceneViewer";
import { ProgressBar } from "./components/ProgressBar";
import { StatusIndicator } from "./components/StatusIndicator";
import { SettingsPanel } from "./components/SettingsPanel";
import { ExportPanel } from "./components/ExportPanel";
import { TransformControls } from "./components/TransformControls";
import {
  Point3D,
  ProcessingProgress,
  ProcessingSettings,
  ProcessingStage,
  VideoMetadata,
  SceneView,
} from "./types/photogrammetry";
import { extractVideoFrames } from "./lib/utils";
import {
  Play,
  Pause,
  Settings,
  Download,
  Eye,
  RotateCcw,
} from "./components/Icons";
import "./App.css";

function App() {
  const [wasmLoaded, setWasmLoaded] = useState(false);
  const [processor, setProcessor] = useState<PhotogrammetryProcessor | null>(
    null
  );
  const [isProcessing, setIsProcessing] = useState(false);
  const [processingStage, setProcessingStage] =
    useState<ProcessingStage>("idle");
  const [progress, setProgress] = useState<ProcessingProgress>({
    current_frame: 0,
    total_frames: 0,
    points_detected: 0,
    triangulated_points: 0,
    stage: "idle",
    progress_percent: 0,
  });

  const [videoFile, setVideoFile] = useState<File | null>(null);
  const [videoMetadata, setVideoMetadata] = useState<VideoMetadata | null>(
    null
  );
  const [pointCloud, setPointCloud] = useState<Point3D[]>([]);
  const [cameraPositions, setCameraPositions] = useState<Point3D[]>([]);
  const [frames, setFrames] = useState<ImageData[]>([]);

  const [settings, setSettings] = useState<ProcessingSettings>({
    maxFrames: 50,
    keyframeInterval: 5,
    featureDetectionThreshold: 0.01,
    triangulationThreshold: 0.1,
    quality: "medium",
  });

  const [sceneView, setSceneView] = useState<SceneView>({
    showPointCloud: true,
    showCameras: true,
    showMesh: false,
    pointSize: 2,
    colorMode: "rgb",
  });

  const [showSettings, setShowSettings] = useState(false);
  const [showExport, setShowExport] = useState(false);

  // Coordinate transformation controls
  const [transformControls, setTransformControls] = useState({
    // Position transformations (sliders from -1 to 1)
    posX: 1, // 1 = no change, -1 = invert
    posY: -1, // -1 = invert (current Y-up conversion)
    posZ: -1, // -1 = invert (current depth convention)

    // Rotation matrix transformations (sliders from -1 to 1)
    rotR0: 1, // First row multiplier
    rotR1: -1, // Second row multiplier
    rotR2: -1, // Third row multiplier

    // User camera transformation when following frames
    userCamOffset: [0, 0, -5] as [number, number, number], // Offset in frame's local coordinate system [x, y, z]
    userCamLookAt: [0, 0, -1] as [number, number, number], // Direction to look relative to frame [x, y, z]
    useFrameRotation: true, // Whether to rotate look direction by frame's rotation matrix
  });

  const [selectedFrameId, setSelectedFrameId] = useState<number | null>(null);

  const processingIntervalRef = useRef<ReturnType<typeof setTimeout> | null>(
    null
  );

  useEffect(() => {
    init()
      .then(() => {
        setWasmLoaded(true);
        const proc = new PhotogrammetryProcessor();
        setProcessor(proc);
        console.log("Photogrammetry WASM module loaded successfully!");
      })
      .catch((err: any) => {
        console.error("Failed to load WASM module:", err);
        setProcessingStage("error");
      });
  }, []);

  const handleVideoUpload = useCallback(async (file: File) => {
    setVideoFile(file);
    setProcessingStage("loading");

    try {
      // Extract video metadata
      const video = document.createElement("video");
      video.src = URL.createObjectURL(file);

      await new Promise((resolve) => {
        video.addEventListener("loadedmetadata", () => {
          const metadata: VideoMetadata = {
            duration: video.duration,
            width: video.videoWidth,
            height: video.videoHeight,
            fps: 30, // Default, would need more complex detection
            size: file.size,
            filename: file.name,
          };
          setVideoMetadata(metadata);
          URL.revokeObjectURL(video.src);
          resolve(void 0);
        });
      });

      setProcessingStage("idle");
    } catch (error) {
      console.error("Error loading video:", error);
      setProcessingStage("error");
    }
  }, []);

  const startProcessing = useCallback(async () => {
    if (!processor || !videoFile) return;

    setIsProcessing(true);
    setProcessingStage("frame_extraction");

    try {
      // Create a fresh processor instance to avoid any stale state issues
      const freshProcessor = new PhotogrammetryProcessor();

      // Extract frames from video
      const extractedFrames = await extractVideoFrames(
        videoFile,
        settings.maxFrames
      );
      setFrames(extractedFrames);

      // Set proper camera intrinsics for 4K video (or estimate based on first frame)
      const firstFrame = extractedFrames[0];
      if (firstFrame) {
        // Estimate focal length for typical drone camera (assume ~90° horizontal FOV)
        const width = firstFrame.width;
        const height = firstFrame.height;
        const estimatedFocalLength =
          width / (2 * Math.tan((90 * Math.PI) / 180 / 2));
        const principalPointX = width / 2;
        const principalPointY = height / 2;

        console.log(
          `Setting camera params: focal_length=${estimatedFocalLength.toFixed(
            1
          )}, pp=(${principalPointX}, ${principalPointY}) for ${width}x${height} video`
        );
        freshProcessor.set_camera_params(
          estimatedFocalLength,
          principalPointX,
          principalPointY
        );
      }

      // Set total frames in fresh processor
      freshProcessor.set_total_frames(extractedFrames.length);

      setProcessingStage("feature_detection");

      // Process frames with WASM - using fresh processor
      const processedFrames = [];
      for (let i = 0; i < extractedFrames.length; i++) {
        const frame = extractedFrames[i];

        try {
          const result = freshProcessor.process_frame(
            i,
            i / 30,
            frame.width,
            frame.height
          );

          if (result) {
            const frameData = JSON.parse(result);
            processedFrames.push(frameData);
          }
        } catch (e) {
          console.warn("Failed to process frame:", e);
        }

        // Important delay to ensure WASM call completes before next call
        await new Promise((resolve) => setTimeout(resolve, 100));

        // Update progress AFTER the process_frame call is completely done
        try {
          const progressData = freshProcessor.get_progress();
          if (progressData) {
            const progressObj = JSON.parse(progressData);
            setProgress(progressObj);
          }
        } catch (e) {
          console.warn("Failed to parse progress data:", e);
        }

        // Update camera positions during processing for real-time visualization
        try {
          const cameras = freshProcessor.get_camera_positions();
          if (cameras) {
            const camerasArray = JSON.parse(cameras);
            setCameraPositions(camerasArray);
          }
        } catch (e) {
          console.warn(
            "Failed to parse camera positions during processing:",
            e
          );
        }

        // Another delay to prevent rapid successive calls
        await new Promise((resolve) => setTimeout(resolve, 100));
      }

      setProcessingStage("triangulation");

      // Add delay before triangulation
      await new Promise((resolve) => setTimeout(resolve, 200));

      // Triangulate points using fresh processor
      try {
        const triangulatedPoints = freshProcessor.triangulate_points();
        if (triangulatedPoints) {
          const pointsArray = JSON.parse(triangulatedPoints);
          setPointCloud(pointsArray);
        }
      } catch (e) {
        console.warn("Failed to parse point cloud data:", e);
      }

      // Add delay before getting camera positions
      await new Promise((resolve) => setTimeout(resolve, 200));

      // Get camera positions using fresh processor
      try {
        const cameras = freshProcessor.get_camera_positions();
        if (cameras) {
          const camerasArray = JSON.parse(cameras);
          setCameraPositions(camerasArray);
        }
      } catch (e) {
        console.warn("Failed to parse camera positions:", e);
      }

      setProcessingStage("completed");
      setIsProcessing(false);
    } catch (error) {
      console.error("Processing error:", error);
      setProcessingStage("error");
      setIsProcessing(false);
    }
  }, [processor, videoFile, settings.maxFrames]);

  const pauseProcessing = useCallback(() => {
    if (processingIntervalRef.current) {
      clearInterval(processingIntervalRef.current);
    }
    setIsProcessing(false);
  }, []);

  const resetProcessing = useCallback(() => {
    setIsProcessing(false);
    setProcessingStage("idle");
    setProgress({
      current_frame: 0,
      total_frames: 0,
      points_detected: 0,
      triangulated_points: 0,
      stage: "idle",
      progress_percent: 0,
    });
    setPointCloud([]);
    setCameraPositions([]);
    setFrames([]);

    if (processor) {
      // Reset processor state
      const newProcessor = new PhotogrammetryProcessor();
      setProcessor(newProcessor);
    }
  }, [processor]);

  // Transform camera positions based on current controls
  const transformedCameraPositions = useMemo(() => {
    return cameraPositions.map((camera) => {
      // Apply position transformation
      const transformedPos = {
        x: camera.x * transformControls.posX,
        y: camera.y * transformControls.posY,
        z: camera.z * transformControls.posZ,
      };

      // Apply rotation matrix transformation if rotation exists
      if (
        "rotation" in camera &&
        Array.isArray(camera.rotation) &&
        camera.rotation.length === 9
      ) {
        const originalRotation = camera.rotation as number[];
        const transformedRotation = [
          originalRotation[0] * transformControls.rotR0,
          originalRotation[1] * transformControls.rotR0,
          originalRotation[2] * transformControls.rotR0,
          originalRotation[3] * transformControls.rotR1,
          originalRotation[4] * transformControls.rotR1,
          originalRotation[5] * transformControls.rotR1,
          originalRotation[6] * transformControls.rotR2,
          originalRotation[7] * transformControls.rotR2,
          originalRotation[8] * transformControls.rotR2,
        ];

        return {
          ...transformedPos,
          rotation: transformedRotation,
          viewDirection:
            "viewDirection" in camera ? camera.viewDirection : undefined,
        };
      }

      return transformedPos;
    });
  }, [cameraPositions, transformControls]);

  // Handle frame click to snap user camera
  const handleFrameClick = useCallback((frameIndex: number) => {
    setSelectedFrameId(frameIndex);
    // This will be handled in SceneViewer via props
  }, []);

  // Handle transform controls change
  const handleTransformChange = useCallback((newControls: any) => {
    setTransformControls(newControls);
  }, []);

  // Reset transform controls to defaults
  const resetTransformControls = useCallback(() => {
    setTransformControls({
      posX: 1,
      posY: -1,
      posZ: -1,
      rotR0: 1,
      rotR1: -1,
      rotR2: -1,
      userCamOffset: [0, 0, -5] as [number, number, number],
      userCamLookAt: [0, 0, -1] as [number, number, number],
      useFrameRotation: true,
    });
  }, []);

  const [showTransformControls, setShowTransformControls] = useState(false);

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <header className="border-b border-border bg-card">
        <div className="container mx-auto px-4 py-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center space-x-4">
              <h1 className="text-2xl font-bold text-foreground">
                Photogrammetry Studio
              </h1>
              <StatusIndicator
                status={processingStage}
                isWasmLoaded={wasmLoaded}
              />
            </div>

            <div className="flex items-center space-x-2">
              <button
                onClick={() => setShowSettings(!showSettings)}
                className="p-2 rounded-lg bg-secondary hover:bg-accent transition-colors"
                title="Settings"
              >
                <Settings size={20} />
              </button>

              <button
                onClick={() => setShowExport(!showExport)}
                className="p-2 rounded-lg bg-secondary hover:bg-accent transition-colors"
                title="Export"
                disabled={pointCloud.length === 0}
              >
                <Download size={20} />
              </button>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="container mx-auto px-4 py-6">
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Left Panel - Upload & Processing */}
          <div className="lg:col-span-1 space-y-6">
            {/* Video Upload */}
            <div className="bg-card rounded-lg p-6 card-shadow">
              <h2 className="text-lg font-semibold mb-4">Video Upload</h2>
              <VideoUploader
                onVideoUpload={handleVideoUpload}
                videoMetadata={videoMetadata}
                isProcessing={isProcessing}
              />
            </div>

            {/* Processing Controls */}
            {videoFile && (
              <div className="bg-card rounded-lg p-6 card-shadow">
                <h2 className="text-lg font-semibold mb-4">
                  Processing Controls
                </h2>
                <div className="flex space-x-2 mb-4">
                  <button
                    onClick={startProcessing}
                    disabled={isProcessing || !wasmLoaded}
                    className="flex items-center space-x-2 px-4 py-2 bg-primary text-primary-foreground rounded-lg hover:bg-primary/90 disabled:opacity-50 btn-transition"
                  >
                    <Play size={16} />
                    <span>Start Processing</span>
                  </button>

                  <button
                    onClick={pauseProcessing}
                    disabled={!isProcessing}
                    className="flex items-center space-x-2 px-4 py-2 bg-secondary text-secondary-foreground rounded-lg hover:bg-accent btn-transition"
                  >
                    <Pause size={16} />
                    <span>Pause</span>
                  </button>

                  <button
                    onClick={resetProcessing}
                    className="flex items-center space-x-2 px-4 py-2 bg-destructive text-destructive-foreground rounded-lg hover:bg-destructive/90 btn-transition"
                  >
                    <RotateCcw size={16} />
                    <span>Reset</span>
                  </button>
                </div>

                <ProgressBar progress={progress} />
              </div>
            )}

            {/* Processing Panel */}
            {(isProcessing || processingStage !== "idle") && (
              <ProcessingPanel stage={processingStage} frames={frames} />
            )}
          </div>

          {/* Right Panel - 3D View */}
          <div className="lg:col-span-2">
            <div className="bg-card rounded-lg p-6 card-shadow h-[600px]">
              <div className="flex items-center justify-between mb-4">
                <h2 className="text-lg font-semibold">3D Scene View</h2>
                <div className="flex items-center space-x-2">
                  <button
                    onClick={() =>
                      setSceneView((prev) => ({
                        ...prev,
                        showPointCloud: !prev.showPointCloud,
                      }))
                    }
                    className={`p-2 rounded-lg transition-colors ${
                      sceneView.showPointCloud
                        ? "bg-primary text-primary-foreground"
                        : "bg-secondary hover:bg-accent"
                    }`}
                    title="Toggle Point Cloud"
                  >
                    <Eye size={16} />
                  </button>

                  <button
                    onClick={() =>
                      setShowTransformControls(!showTransformControls)
                    }
                    className={`p-2 rounded-lg transition-colors ${
                      showTransformControls
                        ? "bg-primary text-primary-foreground"
                        : "bg-secondary hover:bg-accent"
                    }`}
                    title="Transform Controls"
                  >
                    <Settings size={16} />
                  </button>
                </div>
              </div>

              {/* Transform Controls */}
              {showTransformControls && (
                <TransformControls
                  transformControls={transformControls}
                  onTransformChange={handleTransformChange}
                  onResetToDefaults={resetTransformControls}
                />
              )}

              <SceneViewer
                pointCloud={pointCloud}
                cameraPositions={transformedCameraPositions}
                sceneView={sceneView}
                isProcessing={isProcessing}
                videoFrames={frames}
                onFrameClick={handleFrameClick}
                selectedFrameId={selectedFrameId}
                userCamTransform={{
                  offset: transformControls.userCamOffset,
                  lookAt: transformControls.userCamLookAt,
                  useFrameRotation: transformControls.useFrameRotation,
                }}
              />
            </div>
          </div>
        </div>
      </main>

      {/* Settings Panel */}
      {showSettings && (
        <SettingsPanel
          settings={settings}
          onSettingsChange={setSettings}
          onClose={() => setShowSettings(false)}
        />
      )}

      {/* Export Panel */}
      {showExport && processor && (
        <ExportPanel
          processor={processor}
          pointCloud={pointCloud}
          onClose={() => setShowExport(false)}
        />
      )}
    </div>
  );
}

export default App;
