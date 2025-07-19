import React, { useRef, useEffect, useState, useMemo } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import { Point3D, SceneView } from "../types/photogrammetry";
import * as THREE from "three";

interface SceneViewerProps {
  pointCloud: Point3D[];
  cameraPositions: Point3D[];
  sceneView: SceneView;
  isProcessing: boolean;
  videoFrames?: ImageData[]; // Add video frames for camera textures
}

interface PointCloudMeshProps {
  points: Point3D[];
  pointSize: number;
  colorMode: string;
}

const PointCloudMesh: React.FC<PointCloudMeshProps> = ({
  points,
  pointSize,
  colorMode,
}) => {
  const meshRef = useRef<THREE.Points>(null);

  const geometry = useMemo(() => {
    if (points.length === 0) return new THREE.BufferGeometry();

    const vertices = new Float32Array(points.length * 3);
    const colors = new Float32Array(points.length * 3);

    points.forEach((point, i) => {
      vertices[i * 3] = point.x;
      vertices[i * 3 + 1] = point.y;
      vertices[i * 3 + 2] = point.z;

      if (
        colorMode === "rgb" &&
        point.r !== undefined &&
        point.g !== undefined &&
        point.b !== undefined
      ) {
        colors[i * 3] = point.r / 255;
        colors[i * 3 + 1] = point.g / 255;
        colors[i * 3 + 2] = point.b / 255;
      } else if (colorMode === "height") {
        const normalized = Math.max(0, Math.min(1, (point.z + 10) / 20));
        const hue = (240 + normalized * 120) / 360;
        const rgb = new THREE.Color().setHSL(hue, 0.7, 0.5);
        colors[i * 3] = rgb.r;
        colors[i * 3 + 1] = rgb.g;
        colors[i * 3 + 2] = rgb.b;
      } else {
        colors[i * 3] = 1;
        colors[i * 3 + 1] = 1;
        colors[i * 3 + 2] = 1;
      }
    });

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));

    return geometry;
  }, [points, colorMode]);

  const material = useMemo(() => {
    return new THREE.PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: true,
    });
  }, [pointSize]);

  return <points ref={meshRef} geometry={geometry} material={material} />;
};

interface CameraMeshProps {
  positions: Point3D[];
  videoFrames?: ImageData[];
}

const CameraMesh: React.FC<CameraMeshProps> = ({ positions, videoFrames }) => {
  const groupRef = useRef<THREE.Group>(null);

  // Convert ImageData to textures
  const textures = useMemo(() => {
    if (!videoFrames) return [];

    return videoFrames.map((frame) => {
      const canvas = document.createElement("canvas");
      canvas.width = frame.width;
      canvas.height = frame.height;
      const ctx = canvas.getContext("2d");
      if (ctx) {
        ctx.putImageData(frame, 0, 0);
      }
      const texture = new THREE.CanvasTexture(canvas);
      texture.flipY = true; // Flip Y coordinate to match 3D coordinate system
      return texture;
    });
  }, [videoFrames]);

  // Calculate adaptive frame size based on minimum distance between cameras
  const frameSize = useMemo(() => {
    if (positions.length < 2) return 1.0; // Default size

    let minDistance = Infinity;
    for (let i = 1; i < positions.length; i++) {
      const prev = positions[i - 1];
      const curr = positions[i];
      const distance = Math.sqrt(
        Math.pow(curr.x - prev.x, 2) +
          Math.pow(curr.y - prev.y, 2) +
          Math.pow(curr.z - prev.z, 2)
      );
      if (distance > 0) {
        minDistance = Math.min(minDistance, distance);
      }
    }

    // Scale frame size to half the minimum distance, with reasonable bounds
    return Math.max(0.1, Math.min(2.0, minDistance / 2));
  }, [positions]);

  return (
    <group ref={groupRef}>
      {positions.map((camera, index) => {
        const hasTexture = textures[index];
        const aspectRatio = hasTexture
          ? videoFrames![index].width / videoFrames![index].height
          : 1.6; // Default 16:9 aspect ratio

        // Extract rotation from camera data (rx, ry, rz are Euler angles)
        const rotX = camera.rx || 0;
        const rotY = camera.ry || 0;
        const rotZ = camera.rz || 0;

        // Debug log rotation values for the first few cameras
        if (index < 3) {
          console.log(
            `Camera ${index}: rx=${rotX.toFixed(3)}, ry=${rotY.toFixed(
              3
            )}, rz=${rotZ.toFixed(3)}`
          );
        }

        // Calculate the rotation for the plane so its back faces the camera viewing direction
        // The Euler angles (rx, ry, rz) represent the camera's orientation
        // In Three.js, a plane by default has its normal pointing toward +Z
        // We want the plane's normal to point in the camera's viewing direction
        // The camera viewing direction is typically along the negative Z axis in camera coordinates
        // So we apply the camera rotation directly to align the plane with the camera orientation
        const planeRotX = rotX;
        const planeRotY = rotY; // Don't add Math.PI - apply rotation directly
        const planeRotZ = rotZ;

        return (
          <group key={index} position={[camera.x, camera.y, camera.z]}>
            {hasTexture ? (
              // Camera as textured plane showing the video frame
              <>
                <mesh
                  rotation={[planeRotX, planeRotY, planeRotZ]} // Apply rotation to the mesh directly
                >
                  <planeGeometry args={[frameSize * aspectRatio, frameSize]} />
                  <meshBasicMaterial
                    map={textures[index]}
                    transparent={true}
                    alphaTest={0.1}
                    side={THREE.DoubleSide} // Make it visible from both sides
                  />
                </mesh>

                {/* Add a small indicator for camera direction */}
                <mesh
                  position={[0, 0, -frameSize * 0.1]}
                  rotation={[planeRotX, planeRotY, planeRotZ]}
                >
                  <coneGeometry
                    args={[frameSize * 0.05, frameSize * 0.15, 6]}
                  />
                  <meshBasicMaterial
                    color="#ff0000"
                    transparent={true}
                    opacity={0.8}
                  />
                </mesh>

                {/* Add axis indicators to show orientation */}
                <group rotation={[planeRotX, planeRotY, planeRotZ]}>
                  {/* X axis - Red */}
                  <mesh
                    position={[frameSize * 0.3, 0, 0]}
                    rotation={[0, 0, Math.PI / 2]}
                  >
                    <cylinderGeometry
                      args={[
                        frameSize * 0.01,
                        frameSize * 0.01,
                        frameSize * 0.6,
                        8,
                      ]}
                    />
                    <meshBasicMaterial color="#ff0000" />
                  </mesh>
                  {/* Y axis - Green */}
                  <mesh position={[0, frameSize * 0.3, 0]}>
                    <cylinderGeometry
                      args={[
                        frameSize * 0.01,
                        frameSize * 0.01,
                        frameSize * 0.6,
                        8,
                      ]}
                    />
                    <meshBasicMaterial color="#00ff00" />
                  </mesh>
                  {/* Z axis - Blue (viewing direction) */}
                  <mesh
                    position={[0, 0, -frameSize * 0.3]}
                    rotation={[Math.PI / 2, 0, 0]}
                  >
                    <cylinderGeometry
                      args={[
                        frameSize * 0.01,
                        frameSize * 0.01,
                        frameSize * 0.6,
                        8,
                      ]}
                    />
                    <meshBasicMaterial color="#0000ff" />
                  </mesh>
                </group>
              </>
            ) : (
              // Fallback to simple box representation
              <group rotation={[rotX, rotY, rotZ]}>
                {/* Camera body */}
                <mesh>
                  <boxGeometry
                    args={[frameSize * 0.1, frameSize * 0.05, frameSize * 0.05]}
                  />
                  <meshBasicMaterial color="#ff4444" />
                </mesh>

                {/* Camera lens */}
                <mesh position={[frameSize * 0.05, 0, 0]}>
                  <cylinderGeometry
                    args={[
                      frameSize * 0.015,
                      frameSize * 0.015,
                      frameSize * 0.025,
                      8,
                    ]}
                  />
                  <meshBasicMaterial color="#333333" />
                </mesh>
              </group>
            )}
          </group>
        );
      })}
    </group>
  );
};

interface AutoCameraControlsProps {
  cameraPositions: Point3D[];
  isProcessing: boolean;
  autoFollow: boolean;
  onUserInteraction: () => void;
}

const AutoCameraControls: React.FC<AutoCameraControlsProps> = ({
  cameraPositions,
  isProcessing,
  autoFollow,
  onUserInteraction,
}) => {
  const { camera } = useThree();
  const controlsRef = useRef<any>(null);
  const [hasUserInteracted, setHasUserInteracted] = useState(false);

  useEffect(() => {
    if (controlsRef.current && !hasUserInteracted) {
      const controls = controlsRef.current;

      const handleStart = () => {
        setHasUserInteracted(true);
        onUserInteraction();
      };

      controls.addEventListener("start", handleStart);
      return () => controls.removeEventListener("start", handleStart);
    }
  }, [hasUserInteracted, onUserInteraction]);

  useFrame(() => {
    if (
      autoFollow &&
      !hasUserInteracted &&
      isProcessing &&
      controlsRef.current
    ) {
      // Auto-follow the latest camera position
      if (cameraPositions.length > 0) {
        const latestCamera = cameraPositions[cameraPositions.length - 1];
        const controls = controlsRef.current;

        // Smoothly move camera to view the latest position
        const targetPosition = new THREE.Vector3(
          latestCamera.x + 10,
          latestCamera.y + 5,
          latestCamera.z + 10
        );

        camera.position.lerp(targetPosition, 0.02);
        controls.target.lerp(
          new THREE.Vector3(latestCamera.x, latestCamera.y, latestCamera.z),
          0.02
        );
        controls.update();
      }
    }
  });

  return (
    <OrbitControls
      ref={controlsRef}
      enablePan={true}
      enableZoom={true}
      enableRotate={true}
      dampingFactor={0.1}
      enableDamping={true}
    />
  );
};

export const SceneViewer: React.FC<SceneViewerProps> = ({
  pointCloud,
  cameraPositions,
  sceneView,
  isProcessing,
  videoFrames,
}) => {
  const [autoFollow, setAutoFollow] = useState(true);
  const [hasUserInteracted, setHasUserInteracted] = useState(false);

  const handleUserInteraction = () => {
    setAutoFollow(false);
    setHasUserInteracted(true);
  };

  const handleResetAutoFollow = () => {
    setAutoFollow(true);
    setHasUserInteracted(false);
  };

  return (
    <div className="relative w-full h-full">
      <Canvas>
        <PerspectiveCamera makeDefault position={[15, 10, 15]} />

        {/* Lighting */}
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10]} intensity={1} />
        <pointLight position={[-10, -10, -10]} intensity={0.5} />

        {/* Grid */}
        <gridHelper args={[50, 50, "#444444", "#222222"]} />

        {/* Point cloud */}
        {sceneView.showPointCloud && pointCloud.length > 0 && (
          <PointCloudMesh
            points={pointCloud}
            pointSize={sceneView.pointSize}
            colorMode={sceneView.colorMode}
          />
        )}

        {/* Camera positions */}
        {sceneView.showCameras && cameraPositions.length > 0 && (
          <CameraMesh positions={cameraPositions} videoFrames={videoFrames} />
        )}

        {/* Controls */}
        <AutoCameraControls
          cameraPositions={cameraPositions}
          isProcessing={isProcessing}
          autoFollow={autoFollow}
          onUserInteraction={handleUserInteraction}
        />
      </Canvas>

      {/* UI Overlay */}
      <div className="absolute top-4 left-4 bg-card rounded-lg p-3 text-sm space-y-2 shadow-lg">
        <div>Points: {pointCloud.length}</div>
        <div>Cameras: {cameraPositions.length}</div>
        {hasUserInteracted && !autoFollow && (
          <button
            onClick={handleResetAutoFollow}
            className="px-3 py-1 bg-primary text-primary-foreground rounded text-xs hover:bg-primary/90 transition-colors"
          >
            Auto Follow
          </button>
        )}
      </div>

      <div className="absolute top-4 right-4 bg-card rounded-lg p-3 text-xs space-y-1 shadow-lg">
        <div>Left click + drag: Rotate</div>
        <div>Right click + drag: Pan</div>
        <div>Mouse wheel: Zoom</div>
      </div>
    </div>
  );
};
