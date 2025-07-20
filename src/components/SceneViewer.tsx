import React, { useRef, useEffect, useState, useMemo, useCallback } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import { Point3D, SceneView } from "../types/photogrammetry";
import * as THREE from "three";

interface SceneViewerProps {
  pointCloud: Point3D[];
  cameraPositions: Point3D[];
  sceneView: SceneView;
  isProcessing: boolean;
  videoFrames?: ImageData[]; // Add video frames for camera textures
  onFrameClick?: (frameIndex: number) => void;
  selectedFrameId?: number | null;
  userCamTransform?: {
    offset: [number, number, number];
    lookAt: [number, number, number];
    useFrameRotation?: boolean;
  };
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
  onFrameClick?: (frameIndex: number) => void;
  selectedFrameId?: number | null;
}

const CameraMesh: React.FC<CameraMeshProps> = ({ 
  positions, 
  videoFrames, 
  onFrameClick, 
  selectedFrameId 
}) => {
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
    if (positions.length < 2) return 2.0; // Larger default size for single camera

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

    // Scale frame size to half the minimum distance, with much more generous bounds
    // Increase minimum size from 0.1 to 1.0 so frames are always visible
    return Math.max(1.0, Math.min(8.0, minDistance / 2));
  }, [positions]);

  return (
    <group ref={groupRef}>
      {positions.map((camera, index) => {
        const hasTexture = textures[index];
        const aspectRatio = hasTexture
          ? videoFrames![index].width / videoFrames![index].height
          : 1.6; // Default 16:9 aspect ratio

        // Extract rotation from camera data
        // New format: rotation matrix and viewing direction
        let rotationEuler: [number, number, number];

        if (camera.rotation && camera.rotation.length === 9) {
          // Use the provided rotation matrix to compute proper Euler angles
          const m = camera.rotation; // [m00,m01,m02,m10,m11,m12,m20,m21,m22]

          // Extract Euler angles from rotation matrix (ZYX order for Three.js)
          const sy = Math.sqrt(m[0] * m[0] + m[3] * m[3]);
          const singular = sy < 1e-6;

          if (!singular) {
            // Standard case
            rotationEuler = [
              Math.atan2(m[5], m[8]), // X rotation (pitch)
              Math.atan2(-m[2], sy), // Y rotation (yaw)
              Math.atan2(m[1], m[0]), // Z rotation (roll)
            ];
          } else {
            // Gimbal lock case
            rotationEuler = [
              Math.atan2(-m[7], m[4]), // X rotation
              Math.atan2(-m[2], sy), // Y rotation
              0, // Z rotation
            ];
          }

          // REMOVE the extra 180-degree flip - the rotation matrix should be correct as-is
          // The camera plane should directly use the camera's rotation matrix
        } else if (
          camera.rx !== undefined &&
          camera.ry !== undefined &&
          camera.rz !== undefined
        ) {
          // Legacy format: use provided Euler angles
          rotationEuler = [camera.rx, camera.ry, camera.rz];
        } else {
          // Default orientation
          rotationEuler = [0, 0, 0];
        }

        // Debug log rotation values for the first few cameras
        if (index < 3) {
          console.log(
            `Camera ${index}: rotation=[${rotationEuler[0].toFixed(
              3
            )}, ${rotationEuler[1].toFixed(3)}, ${rotationEuler[2].toFixed(
              3
            )}]`,
            camera.viewDirection
              ? `viewDir=[${camera.viewDirection
                  .map((v: number) => v.toFixed(3))
                  .join(", ")}]`
              : "no viewDir",
            camera.rotation
              ? `rotMatrix=[${camera.rotation
                  .slice(0, 3)
                  .map((v: number) => v.toFixed(3))
                  .join(", ")}, ...]`
              : "no rotMatrix"
          );
        }

        const isSelected = selectedFrameId === index;

        return (
          <group key={index} position={[camera.x, camera.y, camera.z]}>
            {hasTexture ? (
              // Camera as textured plane showing the video frame
              <>
                <mesh
                  rotation={rotationEuler} // Apply rotation to the mesh directly
                  onClick={() => onFrameClick && onFrameClick(index)}
                  onPointerOver={(e) => {
                    e.stopPropagation();
                    document.body.style.cursor = 'pointer';
                  }}
                  onPointerOut={() => {
                    document.body.style.cursor = 'auto';
                  }}
                >
                  <planeGeometry args={[frameSize * aspectRatio, frameSize]} />
                  <meshBasicMaterial
                    map={textures[index]}
                    transparent={true}
                    alphaTest={0.1}
                    side={THREE.DoubleSide} // Make it visible from both sides
                  />
                  
                  {/* Selection highlight */}
                  {isSelected && (
                    <meshBasicMaterial
                      color="#00ff00"
                      transparent={true}
                      opacity={0.3}
                      side={THREE.DoubleSide}
                    />
                  )}
                </mesh>

                {/* Selection border for selected frame */}
                {isSelected && (
                  <lineSegments rotation={rotationEuler}>
                    <edgesGeometry attach="geometry" args={[new THREE.PlaneGeometry(frameSize * aspectRatio, frameSize)]} />
                    <lineBasicMaterial attach="material" color="#00ff00" linewidth={3} />
                  </lineSegments>
                )}

                {/* Add a small indicator for camera direction */}
                <mesh
                  position={[0, 0, -frameSize * 0.1]}
                  rotation={rotationEuler}
                >
                  <coneGeometry
                    args={[frameSize * 0.05, frameSize * 0.15, 6]}
                  />
                  <meshBasicMaterial
                    color={isSelected ? "#00ff00" : "#ff0000"}
                    transparent={true}
                    opacity={0.8}
                  />
                </mesh>

                {/* Add axis indicators to show orientation */}
                <group rotation={rotationEuler}>
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
              <group rotation={rotationEuler}>
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
  selectedFrameId?: number | null;
  userCamTransform?: {
    offset: [number, number, number];
    lookAt: [number, number, number];
    useFrameRotation?: boolean;
  };
}

const AutoCameraControls: React.FC<AutoCameraControlsProps> = ({
  cameraPositions,
  isProcessing,
  autoFollow,
  onUserInteraction,
  selectedFrameId,
  userCamTransform = { offset: [0, 0, -5], lookAt: [0, 0, -1], useFrameRotation: true },
}) => {
  const controlsRef = useRef<any>(null);
  const [hasUserInteracted, setHasUserInteracted] = useState(false);
  
  // Target positions for smooth animation
  const targetPositionRef = useRef<THREE.Vector3 | null>(null);
  const targetLookAtRef = useRef<THREE.Vector3 | null>(null);
  const [isAnimatingToFrame, setIsAnimatingToFrame] = useState(false);

  // Shared function to calculate user camera position and look-at from a frame
  const calculateCameraFromFrame = useCallback((frameCamera: Point3D) => {
    // Position camera at offset in the frame's local coordinate system
    let cameraPos = new THREE.Vector3(
      frameCamera.x + userCamTransform.offset[0],
      frameCamera.y + userCamTransform.offset[1], 
      frameCamera.z + userCamTransform.offset[2]
    );
    
    // Calculate look-at point - look in the same direction as the frame
    let lookAtPos = new THREE.Vector3(frameCamera.x, frameCamera.y, frameCamera.z);
    
    // If the frame has a rotation matrix, transform the offset and look direction
    if (frameCamera.rotation && frameCamera.rotation.length === 9) {
      const rotMatrix = new THREE.Matrix3().fromArray(frameCamera.rotation);
      const rotMatrix4 = new THREE.Matrix4().setFromMatrix3(rotMatrix);
      
      // Transform the offset by the frame's rotation matrix
      const localOffset = new THREE.Vector3(
        userCamTransform.offset[0], 
        userCamTransform.offset[1], 
        userCamTransform.offset[2]
      );
      localOffset.applyMatrix4(rotMatrix4);
      
      // Position the user camera in world space
      cameraPos = new THREE.Vector3().copy(new THREE.Vector3(frameCamera.x, frameCamera.y, frameCamera.z)).add(localOffset);
      
      // Look in the frame's viewing direction (perpendicular to the plane)
      if (frameCamera.viewDirection && frameCamera.viewDirection.length === 3) {
        // Use the frame's actual viewing direction
        const viewDirection = new THREE.Vector3(
          frameCamera.viewDirection[0],
          frameCamera.viewDirection[1],
          frameCamera.viewDirection[2]
        );
        lookAtPos = new THREE.Vector3().copy(cameraPos).add(viewDirection.multiplyScalar(10.0));
      } else {
        // Fallback: look forward in the frame's coordinate system (transform [0, 0, -1])
        const forwardDir = new THREE.Vector3(0, 0, -1);
        forwardDir.applyMatrix4(rotMatrix4);
        lookAtPos = new THREE.Vector3().copy(cameraPos).add(forwardDir.multiplyScalar(10.0));
      }
    }
    
    return { cameraPos, lookAtPos };
  }, [userCamTransform]);

  // Handle user interaction detection
  useEffect(() => {
    if (controlsRef.current && !hasUserInteracted) {
      const controls = controlsRef.current;

      const handleStart = () => {
        setHasUserInteracted(true);
        onUserInteraction();
        // Stop any ongoing frame animation when user interacts
        setIsAnimatingToFrame(false);
        targetPositionRef.current = null;
        targetLookAtRef.current = null;
      };

      controls.addEventListener("start", handleStart);
      return () => controls.removeEventListener("start", handleStart);
    }
  }, [hasUserInteracted, onUserInteraction]);

  // Handle frame selection snap-to by setting target positions
  useEffect(() => {
    if (selectedFrameId !== null && selectedFrameId !== undefined && cameraPositions[selectedFrameId] && controlsRef.current) {
      const selectedCamera = cameraPositions[selectedFrameId];
      
      // Use shared function to calculate camera position and look-at
      const { cameraPos, lookAtPos } = calculateCameraFromFrame(selectedCamera);
      
      // Set target positions for smooth animation
      targetPositionRef.current = cameraPos;
      targetLookAtRef.current = lookAtPos;
      setIsAnimatingToFrame(true);
      
      // Reset user interaction flag so they can interact again
      setHasUserInteracted(false);
    }
  }, [selectedFrameId, cameraPositions, userCamTransform, calculateCameraFromFrame]);

  useFrame(() => {
    if (!controlsRef.current) return;
    
    const controls = controlsRef.current;
    
    // Handle frame selection animation (highest priority)
    if (isAnimatingToFrame && targetPositionRef.current && targetLookAtRef.current) {
      const currentPos = controls.object.position;
      const currentTarget = controls.target;
      
      // Smooth interpolation towards target
      const lerpFactor = 0.08; // Smooth animation speed
      currentPos.lerp(targetPositionRef.current, lerpFactor);
      currentTarget.lerp(targetLookAtRef.current, lerpFactor);
      
      // Check if we're close enough to stop animating
      const positionDistance = currentPos.distanceTo(targetPositionRef.current);
      const targetDistance = currentTarget.distanceTo(targetLookAtRef.current);
      
      if (positionDistance < 0.01 && targetDistance < 0.01) {
        // Animation complete - snap to final position
        currentPos.copy(targetPositionRef.current);
        currentTarget.copy(targetLookAtRef.current);
        setIsAnimatingToFrame(false);
        targetPositionRef.current = null;
        targetLookAtRef.current = null;
      }
      
      controls.update();
      return; // Don't do auto-follow when animating to frame
    }
    
    // Auto-follow logic (only when not animating to frame and no frame selected)
    if (
      autoFollow &&
      !hasUserInteracted &&
      isProcessing &&
      selectedFrameId === null &&
      cameraPositions.length > 0
    ) {
      const latestCamera = cameraPositions[cameraPositions.length - 1];
      
      // Use the same shared function for consistent behavior
      const { cameraPos: targetPosition, lookAtPos: targetLookAt } = calculateCameraFromFrame(latestCamera);

      // Smooth interpolation for auto-follow
      const currentPos = controls.object.position;
      const currentTarget = controls.target;
      const positionDistance = currentPos.distanceTo(targetPosition);
      const targetDistance = currentTarget.distanceTo(targetLookAt);

      const baseSpeed = 0.12;
      const maxSpeed = 0.25;
      const minSpeed = 0.08;

      const positionSpeed = Math.min(maxSpeed, Math.max(minSpeed, baseSpeed + positionDistance * 0.02));
      const lookAtSpeed = Math.min(maxSpeed, Math.max(minSpeed, baseSpeed + targetDistance * 0.02));

      currentPos.lerp(targetPosition, positionSpeed);
      currentTarget.lerp(targetLookAt, lookAtSpeed);
      controls.update();
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
  onFrameClick,
  selectedFrameId = null,
  userCamTransform = { offset: [0, 0, -5], lookAt: [0, 0, -1], useFrameRotation: true },
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
          <CameraMesh 
            positions={cameraPositions} 
            videoFrames={videoFrames} 
            onFrameClick={onFrameClick}
            selectedFrameId={selectedFrameId}
          />
        )}

        {/* Controls */}
        <AutoCameraControls
          cameraPositions={cameraPositions}
          isProcessing={isProcessing}
          autoFollow={autoFollow}
          onUserInteraction={handleUserInteraction}
          selectedFrameId={selectedFrameId}
          userCamTransform={userCamTransform}
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
