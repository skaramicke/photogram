import React, { useRef, useEffect, useState, useMemo } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera, Html } from "@react-three/drei";
import { Point3D, SceneView } from "../types/photogrammetry";
import * as THREE from "three";

interface SceneViewerProps {
  pointCloud: Point3D[];
  cameraPositions: Point3D[];
  sceneView: SceneView;
  isProcessing: boolean;
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
}

const CameraMesh: React.FC<CameraMeshProps> = ({ positions }) => {
  const groupRef = useRef<THREE.Group>(null);

  return (
    <group ref={groupRef}>
      {positions.map((camera, index) => (
        <group key={index} position={[camera.x, camera.y, camera.z]}>
          {/* Camera body */}
          <mesh>
            <boxGeometry args={[0.2, 0.1, 0.1]} />
            <meshBasicMaterial color="#ff4444" />
          </mesh>

          {/* Camera lens */}
          <mesh position={[0.1, 0, 0]}>
            <cylinderGeometry args={[0.03, 0.03, 0.05, 8]} />
            <meshBasicMaterial color="#333333" />
          </mesh>

          {/* Frame number */}
          <Html distanceFactor={10}>
            <div className="text-white text-xs font-mono bg-black bg-opacity-50 px-1 rounded">
              {index}
            </div>
          </Html>
        </group>
      ))}
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
          <CameraMesh positions={cameraPositions} />
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
