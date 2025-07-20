import React from 'react'
import { ProcessingStage } from "../types/photogrammetry";

interface ProcessingPanelProps {
  stage: ProcessingStage;
  frames: ImageData[];
}

export const ProcessingPanel: React.FC<ProcessingPanelProps> = ({ stage, frames }) => {
  const getStageDescription = () => {
    switch (stage) {
      case "frame_extraction":
        return "Extracting frames from video at regular intervals";
      case "feature_detection":
        return "Detecting key features in each frame using computer vision";
      case "matching":
        return "Matching features between frames to find correspondences";
      case "triangulation":
        return "Triangulating 3D points from matched features";
      case "bundle_adjustment":
        return "Optimizing camera poses and point positions";
      case "meshing":
        return "Generating 3D mesh from point cloud";
      case "completed":
        return "Processing completed successfully";
      case "error":
        return "An error occurred during processing";
      default:
        return "Processing...";
    }
  };

  return (
    <div className="bg-card rounded-lg p-6 card-shadow">
      <h2 className="text-lg font-semibold mb-4">Processing Status</h2>

      <div className="space-y-4">
        <div className="text-sm text-muted-foreground">
          {getStageDescription()}
        </div>

        {frames.length > 0 && (
          <div className="grid grid-cols-4 gap-2">
            {frames.slice(0, 8).map((frame, index) => (
              <div key={index} className="aspect-video bg-muted rounded border">
                <canvas
                  width={frame.width}
                  height={frame.height}
                  className="w-full h-full object-cover rounded"
                  ref={(canvas) => {
                    if (canvas) {
                      const ctx = canvas.getContext("2d");
                      if (ctx) {
                        ctx.putImageData(frame, 0, 0);
                      }
                    }
                  }}
                />
              </div>
            ))}
          </div>
        )}

        <div className="text-xs text-muted-foreground">
          {frames.length > 8 && `+${frames.length - 8} more frames`}
        </div>
      </div>
    </div>
  );
};
