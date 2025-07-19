import React from 'react'
import { ProcessingProgress } from '../types/photogrammetry'

interface ProgressBarProps {
  progress: ProcessingProgress
}

export const ProgressBar: React.FC<ProgressBarProps> = ({ progress }) => {
  // Provide default values for missing fields
  const progressPercent = progress.progress_percent ?? 0;
  const currentFrame = progress.current_frame ?? 0;
  const totalFrames = progress.total_frames ?? 0;
  const pointsDetected = progress.points_detected ?? 0;
  const triangulatedPoints = progress.triangulated_points ?? 0;
  const stage = progress.stage ?? "initialized";

  return (
    <div className="space-y-2">
      <div className="flex justify-between text-sm">
        <span className="text-muted-foreground">Progress</span>
        <span className="font-medium">{progressPercent.toFixed(1)}%</span>
      </div>

      <div className="w-full bg-muted rounded-full h-2">
        <div
          className="bg-primary h-2 rounded-full progress-bar"
          style={{ width: `${progressPercent}%` }}
        />
      </div>

      <div className="text-xs text-muted-foreground space-y-1">
        <div>Stage: {stage.replace("_", " ")}</div>
        <div>
          Frames: {currentFrame} / {totalFrames}
        </div>
        <div>Points detected: {pointsDetected}</div>
        <div>Triangulated points: {triangulatedPoints}</div>
      </div>
    </div>
  );
}
