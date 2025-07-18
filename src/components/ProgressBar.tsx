import React from 'react'
import { ProcessingProgress } from '../types/photogrammetry'

interface ProgressBarProps {
  progress: ProcessingProgress
}

export const ProgressBar: React.FC<ProgressBarProps> = ({ progress }) => {
  return (
    <div className="space-y-2">
      <div className="flex justify-between text-sm">
        <span className="text-muted-foreground">Progress</span>
        <span className="font-medium">{progress.progress_percent.toFixed(1)}%</span>
      </div>
      
      <div className="w-full bg-muted rounded-full h-2">
        <div 
          className="bg-primary h-2 rounded-full progress-bar"
          style={{ width: `${progress.progress_percent}%` }}
        />
      </div>
      
      <div className="text-xs text-muted-foreground space-y-1">
        <div>Stage: {progress.stage.replace('_', ' ')}</div>
        <div>Frames: {progress.current_frame} / {progress.total_frames}</div>
        <div>Points detected: {progress.points_detected}</div>
        <div>Triangulated points: {progress.triangulated_points}</div>
      </div>
    </div>
  )
}
