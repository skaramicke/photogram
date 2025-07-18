import React, { useCallback } from 'react'
import { VideoMetadata } from '../types/photogrammetry'
import { formatFileSize, formatDuration } from '../lib/utils'

interface VideoUploaderProps {
  onVideoUpload: (file: File) => void
  videoMetadata: VideoMetadata | null
  isProcessing: boolean
}

export const VideoUploader: React.FC<VideoUploaderProps> = ({
  onVideoUpload,
  videoMetadata,
  isProcessing
}) => {
  const handleDrop = useCallback((e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault()
    const files = Array.from(e.dataTransfer.files)
    const videoFile = files.find(file => file.type.startsWith('video/'))
    if (videoFile) {
      onVideoUpload(videoFile)
    }
  }, [onVideoUpload])

  const handleFileInput = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    if (file) {
      onVideoUpload(file)
    }
  }, [onVideoUpload])

  const handleDragOver = useCallback((e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault()
  }, [])

  return (
    <div>
      {!videoMetadata ? (
        <div
          onDrop={handleDrop}
          onDragOver={handleDragOver}
          className="dropzone p-8 text-center cursor-pointer"
        >
          <div className="space-y-4">
            <div className="text-4xl">📹</div>
            <div>
              <p className="text-lg font-medium mb-2">
                Drop your video file here
              </p>
              <p className="text-sm text-muted-foreground mb-4">
                or click to browse
              </p>
              <input
                type="file"
                accept="video/*"
                onChange={handleFileInput}
                className="hidden"
                id="video-upload"
                disabled={isProcessing}
              />
              <label
                htmlFor="video-upload"
                className="inline-block px-4 py-2 bg-primary text-primary-foreground rounded-lg hover:bg-primary/90 transition-colors cursor-pointer"
              >
                Choose Video File
              </label>
            </div>
            <p className="text-xs text-muted-foreground">
              Supported formats: MP4, AVI, MOV, WMV
            </p>
          </div>
        </div>
      ) : (
        <div className="space-y-4">
          <div className="flex items-center justify-between">
            <h3 className="font-medium">Video Loaded</h3>
            <button
              onClick={() => window.location.reload()}
              className="text-sm text-muted-foreground hover:text-foreground"
              disabled={isProcessing}
            >
              Change Video
            </button>
          </div>
          
          <div className="bg-muted rounded-lg p-4">
            <div className="grid grid-cols-2 gap-4 text-sm">
              <div>
                <span className="text-muted-foreground">Filename:</span>
                <p className="font-medium truncate">{videoMetadata.filename}</p>
              </div>
              <div>
                <span className="text-muted-foreground">Size:</span>
                <p className="font-medium">{formatFileSize(videoMetadata.size)}</p>
              </div>
              <div>
                <span className="text-muted-foreground">Duration:</span>
                <p className="font-medium">{formatDuration(videoMetadata.duration)}</p>
              </div>
              <div>
                <span className="text-muted-foreground">Resolution:</span>
                <p className="font-medium">{videoMetadata.width}x{videoMetadata.height}</p>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
