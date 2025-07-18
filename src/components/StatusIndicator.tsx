import React from 'react'
import { ProcessingStage } from '../types/photogrammetry'

interface StatusIndicatorProps {
  status: ProcessingStage
  isWasmLoaded: boolean
}

export const StatusIndicator: React.FC<StatusIndicatorProps> = ({ status, isWasmLoaded }) => {
  const getStatusInfo = () => {
    if (!isWasmLoaded) {
      return { className: 'status-indicator idle', text: 'Loading WASM...' }
    }
    
    switch (status) {
      case 'idle':
        return { className: 'status-indicator idle', text: 'Ready' }
      case 'loading':
        return { className: 'status-indicator processing', text: 'Loading video...' }
      case 'frame_extraction':
        return { className: 'status-indicator processing', text: 'Extracting frames...' }
      case 'feature_detection':
        return { className: 'status-indicator processing', text: 'Detecting features...' }
      case 'matching':
        return { className: 'status-indicator processing', text: 'Matching features...' }
      case 'triangulation':
        return { className: 'status-indicator processing', text: 'Triangulating points...' }
      case 'bundle_adjustment':
        return { className: 'status-indicator processing', text: 'Bundle adjustment...' }
      case 'meshing':
        return { className: 'status-indicator processing', text: 'Generating mesh...' }
      case 'completed':
        return { className: 'status-indicator completed', text: 'Processing complete' }
      case 'error':
        return { className: 'status-indicator error', text: 'Error occurred' }
      default:
        return { className: 'status-indicator idle', text: 'Unknown' }
    }
  }

  const { className, text } = getStatusInfo()

  return (
    <div className="flex items-center">
      <span className={className}></span>
      <span className="text-sm text-muted-foreground">{text}</span>
    </div>
  )
}
