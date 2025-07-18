import React from 'react'
import { ProcessingSettings } from '../types/photogrammetry'

interface SettingsPanelProps {
  settings: ProcessingSettings
  onSettingsChange: (settings: ProcessingSettings) => void
  onClose: () => void
}

export const SettingsPanel: React.FC<SettingsPanelProps> = ({
  settings,
  onSettingsChange,
  onClose
}) => {
  const handleSettingChange = (key: keyof ProcessingSettings, value: any) => {
    onSettingsChange({
      ...settings,
      [key]: value
    })
  }

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-card rounded-lg p-6 max-w-md w-full mx-4">
        <div className="flex justify-between items-center mb-4">
          <h2 className="text-lg font-semibold">Processing Settings</h2>
          <button
            onClick={onClose}
            className="text-muted-foreground hover:text-foreground"
          >
            ✕
          </button>
        </div>
        
        <div className="space-y-6">
          <div>
            <label className="block text-sm font-medium mb-2">
              Max Frames
            </label>
            <input
              type="number"
              value={settings.maxFrames}
              onChange={(e) => handleSettingChange('maxFrames', parseInt(e.target.value))}
              className="w-full px-3 py-2 bg-background border border-border rounded-lg"
              min="10"
              max="500"
            />
            <p className="text-xs text-muted-foreground mt-1">
              Maximum number of frames to extract from video
            </p>
          </div>

          <div>
            <label className="block text-sm font-medium mb-2">
              Keyframe Interval
            </label>
            <input
              type="number"
              value={settings.keyframeInterval}
              onChange={(e) => handleSettingChange('keyframeInterval', parseInt(e.target.value))}
              className="w-full px-3 py-2 bg-background border border-border rounded-lg"
              min="1"
              max="30"
            />
            <p className="text-xs text-muted-foreground mt-1">
              Frames between keyframes (lower = more detail, higher = faster)
            </p>
          </div>

          <div>
            <label className="block text-sm font-medium mb-2">
              Feature Detection Threshold
            </label>
            <input
              type="number"
              value={settings.featureDetectionThreshold}
              onChange={(e) => handleSettingChange('featureDetectionThreshold', parseFloat(e.target.value))}
              className="w-full px-3 py-2 bg-background border border-border rounded-lg"
              min="0.001"
              max="1"
              step="0.001"
            />
            <p className="text-xs text-muted-foreground mt-1">
              Sensitivity of feature detection (lower = more features)
            </p>
          </div>

          <div>
            <label className="block text-sm font-medium mb-2">
              Triangulation Threshold
            </label>
            <input
              type="number"
              value={settings.triangulationThreshold}
              onChange={(e) => handleSettingChange('triangulationThreshold', parseFloat(e.target.value))}
              className="w-full px-3 py-2 bg-background border border-border rounded-lg"
              min="0.01"
              max="10"
              step="0.01"
            />
            <p className="text-xs text-muted-foreground mt-1">
              Minimum confidence for triangulated points
            </p>
          </div>

          <div>
            <label className="block text-sm font-medium mb-2">
              Quality Preset
            </label>
            <select
              value={settings.quality}
              onChange={(e) => handleSettingChange('quality', e.target.value)}
              className="w-full px-3 py-2 bg-background border border-border rounded-lg"
            >
              <option value="low">Low (Fast)</option>
              <option value="medium">Medium (Balanced)</option>
              <option value="high">High (Slow)</option>
              <option value="ultra">Ultra (Very Slow)</option>
            </select>
            <p className="text-xs text-muted-foreground mt-1">
              Preset quality settings that adjust multiple parameters
            </p>
          </div>
        </div>

        <div className="flex justify-end space-x-2 mt-6">
          <button
            onClick={onClose}
            className="px-4 py-2 text-sm bg-secondary text-secondary-foreground rounded-lg hover:bg-accent"
          >
            Cancel
          </button>
          <button
            onClick={onClose}
            className="px-4 py-2 text-sm bg-primary text-primary-foreground rounded-lg hover:bg-primary/90"
          >
            Apply Settings
          </button>
        </div>
      </div>
    </div>
  )
}
