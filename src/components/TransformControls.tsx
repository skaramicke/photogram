import React from 'react';
import './TransformControls.css';

interface TransformControlsProps {
  transformControls: {
    posX: number;
    posY: number;
    posZ: number;
    rotR0: number;
    rotR1: number;
    rotR2: number;
    userCamOffset: [number, number, number];
    userCamLookAt: [number, number, number];
    useFrameRotation: boolean;
  };
  onTransformChange: (controls: any) => void;
  onResetToDefaults: () => void;
}

export const TransformControls: React.FC<TransformControlsProps> = ({
  transformControls,
  onTransformChange,
  onResetToDefaults,
}) => {
  const handlePositionChange = (axis: 'posX' | 'posY' | 'posZ', value: number) => {
    onTransformChange({
      ...transformControls,
      [axis]: value,
    });
  };

  const handleRotationChange = (row: 'rotR0' | 'rotR1' | 'rotR2', value: number) => {
    onTransformChange({
      ...transformControls,
      [row]: value,
    });
  };

  return (
    <div className="transform-controls">
      <div className="transform-section">
        <h4>Frame Position Transform</h4>
        <div className="slider-controls">
          <div className="slider-control">
            <label className="block text-xs text-muted-foreground mb-1">
              X Multiplier: {transformControls.posX.toFixed(2)}
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              value={transformControls.posX}
              onChange={(e) => handlePositionChange('posX', Number(e.target.value))}
              className="slider"
            />
          </div>
          <div className="slider-control">
            <label className="block text-xs text-muted-foreground mb-1">
              Y Multiplier: {transformControls.posY.toFixed(2)}
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              value={transformControls.posY}
              onChange={(e) => handlePositionChange('posY', Number(e.target.value))}
              className="slider"
            />
          </div>
          <div className="slider-control">
            <label className="block text-xs text-muted-foreground mb-1">
              Z Multiplier: {transformControls.posZ.toFixed(2)}
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              value={transformControls.posZ}
              onChange={(e) => handlePositionChange('posZ', Number(e.target.value))}
              className="slider"
            />
          </div>
        </div>
      </div>

      <div className="transform-section">
        <h4>Frame Rotation Matrix Transform</h4>
        <div className="rotation-sliders">
          <div className="slider-control">
            <label className="block text-xs text-muted-foreground mb-1">
              Row 1 Multiplier: {transformControls.rotR0.toFixed(2)}
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              value={transformControls.rotR0}
              onChange={(e) => handleRotationChange('rotR0', Number(e.target.value))}
              className="slider"
            />
          </div>
          <div className="slider-control">
            <label className="block text-xs text-muted-foreground mb-1">
              Row 2 Multiplier: {transformControls.rotR1.toFixed(2)}
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              value={transformControls.rotR1}
              onChange={(e) => handleRotationChange('rotR1', Number(e.target.value))}
              className="slider"
            />
          </div>
          <div className="slider-control">
            <label className="block text-xs text-muted-foreground mb-1">
              Row 3 Multiplier: {transformControls.rotR2.toFixed(2)}
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              value={transformControls.rotR2}
              onChange={(e) => handleRotationChange('rotR2', Number(e.target.value))}
              className="slider"
            />
          </div>
        </div>
      </div>

      <div className="transform-section">
        <h4>User Camera Follow Settings</h4>
                <div>
          <label className="block text-sm font-medium mb-2">
            User Camera Follow Settings
          </label>
          <div className="space-y-3">
            <div>
              <label className="block text-xs text-muted-foreground mb-1">
                Camera Offset (X, Y, Z)
              </label>
              <div className="grid grid-cols-3 gap-2">
                <input
                  type="number"
                  step="0.5"
                  value={transformControls.userCamOffset[0]}
                  onChange={(e) =>
                    onTransformChange({
                      ...transformControls,
                      userCamOffset: [
                        parseFloat(e.target.value) || 0,
                        transformControls.userCamOffset[1],
                        transformControls.userCamOffset[2],
                      ],
                    })
                  }
                  className="px-2 py-1 text-xs border rounded"
                />
                <input
                  type="number"
                  step="0.5"
                  value={transformControls.userCamOffset[1]}
                  onChange={(e) =>
                    onTransformChange({
                      ...transformControls,
                      userCamOffset: [
                        transformControls.userCamOffset[0],
                        parseFloat(e.target.value) || 0,
                        transformControls.userCamOffset[2],
                      ],
                    })
                  }
                  className="px-2 py-1 text-xs border rounded"
                />
                <input
                  type="number"
                  step="0.5"
                  value={transformControls.userCamOffset[2]}
                  onChange={(e) =>
                    onTransformChange({
                      ...transformControls,
                      userCamOffset: [
                        transformControls.userCamOffset[0],
                        transformControls.userCamOffset[1],
                        parseFloat(e.target.value) || 0,
                      ],
                    })
                  }
                  className="px-2 py-1 text-xs border rounded"
                />
              </div>
            </div>

            <div>
              <label className="block text-xs text-muted-foreground mb-1">
                Look Direction Mode
              </label>
              <div className="flex gap-2 mb-2">
                <label className="flex items-center gap-1 text-xs">
                  <input
                    type="radio"
                    name="lookMode"
                    checked={transformControls.useFrameRotation}
                    onChange={() =>
                      onTransformChange({
                        ...transformControls,
                        useFrameRotation: true,
                      })
                    }
                  />
                  Use Frame Rotation
                </label>
                <label className="flex items-center gap-1 text-xs">
                  <input
                    type="radio"
                    name="lookMode"
                    checked={!transformControls.useFrameRotation}
                    onChange={() =>
                      onTransformChange({
                        ...transformControls,
                        useFrameRotation: false,
                      })
                    }
                  />
                  Fixed Direction
                </label>
              </div>
            </div>

            <div>
              <label className="block text-xs text-muted-foreground mb-1">
                Look Direction (X, Y, Z)
              </label>
              <div className="grid grid-cols-3 gap-2">
                <input
                  type="number"
                  step="0.1"
                  value={transformControls.userCamLookAt[0]}
                  onChange={(e) =>
                    onTransformChange({
                      ...transformControls,
                      userCamLookAt: [
                        parseFloat(e.target.value) || 0,
                        transformControls.userCamLookAt[1],
                        transformControls.userCamLookAt[2],
                      ],
                    })
                  }
                  className="px-2 py-1 text-xs border rounded"
                />
                <input
                  type="number"
                  step="0.1"
                  value={transformControls.userCamLookAt[1]}
                  onChange={(e) =>
                    onTransformChange({
                      ...transformControls,
                      userCamLookAt: [
                        transformControls.userCamLookAt[0],
                        parseFloat(e.target.value) || 0,
                        transformControls.userCamLookAt[2],
                      ],
                    })
                  }
                  className="px-2 py-1 text-xs border rounded"
                />
                <input
                  type="number"
                  step="0.1"
                  value={transformControls.userCamLookAt[2]}
                  onChange={(e) =>
                    onTransformChange({
                      ...transformControls,
                      userCamLookAt: [
                        transformControls.userCamLookAt[0],
                        transformControls.userCamLookAt[1],
                        parseFloat(e.target.value) || 0,
                      ],
                    })
                  }
                  className="px-2 py-1 text-xs border rounded"
                />
              </div>
              <p className="text-xs text-muted-foreground mt-1">
                {transformControls.useFrameRotation 
                  ? "Direction rotated by each frame's orientation" 
                  : "Fixed direction in world space (try 0, 0, 0 to look at frame)"}
              </p>
            </div>
          </div>
        </div>
      </div>

      <div className="transform-actions">
        <button onClick={onResetToDefaults} className="reset-button">
          Reset to Defaults
        </button>
      </div>
    </div>
  );
};
