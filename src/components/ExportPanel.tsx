import React, { useState } from 'react'
import { Point3D, ExportFormat } from '../types/photogrammetry'
import { downloadFile } from '../lib/utils'

interface ExportPanelProps {
  processor: any // PhotogrammetryProcessor type
  pointCloud: Point3D[]
  onClose: () => void
}

export const ExportPanel: React.FC<ExportPanelProps> = ({
  processor,
  pointCloud,
  onClose
}) => {
  const [exportFormat, setExportFormat] = useState<ExportFormat>({
    type: 'ply',
    includeColors: true,
    includeCameras: true,
    decimation: 1
  })

  const handleExport = () => {
    switch (exportFormat.type) {
      case 'ply':
        const plyContent = processor.export_ply()
        downloadFile(plyContent, 'pointcloud.ply', 'text/plain')
        break
      
      case 'obj':
        const objContent = generateObjContent(pointCloud)
        downloadFile(objContent, 'pointcloud.obj', 'text/plain')
        break
      
      case 'json':
        const jsonContent = JSON.stringify(pointCloud, null, 2)
        downloadFile(jsonContent, 'pointcloud.json', 'application/json')
        break
    }
    onClose()
  }

  const generateObjContent = (points: Point3D[]): string => {
    let content = '# Point Cloud Export\n'
    content += `# Generated on ${new Date().toISOString()}\n`
    content += `# Total points: ${points.length}\n\n`
    
    points.forEach(point => {
      content += `v ${point.x} ${point.y} ${point.z}\n`
    })
    
    return content
  }

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-card rounded-lg p-6 max-w-md w-full mx-4">
        <div className="flex justify-between items-center mb-4">
          <h2 className="text-lg font-semibold">Export Point Cloud</h2>
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
              Export Format
            </label>
            <select
              value={exportFormat.type}
              onChange={(e) => setExportFormat(prev => ({ ...prev, type: e.target.value as any }))}
              className="w-full px-3 py-2 bg-background border border-border rounded-lg"
            >
              <option value="ply">PLY (Point Cloud)</option>
              <option value="obj">OBJ (Wavefront)</option>
              <option value="json">JSON (Raw Data)</option>
            </select>
          </div>

          <div className="space-y-3">
            <div className="flex items-center space-x-2">
              <input
                type="checkbox"
                id="includeColors"
                checked={exportFormat.includeColors}
                onChange={(e) => setExportFormat(prev => ({ ...prev, includeColors: e.target.checked }))}
                className="rounded"
              />
              <label htmlFor="includeColors" className="text-sm">
                Include color information
              </label>
            </div>

            <div className="flex items-center space-x-2">
              <input
                type="checkbox"
                id="includeCameras"
                checked={exportFormat.includeCameras}
                onChange={(e) => setExportFormat(prev => ({ ...prev, includeCameras: e.target.checked }))}
                className="rounded"
              />
              <label htmlFor="includeCameras" className="text-sm">
                Include camera positions
              </label>
            </div>
          </div>

          <div>
            <label className="block text-sm font-medium mb-2">
              Decimation Factor
            </label>
            <input
              type="number"
              value={exportFormat.decimation}
              onChange={(e) => setExportFormat(prev => ({ ...prev, decimation: parseInt(e.target.value) }))}
              className="w-full px-3 py-2 bg-background border border-border rounded-lg"
              min="1"
              max="10"
            />
            <p className="text-xs text-muted-foreground mt-1">
              Export every Nth point (1 = all points, 2 = every other point, etc.)
            </p>
          </div>

          <div className="bg-muted rounded-lg p-3">
            <div className="text-sm space-y-1">
              <div>Points to export: {Math.floor(pointCloud.length / exportFormat.decimation)}</div>
              <div>Original points: {pointCloud.length}</div>
              <div>File format: {exportFormat.type.toUpperCase()}</div>
            </div>
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
            onClick={handleExport}
            className="px-4 py-2 text-sm bg-primary text-primary-foreground rounded-lg hover:bg-primary/90"
            disabled={pointCloud.length === 0}
          >
            Export
          </button>
        </div>
      </div>
    </div>
  )
}
