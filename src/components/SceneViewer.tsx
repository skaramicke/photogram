import React, { useRef, useEffect } from 'react'
import { Point3D, SceneView } from '../types/photogrammetry'

interface SceneViewerProps {
  pointCloud: Point3D[]
  cameraPositions: Point3D[]
  sceneView: SceneView
  isProcessing: boolean
}

export const SceneViewer: React.FC<SceneViewerProps> = ({
  pointCloud,
  cameraPositions,
  sceneView,
  isProcessing
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const animationRef = useRef<number>()
  const mouseRef = useRef({ x: 0, y: 0, down: false })
  const cameraRef = useRef({ x: 0, y: 0, z: 50, rotX: 0, rotY: 0 })

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const resizeCanvas = () => {
      const rect = canvas.getBoundingClientRect()
      canvas.width = rect.width
      canvas.height = rect.height
    }

    resizeCanvas()
    window.addEventListener('resize', resizeCanvas)

    const project3D = (x: number, y: number, z: number) => {
      const { x: cx, y: cy, z: cz, rotX, rotY } = cameraRef.current
      
      // Apply camera rotation
      const cosX = Math.cos(rotX)
      const sinX = Math.sin(rotX)
      const cosY = Math.cos(rotY)
      const sinY = Math.sin(rotY)
      
      // Translate to camera position
      const tx = x - cx
      const ty = y - cy
      const tz = z - cz
      
      // Rotate around Y axis
      const rx = tx * cosY - tz * sinY
      const rz = tx * sinY + tz * cosY
      
      // Rotate around X axis
      const ry = ty * cosX - rz * sinX
      const rfz = ty * sinX + rz * cosX
      
      // Project to 2D
      const distance = Math.max(rfz, 0.1)
      const scale = 800 / distance
      
      return {
        x: (rx * scale) + canvas.width / 2,
        y: (ry * scale) + canvas.height / 2,
        scale: scale
      }
    }

    const render = () => {
      ctx.clearRect(0, 0, canvas.width, canvas.height)
      
      // Draw background
      ctx.fillStyle = '#1a1a1a'
      ctx.fillRect(0, 0, canvas.width, canvas.height)
      
      if (isProcessing) {
        // Show processing animation
        const time = Date.now() * 0.001
        ctx.fillStyle = 'rgba(255, 255, 255, 0.1)'
        for (let i = 0; i < 20; i++) {
          const x = Math.sin(time + i * 0.5) * 100 + canvas.width / 2
          const y = Math.cos(time + i * 0.3) * 100 + canvas.height / 2
          ctx.beginPath()
          ctx.arc(x, y, 2, 0, Math.PI * 2)
          ctx.fill()
        }
      }
      
      // Draw point cloud
      if (sceneView.showPointCloud && pointCloud.length > 0) {
        pointCloud.forEach(point => {
          const projected = project3D(point.x, point.y, point.z)
          
          if (projected.x > 0 && projected.x < canvas.width && 
              projected.y > 0 && projected.y < canvas.height) {
            
            const size = Math.max(sceneView.pointSize * projected.scale * 0.01, 0.5)
            
            // Color based on mode
            let color = '#ffffff'
            if (sceneView.colorMode === 'height') {
              const normalized = (point.z + 10) / 20
              color = `hsl(${240 + normalized * 120}, 70%, 50%)`
            } else if (sceneView.colorMode === 'confidence') {
              const conf = Math.max(0, Math.min(1, point.confidence))
              color = `hsl(${120 * conf}, 70%, 50%)`
            }
            
            ctx.fillStyle = color
            ctx.beginPath()
            ctx.arc(projected.x, projected.y, size, 0, Math.PI * 2)
            ctx.fill()
          }
        })
      }
      
      // Draw camera positions
      if (sceneView.showCameras && cameraPositions.length > 0) {
        cameraPositions.forEach((camera, index) => {
          const projected = project3D(camera.x, camera.y, camera.z)
          
          if (projected.x > 0 && projected.x < canvas.width && 
              projected.y > 0 && projected.y < canvas.height) {
            
            const size = Math.max(8 * projected.scale * 0.01, 3)
            
            ctx.fillStyle = '#ff4444'
            ctx.beginPath()
            ctx.arc(projected.x, projected.y, size, 0, Math.PI * 2)
            ctx.fill()
            
            // Draw camera index
            ctx.fillStyle = '#ffffff'
            ctx.font = '12px monospace'
            ctx.textAlign = 'center'
            ctx.fillText(index.toString(), projected.x, projected.y - size - 5)
          }
        })
      }
      
      // Draw info
      ctx.fillStyle = '#ffffff'
      ctx.font = '14px monospace'
      ctx.textAlign = 'left'
      ctx.fillText(`Points: ${pointCloud.length}`, 10, 25)
      ctx.fillText(`Cameras: ${cameraPositions.length}`, 10, 45)
      ctx.fillText(`Camera: (${cameraRef.current.x.toFixed(1)}, ${cameraRef.current.y.toFixed(1)}, ${cameraRef.current.z.toFixed(1)})`, 10, 65)
      
      animationRef.current = requestAnimationFrame(render)
    }

    // Mouse controls
    const handleMouseDown = (e: MouseEvent) => {
      mouseRef.current = { x: e.clientX, y: e.clientY, down: true }
    }

    const handleMouseMove = (e: MouseEvent) => {
      if (mouseRef.current.down) {
        const dx = e.clientX - mouseRef.current.x
        const dy = e.clientY - mouseRef.current.y
        
        cameraRef.current.rotY += dx * 0.01
        cameraRef.current.rotX += dy * 0.01
        
        mouseRef.current.x = e.clientX
        mouseRef.current.y = e.clientY
      }
    }

    const handleMouseUp = () => {
      mouseRef.current.down = false
    }

    const handleWheel = (e: WheelEvent) => {
      e.preventDefault()
      const delta = e.deltaY * 0.01
      cameraRef.current.z = Math.max(10, Math.min(200, cameraRef.current.z + delta))
    }

    canvas.addEventListener('mousedown', handleMouseDown)
    canvas.addEventListener('mousemove', handleMouseMove)
    canvas.addEventListener('mouseup', handleMouseUp)
    canvas.addEventListener('wheel', handleWheel, { passive: false })

    render()

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current)
      }
      canvas.removeEventListener('mousedown', handleMouseDown)
      canvas.removeEventListener('mousemove', handleMouseMove)
      canvas.removeEventListener('mouseup', handleMouseUp)
      canvas.removeEventListener('wheel', handleWheel)
      window.removeEventListener('resize', resizeCanvas)
    }
  }, [pointCloud, cameraPositions, sceneView, isProcessing])

  return (
    <div className="relative w-full h-full">
      <canvas
        ref={canvasRef}
        className="w-full h-full three-canvas cursor-grab"
        style={{ cursor: mouseRef.current?.down ? 'grabbing' : 'grab' }}
      />
      
      <div className="view-controls">
        <div className="bg-card rounded-lg p-2 text-xs space-y-1">
          <div>Click and drag to rotate</div>
          <div>Scroll to zoom</div>
        </div>
      </div>
    </div>
  )
}
