export function cn(...inputs: string[]) {
  return inputs.filter(Boolean).join(' ')
}

export function formatFileSize(bytes: number): string {
  if (bytes === 0) return '0 Bytes'
  const k = 1024
  const sizes = ['Bytes', 'KB', 'MB', 'GB']
  const i = Math.floor(Math.log(bytes) / Math.log(k))
  return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i]
}

export function formatDuration(seconds: number): string {
  const hours = Math.floor(seconds / 3600)
  const minutes = Math.floor((seconds % 3600) / 60)
  const remainingSeconds = Math.floor(seconds % 60)
  
  if (hours > 0) {
    return `${hours}:${minutes.toString().padStart(2, '0')}:${remainingSeconds.toString().padStart(2, '0')}`
  }
  return `${minutes}:${remainingSeconds.toString().padStart(2, '0')}`
}

export function debounce<T extends (...args: any[]) => void>(
  func: T,
  delay: number
): (...args: Parameters<T>) => void {
  let timeoutId: ReturnType<typeof setTimeout>
  return (...args: Parameters<T>) => {
    clearTimeout(timeoutId)
    timeoutId = setTimeout(() => func(...args), delay)
  }
}

export function downloadFile(content: string, filename: string, contentType: string = 'text/plain'): void {
  const blob = new Blob([content], { type: contentType })
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.download = filename
  document.body.appendChild(link)
  link.click()
  document.body.removeChild(link)
  URL.revokeObjectURL(url)
}

export function extractVideoFrames(videoFile: File, maxFrames: number = 100): Promise<ImageData[]> {
  return new Promise((resolve, reject) => {
    const video = document.createElement('video')
    const canvas = document.createElement('canvas')
    const ctx = canvas.getContext('2d')
    
    if (!ctx) {
      reject(new Error('Could not get canvas context'))
      return
    }
    
    video.src = URL.createObjectURL(videoFile)
    video.addEventListener('loadedmetadata', () => {
      const duration = video.duration
      const frameInterval = duration / maxFrames
      const frames: ImageData[] = []
      let currentTime = 0
      
      const extractFrame = () => {
        if (frames.length >= maxFrames || currentTime >= duration) {
          URL.revokeObjectURL(video.src);
          resolve(frames);
          return;
        }
        
        video.currentTime = currentTime
      }
      
      video.addEventListener('seeked', () => {
        canvas.width = video.videoWidth
        canvas.height = video.videoHeight
        ctx.drawImage(video, 0, 0)
        
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height)
        frames.push(imageData)
        
        currentTime += frameInterval
        extractFrame()
      })
      
      extractFrame()
    })
    
    video.addEventListener('error', () => {
      reject(new Error('Error loading video'))
    })
  })
}
