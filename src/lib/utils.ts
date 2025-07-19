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
    video.muted = true; // Ensure video can autoplay if needed
    video.preload = "metadata";

    video.addEventListener("loadedmetadata", async () => {
      const duration = video.duration;

      // Ensure we have a valid duration
      if (!duration || duration <= 0) {
        reject(new Error("Invalid video duration"));
        return;
      }

      const frames: ImageData[] = [];

      console.log(
        `Video duration: ${duration} seconds, extracting ${maxFrames} frames`
      );

      // Create array of timestamps evenly spaced across the video duration
      const timestamps: number[] = [];
      if (maxFrames === 1) {
        timestamps.push(duration / 2); // Middle frame for single frame
      } else {
        for (let i = 0; i < maxFrames; i++) {
          // Space frames evenly from 0 to duration, leaving small margin at end
          const timestamp = (i / (maxFrames - 1)) * (duration * 0.95); // Use 95% of duration to avoid end issues
          timestamps.push(timestamp);
        }
      }

      console.log(
        `Frame timestamps:`,
        timestamps.map((t) => `${t.toFixed(2)}s`).join(", ")
      );

      let currentIndex = 0;

      const extractFrame = () => {
        if (currentIndex >= timestamps.length) {
          URL.revokeObjectURL(video.src);
          console.log(
            `Frame extraction complete: ${frames.length} frames extracted`
          );
          resolve(frames);
          return;
        }

        const timestamp = timestamps[currentIndex];
        console.log(
          `Extracting frame ${
            currentIndex + 1
          }/${maxFrames} at ${timestamp.toFixed(2)}s`
        );
        video.currentTime = timestamp;
      };

      video.addEventListener("seeked", () => {
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
        ctx.drawImage(video, 0, 0);

        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        frames.push(imageData);

        console.log(
          `Frame ${currentIndex + 1} extracted from ${video.currentTime.toFixed(
            2
          )}s`
        );

        currentIndex++;
        extractFrame();
      });

      // Add error handling for seeking
      video.addEventListener("error", (e) => {
        console.error("Video error during frame extraction:", e);
        reject(new Error("Video error during frame extraction"));
      });

      extractFrame();
    });
    
    video.addEventListener('error', () => {
      reject(new Error('Error loading video'))
    })
  })
}
