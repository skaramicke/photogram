# Photogrammetry Studio

A web-based photogrammetry application that converts video files into 3D point clouds and models using Rust WebAssembly for high-performance processing.

## Features

- **Video Upload**: Drag and drop video files (MP4, AVI, MOV, WMV)
- **Real-time Processing**: Frame extraction, feature detection, and triangulation
- **3D Visualization**: Interactive 3D scene viewer with point clouds and camera positions
- **Export Options**: Export point clouds in PLY, OBJ, or JSON formats
- **Quality Settings**: Adjustable processing parameters for different quality levels
- **WebAssembly Performance**: Heavy computational tasks run in Rust WASM for optimal speed

## Architecture

This project demonstrates a complete Rust WASM + React workflow:

- `wasm-lib/`: Rust library that compiles to WebAssembly
- `src/`: React frontend application
- `vite.config.ts`: Vite configuration for optimal WASM integration
- `build.sh`: Build script for Cloudflare Pages deployment

## Development

### Prerequisites

- Node.js 18+
- Rust 1.70+
- wasm-pack

### Setup

1. Install dependencies:
```bash
npm install
```

2. Build the WASM module:
```bash
npm run build:wasm
```

3. Start the development server:
```bash
npm run dev
```

### Building for Production

```bash
npm run build
```

This will:
1. Build the Rust WASM module
2. Compile TypeScript and React
3. Generate optimized build in `dist/`

## Cloudflare Pages Deployment

This project is configured for easy deployment on Cloudflare Pages:

### Automatic Deployment

1. Connect your GitHub repository to Cloudflare Pages
2. Use these build settings:
   - **Build command**: `npm run build`
   - **Build output directory**: `dist`
   - **Root directory**: `/`

### Manual Deployment

1. Build the project:
```bash
npm run build
```

2. Deploy the `dist/` directory to Cloudflare Pages

### Environment Variables

The build process requires:
- `NODE_VERSION`: `18`
- `RUST_VERSION`: `1.70.0` (if using Cloudflare's Rust support)

## WASM Module

The Rust WASM module (`wasm-lib/`) provides:

- `greet(name)`: Returns a greeting message
- `fibonacci(n)`: Calculates Fibonacci numbers
- `process_array(numbers)`: Squares an array of numbers

These functions demonstrate different data types and interaction patterns between Rust and JavaScript.

## Contributing

1. Fork the repository
2. Create a new branch
3. Make your changes
4. Test locally with `npm run dev`
5. Build with `npm run build`
6. Submit a pull request

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.