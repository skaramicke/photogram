#!/bin/bash

# Build script for Cloudflare Pages
set -e

echo "Installing dependencies..."
npm install

echo "Building WASM module..."
npm run build:wasm

echo "Building React app..."
npm run build

echo "Build complete! Output in dist/ directory"
