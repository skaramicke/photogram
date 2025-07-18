#!/bin/bash
set -e

echo "=== Installing Node.js dependencies ==="
npm install

echo "=== Installing Rust ==="
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source $HOME/.cargo/env
rustc --version

echo "=== Adding wasm32 target ==="
rustup target add wasm32-unknown-unknown

echo "=== Installing wasm-pack ==="
curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh
wasm-pack --version

echo "=== Building WASM module ==="
npm run build:wasm

echo "=== Building React app ==="
npm run build:react

echo "=== Build complete! ==="
