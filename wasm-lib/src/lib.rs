use wasm_bindgen::prelude::*;
use web_sys::console;

// This is like the `extern` block in C
#[wasm_bindgen]
extern "C" {
    // Bind the `alert` function from the browser
    fn alert(s: &str);
}

// Define a macro to provide `println!(..)` style syntax for `console.log` logging
macro_rules! log {
    ( $( $t:tt )* ) => {
        console::log_1(&format!( $( $t )* ).into());
    }
}

// Export a `greet` function from Rust to JavaScript
#[wasm_bindgen]
pub fn greet(name: &str) -> String {
    log!("Hello from Rust! Greeting {}", name);
    format!("Hello, {}! This message is from Rust compiled to WebAssembly.", name)
}

// Export a function that performs some computation
#[wasm_bindgen]
pub fn fibonacci(n: u32) -> u32 {
    log!("Computing fibonacci({}) in Rust", n);
    match n {
        0 => 0,
        1 => 1,
        _ => fibonacci(n - 1) + fibonacci(n - 2),
    }
}

// Export a function that processes an array
#[wasm_bindgen]
pub fn process_array(numbers: &[f64]) -> Vec<f64> {
    log!("Processing array of {} numbers in Rust", numbers.len());
    numbers.iter().map(|x| x * x).collect()
}

// Called when the wasm module is instantiated
#[wasm_bindgen(start)]
pub fn main() {
    log!("Rust WASM module loaded!");
}
