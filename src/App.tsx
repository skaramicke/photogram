import { useState, useEffect } from 'react'
import init, { greet, fibonacci, process_array } from './wasm/photogram_wasm.js'
import './App.css'

function App() {
  const [wasmLoaded, setWasmLoaded] = useState(false)
  const [name, setName] = useState('World')
  const [greeting, setGreeting] = useState('')
  const [fibNumber, setFibNumber] = useState(10)
  const [fibResult, setFibResult] = useState<number | null>(null)
  const [numbers, setNumbers] = useState('1,2,3,4,5')
  const [squaredNumbers, setSquaredNumbers] = useState<number[]>([])

  useEffect(() => {
    init().then(() => {
      setWasmLoaded(true)
      console.log('WASM module loaded successfully!')
    }).catch((err: any) => {
      console.error('Failed to load WASM module:', err)
    })
  }, [])

  const handleGreet = () => {
    if (wasmLoaded) {
      const result = greet(name)
      setGreeting(result)
    }
  }

  const handleFibonacci = () => {
    if (wasmLoaded) {
      const result = fibonacci(fibNumber)
      setFibResult(result)
    }
  }

  const handleProcessArray = () => {
    if (wasmLoaded) {
      try {
        const numberArray = numbers.split(',').map((n: string) => parseFloat(n.trim())).filter((n: number) => !isNaN(n))
        const result = process_array(new Float64Array(numberArray))
        setSquaredNumbers(Array.from(result))
      } catch (err: any) {
        console.error('Error processing array:', err)
      }
    }
  }

  return (
    <div className="App">
      <header className="App-header">
        <h1>Photogram - Rust WASM Demo</h1>
        <p>
          {wasmLoaded ? 
            '✅ WebAssembly module loaded successfully!' : 
            '⏳ Loading WebAssembly module...'
          }
        </p>
      </header>

      <main>
        <section className="demo-section">
          <h2>Greeting Function</h2>
          <div className="input-group">
            <input
              type="text"
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="Enter your name"
            />
            <button onClick={handleGreet} disabled={!wasmLoaded}>
              Greet
            </button>
          </div>
          {greeting && <p className="result">{greeting}</p>}
        </section>

        <section className="demo-section">
          <h2>Fibonacci Calculator</h2>
          <div className="input-group">
            <input
              type="number"
              value={fibNumber}
              onChange={(e) => setFibNumber(parseInt(e.target.value) || 0)}
              min="0"
              max="40"
            />
            <button onClick={handleFibonacci} disabled={!wasmLoaded}>
              Calculate
            </button>
          </div>
          {fibResult !== null && (
            <p className="result">
              Fibonacci({fibNumber}) = {fibResult}
            </p>
          )}
        </section>

        <section className="demo-section">
          <h2>Array Processing (Square Numbers)</h2>
          <div className="input-group">
            <input
              type="text"
              value={numbers}
              onChange={(e) => setNumbers(e.target.value)}
              placeholder="Enter numbers separated by commas"
            />
            <button onClick={handleProcessArray} disabled={!wasmLoaded}>
              Process Array
            </button>
          </div>
          {squaredNumbers.length > 0 && (
            <p className="result">
              Squared: [{squaredNumbers.join(', ')}]
            </p>
          )}
        </section>
      </main>
    </div>
  )
}

export default App
