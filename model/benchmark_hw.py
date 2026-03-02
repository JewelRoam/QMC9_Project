import onnxruntime as ort
import numpy as np
import time
import os

# Configuration
MODEL_PATH = "yolo11n.onnx"      # Ensure the model filename is correct
ITERATIONS = 100                 # Number of test loops
WARMUP = 20                     # Warm-up iterations to exclude initialization overhead
INPUT_SHAPE = (1, 3, 640, 640)  # Default YOLOv11 input dimensions

def benchmark(provider_name, provider_options=None):
    print(f"\nInitializing: {provider_name}...")
    try:
        # 1. Instantiate the inference session
        session = ort.InferenceSession(
            MODEL_PATH, 
            providers=[provider_name] if provider_options is None else [(provider_name, provider_options)]
        )
        
        # 2. Prepare random input data (Stored in RAM to exclude disk I/O latency)
        input_name = session.get_inputs()[0].name
        dummy_input = np.random.random(INPUT_SHAPE).astype(np.float32)
        
        # 3. Warm-up
        # Hardware initialization and VRAM allocation usually occur during the first few inferences.
        # These must be excluded from the performance benchmark.
        for _ in range(WARMUP):
            session.run(None, {input_name: dummy_input})
            
        # 4. Official Benchmark Test
        start_time = time.perf_counter()
        for _ in range(ITERATIONS):
            session.run(None, {input_name: dummy_input})
        end_time = time.perf_counter()
        
        # 5. Calculate results
        total_time_ms = (end_time - start_time) * 1000
        avg_latency = total_time_ms / ITERATIONS
        fps = 1000 / avg_latency
        
        print(f"[{provider_name}] Average Latency: {avg_latency:.2f} ms")
        print(f"[{provider_name}] Throughput (FPS): {fps:.2f}")
        
    except Exception as e:
        print(f"Error: Could not start {provider_name}. Please check drivers or installation.")
        print(e)

if __name__ == "__main__":
    if not os.path.exists(MODEL_PATH):
        print(f"Error: Model file {MODEL_PATH} not found. Please run the export script first.")
    else:
        # Test 1: NVIDIA GPU (Using TensorRT acceleration)
        benchmark('TensorrtExecutionProvider', {
            'device_id': 0,
            'trt_fp16_enable': True,
        })

        # Test 2: NVIDIA GPU (Using CUDA acceleration)
        benchmark('CUDAExecutionProvider')

        # Test 3: Intel NPU (Using OpenVINO acceleration)
        benchmark('OpenVINOExecutionProvider', {
            'device_type': 'NPU',
            'precision': 'FP16',
            'cache_dir': './ov_cache'
        })

        # Test 4: CPU (Baseline for comparison)
        benchmark('CPUExecutionProvider')