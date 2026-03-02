import openvino as ov
import numpy as np

def test_npu_direct():
    try:
        core = ov.Core()
        # 随便定义一个超简单的模型 (1个输入 -> 1个ReLU)
        print("Creating dummy model...")
        import openvino.opset13 as ops
        param = ops.parameter([1, 3, 224, 224], name="input", dtype=np.float32)
        relu = ops.relu(param)
        model = ov.Model([relu], [param])

        print("Compiling model for NPU...")
        # 这一步最关键，如果 DLL 缺失，这里就会报错
        compiled_model = core.compile_model(model, "NPU")
        
        print("✅ SUCCESS: NPU is fully functional!")
    except Exception as e:
        print(f"❌ FAILED: {e}")

if __name__ == "__main__":
    test_npu_direct()