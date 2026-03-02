from ultralytics import YOLO

def main():
    model = YOLO("yolo11n.pt")
    # half=True
    path = model.export(format="onnx", imgsz=640, dynamic=True, simplify=True)
    print(f"Exported: {path}")

if __name__ == "__main__":
    main()