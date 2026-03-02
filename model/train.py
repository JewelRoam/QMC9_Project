from ultralytics import YOLO

def train_model():
    model = YOLO("yolo11n.pt")

    # Train on kitti dataset
    results = model.train(data="kitti.yaml", epochs=10, imgsz=640)

if __name__ == '__main__':
    train_model()