from ultralytics import YOLO

# Load a pretrained YOLO11m model
model = YOLO("../weights/yolo11m.pt")

# Train the model
train_results = model.train(
    data="../data/YOLODataset/dataset.yaml",  # Path to dataset configuration file
    epochs=100,  # Number of training epochs
    imgsz=640,  # Image size for training
    device="cpu",  # Device to run on (e.g., 'cpu', 0, [0,1,2,3])
)

# Evaluate the model's performance on the validation set
metrics = model.val()

# Export the model to ONNX format for deployment
path = model.export(format="onnx")  # Returns the path to the exported model
