from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch
 # load a pretrained model (recommended for training)

# Use the model
model.train(data="config.yaml", epochs=800)  # train the model
