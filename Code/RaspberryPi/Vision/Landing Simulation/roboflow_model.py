import cv2 as cv
from roboflow import Roboflow
from ultralytics import yolo
from PIL import Image, ImageDraw, ImageFont

rf = Roboflow(api_key="PaXmw5uXSkFHgeZxBdtN")
project = rf.workspace().project("visionlanding")
model = project.version(1).model

# capture = cv.VideoCapture(1)
# frame_raw = cv.imread('TrainingImages/real/outdoor/mixed_overhead/IMG_6473.jpg')

while True:  # For video: capture.isOpened()

    # ret, frame_raw = capture.read()
    frame = cv.resize(frame_raw, (640, 480))

    prediction = model.predict(frame, confidence=40, overlap=30)
    detections = prediction.json()['predictions']
    # print(prediction.json())
    cv.imshow("prediction", prediction)

    !yolo task = detect \
            mode = train \
            model = yolov8s.pt \
            data = {dataset.location} / data.yaml \
            epochs = 100 \
            imgsz = 640

    # Press "k" to quit
    if cv.waitKey(27) == ord('k'):
        # capture.release()
        cv.destroyAllWindows()
        break
