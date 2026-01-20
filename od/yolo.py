import cv2
import time
#import serial
from ultralytics import YOLO

#print("Using OpenCV from:", cv2.__file__)
#latency reduce attempts
MODEL_PATH = "yolov8n.pt"   
IMG_SIZE =640             # adjust input size
FRAME_SKIP=0               # 0 = run on every frame, 1 = run on every 2nd frame, 2 = every 3rd, etc
PERSON_ONLY =True           # True = only detect people
SENSOR_ID = 0

# Load YOLO model and move to GPU in half precision
model = YOLO(MODEL_PATH)
try:
    model.to("cuda")
    print("Model moved to CUDA")
except Exception as e:
    print("Warning: could not move model to CUDA, running on default device:", e)

# Your Jetson CSI GStreamer pipeline
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! appsink max-buffers=1 drop=true"
        % (sensor_id, capture_width, capture_height, framerate, flip_method)
    )
 
cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=SENSOR_ID), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

frame_id = 0
last_time = time.time()
fps_counter = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break

    frame_id += 1

    # Frame skipping for speed
    run_inference = True
    if FRAME_SKIP > 0 and (frame_id % (FRAME_SKIP + 1) != 0):
        run_inference = False
    
    person_detected=False #sets 

    if run_inference:
        # YOLO inference
        if PERSON_ONLY:
            results = model(frame, imgsz=IMG_SIZE, classes=[0])  # class 0 = person

        else:
            results = model(frame, imgsz=IMG_SIZE)

        annotated = results[0].plot()
        # Check detection of person
        if len(results[0].boxes) > 0:
            person_detected = True
    else:
        # If skipping this frame, just show raw frame
        annotated = frame
    #if person is detected send to serial
   # if person_detected:
    #    print("Detcted person")  
     #   ser.write(b"1\n")      # send True
   # else:
    #    ser.write(b"0\n")  #send False

    disp=cv2.resize(annotated,(1920,1080))#adjust the display size here
    cv2.imshow("YOLO Detection", disp)
    # FPS measurement
    fps_counter += 1
    current_time = time.time()
    if current_time - last_time >= 1.0:
        print(f"FPS (approx): {fps_counter}")
        fps_counter = 0
        last_time = current_time

    if cv2.waitKey(1) & 0xFF in (27, ord('q')):
        break

cap.release()
cv2.destroyAllWindows()

