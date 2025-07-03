import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (1152,648)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

while True:
    im = picam2.capture_array()
    cv2.imshow("camera feed", im)
    if cv2.waitKey(1) == ord('q'):
        break






cv2.destroyAllWindows()

# # Use libcamera-v4l2 as the video source for Raspberry Pi Camera Module
# cap = cv2.VideoCapture(-1)  # Make sure /dev/video0 corresponds to the Pi camera

# if not cap.isOpened():
#     print("Error: Could not open video stream.")
#     exit()

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Error: Failed to capture image.")
#         break
#     cv2.imshow('Video Feed', frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
#         break

# cap.release()
# cv2.destroyAllWindows()
