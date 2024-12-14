import cv2

# Use libcamera-v4l2 as the video source for Raspberry Pi Camera Module
cap = cv2.VideoCapture("/dev/video0")  # Make sure /dev/video0 corresponds to the Pi camera

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break
    cv2.imshow('Video Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()
