import cv2
import mediapipe as mp
import time

# Settings
maximum_time = 15  # Seconds
# Load Face Detector
face_detection = mp.solutions.face_detection.FaceDetection()
# Take frame from capera
cap = cv2.VideoCapture(0)
# Track TIME
starting_time = time.time()
while True:
    # Take frame from camera
    ret, frame = cap.read()
    height, width, channels = frame.shape
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    cv2.rectangle(frame, (0, 0), (width, 70), (10, 10, 10), -1)
    # Display frame
    #cv2.imshow("Frame", frame)
    # Face Detection
    results = face_detection.process(rgb_frame)
    elapsed_time = round(time.time() - starting_time, 3)

    # Is the face DETECTED?
    # if results.detections:
    #     print("Face looking at the screen")
    #     # Is the face DETECTED?
    if results.detections:
        # if elapsed_time > maximum_time:
        #     # Reached maximum time, show alert
        #     cv2.rectangle(frame, (0, 0), (width, height), (0, 0, 225), 10)
        #     cv2.setWindowProperty("Frame", cv2.WND_PROP_TOPMOST, 1)
        print("Face looking at the screen")
    else:
        print("NO FACE")
        # Reset the counter
        #starting_time = time.time()
        # Draw elapsed time on screen
    cv2.putText(frame, "{} seconds".format(elapsed_time), (10, 30), cv2.FONT_HERSHEY_PLAIN,
                2, (15, 225, 215), 2)
    cv2.putText(frame, "counter {}".format(elapsed_time), (10, 60), cv2.FONT_HERSHEY_PLAIN,
                2, (15, 225, 215), 2)
    print("Elapsed: {}".format(elapsed_time))
    # Display frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == 27:
        break