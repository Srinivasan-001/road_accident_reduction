import cv2
import numpy as np
import dlib
import time
import socket
import threading
from collections import OrderedDict
from scipy.spatial import distance
import torch
import torchvision
from ultralytics import YOLO

# Suppress potential warnings
import warnings

warnings.filterwarnings("ignore")

# Initialize YOLO model for object detection
try:
    model = YOLO('yolov8n.pt')
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    print("Please download the model from: https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt")
    exit(1)

# Initialize detector and predictor
detector = dlib.get_frontal_face_detector()
try:
    predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
except RuntimeError:
    print("Error: shape predictor file not found.")
    print("Please download from: http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2")
    exit(1)

# Constants for eye aspect ratio (EAR) - adjusted for better accuracy
EYE_AR_THRESH = 0.20  # Reduced threshold to avoid false positives
EYE_CONFIRM_FRAMES = 3  # Number of consecutive frames to confirm eye closure

# Constants for phone detection
PHONE_DETECTION_CONFIDENCE = 0.5


# Function to calculate eye aspect ratio
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear


# Function to get eye landmarks
def get_eye_landmarks(facial_landmarks):
    left_eye = [(facial_landmarks.part(n).x, facial_landmarks.part(n).y) for n in range(36, 42)]
    right_eye = [(facial_landmarks.part(n).x, facial_landmarks.part(n).y) for n in range(42, 48)]
    return left_eye, right_eye


# Function to detect phone use with YOLO
def detect_phone_use(frame):
    try:
        results = model(frame)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.cls[0] == 67 and box.conf[0] > PHONE_DETECTION_CONFIDENCE:
                    x1, y1, x2, y2 = box.xyxy[0]
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    return True
    except Exception as e:
        print(f"Phone detection error: {e}")
    return False


# Function to send alert to ESP8266
def send_alert(alert_type):
    try:
        ESP_IP = "192.168.16.151"  # Replace with your ESP8266 IP
        ESP_PORT = 4210  # Match ESP8266 UDP port

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Use UDP
        message = f"ALERT:{alert_type}"
        sock.sendto(message.encode(), (ESP_IP, ESP_PORT))

        print(f"Alert sent: {alert_type}")
    except Exception as e:
        print(f"Failed to send alert: {e}")


def start_monitoring():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Initialize state variables
    last_drowsy_time = 0
    last_phone_time = 0
    alert_cooldown = 3  # Cooldown period in seconds
    eye_closed_frames = 0  # Counter for consecutive closed eye frames
    closed_eyes_confirmed = False  # Flag to confirm actual eye closure

    # Calibration phase
    print("Calibrating eye detection... Please look at the camera normally.")
    ear_values = []

    # Collect EAR values for calibration
    for _ in range(30):  # Collect 30 frames of data
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)

        for face in faces:
            landmarks = predictor(gray, face)
            left_eye, right_eye = get_eye_landmarks(landmarks)
            left_ear = eye_aspect_ratio(left_eye)
            right_ear = eye_aspect_ratio(right_eye)
            ear = (left_ear + right_ear) / 2.0
            ear_values.append(ear)

        cv2.imshow('Calibration', frame)
        cv2.waitKey(1)

    # Calculate personalized threshold if enough data is collected
    if ear_values:
        # Set threshold to 70% of the average EAR value
        personal_threshold = sum(ear_values) / len(ear_values) * 0.7
        EYE_AR_THRESH = max(0.18, min(personal_threshold, 0.25))  # Keep within reasonable bounds
        print(f"Calibrated EAR threshold: {EYE_AR_THRESH}")
    else:
        print("Calibration failed, using default threshold")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)
        current_drowsy = False
        current_phone_use = False
        current_time = time.time()

        if len(faces) == 0:
            # Reset eye closed counter if no face is detected
            eye_closed_frames = 0
            closed_eyes_confirmed = False

            # Display status
            cv2.putText(frame, "No face detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            for face in faces:
                landmarks = predictor(gray, face)
                left_eye, right_eye = get_eye_landmarks(landmarks)
                left_ear = eye_aspect_ratio(left_eye)
                right_ear = eye_aspect_ratio(right_eye)
                ear = (left_ear + right_ear) / 2.0

                # Display current EAR value for debugging
                cv2.putText(frame, f"EAR: {ear:.2f}", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Improved Drowsiness Detection with confirmation
                if ear < EYE_AR_THRESH:
                    eye_closed_frames += 1
                    if eye_closed_frames >= EYE_CONFIRM_FRAMES:
                        closed_eyes_confirmed = True
                        cv2.putText(frame, "DROWSINESS ALERT!", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                        # Only send alert if cooldown has passed and eyes are closed
                        if current_time - last_drowsy_time > alert_cooldown and closed_eyes_confirmed:
                            threading.Thread(target=send_alert, args=("DROWSY",)).start()
                            last_drowsy_time = current_time
                            current_drowsy = True
                else:
                    eye_closed_frames = 0
                    closed_eyes_confirmed = False

                # Phone Usage Detection
                if detect_phone_use(frame):
                    current_phone_use = True
                    cv2.putText(frame, "PHONE USE DETECTED!", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    # Send alert if cooldown has passed
                    if current_time - last_phone_time > alert_cooldown:
                        threading.Thread(target=send_alert, args=("PHONE",)).start()
                        last_phone_time = current_time

                # Draw eye landmarks for visualization
                for (x, y) in left_eye:
                    cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)
                for (x, y) in right_eye:
                    cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)

        # Display the frame
        cv2.imshow('Driver Monitoring', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    print("System Requirements:")
    print("1. Ensure 'shape_predictor_68_face_landmarks.dat' is in the current directory")
    print("2. Ensure 'yolov8n.pt' model is downloaded")
    print("3. Make sure all required libraries are installed")

    try:
        start_monitoring()
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()