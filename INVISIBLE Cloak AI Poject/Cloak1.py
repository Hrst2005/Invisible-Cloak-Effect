import cv2
import numpy as np
import time

# Open Lenovo external camera (index=1)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not cap.isOpened():
    raise RuntimeError("❌ Camera not opening. Check cable/permissions.")

# Set resolution for smooth processing
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)

print("[INFO] Capturing clean background... Remove cloak and stay still!")
time.sleep(3)

# Capture multiple frames and average to remove static artifacts
background_frames = []
for i in range(60):
    ret, frame = cap.read()
    if ret:
        frame = cv2.flip(frame, 1)
        background_frames.append(frame)

# Take average to smooth out any small motion/static noise
background = np.median(background_frames, axis=0).astype(dtype=np.uint8)

print("[INFO] Background captured ✅ Now wear red cloak!")

# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red cloak HSV range
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = cv2.bitwise_or(mask1, mask2)

    # Noise removal
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=1)
    mask_inv = cv2.bitwise_not(mask)

    # Replace cloak area with background
    cloak_area = cv2.bitwise_and(background, background, mask=mask)
    visible_area = cv2.bitwise_and(frame, frame, mask=mask_inv)
    output = cv2.add(cloak_area, visible_area)

    cv2.imshow("🪄 Harshit Invisible Cloak", output)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:  # Quit on 'q' or ESC
        break

cap.release()
cv2.destroyAllWindows()
