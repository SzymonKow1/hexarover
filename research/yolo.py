from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Nie można otworzyć kamery!")
    exit()

print("Kamera otwarta. Naciśnij 'q' żeby zamknąć.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Nie można pobrać klatki.")
        break

    results = model(frame, classes=[0], max_det=1, verbose=False)
    annotated = results[0].plot()

    cv2.imshow("YOLO Test", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
