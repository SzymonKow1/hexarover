import cv2
from ultralytics import YOLO

CAMERA_FOV_DEG = 60.0 # dostosuj tę wartość do specyfikacji swojej kamery

def calculate_angle(x_center, image_width, fov):
    center_of_camera = image_width / 2.0
    pixel_difference = x_center - center_of_camera
    angle_per_pixel = fov / image_width
    return pixel_difference * angle_per_pixel

def main():
    print("Loading YOLOv8 model...")
    model = YOLO("yolov8n.pt")

    # ZMIANA: Zamiast VIDEO_PATH, podajemy 0, aby użyć domyślnej kamery
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("BŁĄD: Nie można otworzyć kamery. Upewnij się, że jest podłączona i nie jest używana przez inny program.")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Nie można pobrać klatki z kamery.")
            break

        frame = cv2.resize(frame, (800, 600))
        image_width = frame.shape[1]

        results = model(frame, classes=[0], verbose=False)
        bounding_box = results[0].plot()

        # calculate angle
        for r in results:
            boxes = r.boxes
            if len(boxes) > 0:
                box = boxes[0]

                x_center, y_center, width, height = box.xywh[0].tolist()

                angle = calculate_angle(x_center, image_width, CAMERA_FOV_DEG)

                text = f"Angle: {angle:.1f} deg"
                cv2.putText(bounding_box, text, (int(x_center) - 50, int(y_center) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                print(f"Angle: {angle:.1f} deg")

        cv2.imshow("Human Angle to Bobik", bounding_box)

        # ZMIANA: cv2.waitKey(1) zamiast 30 zapewnia mniejsze opóźnienie przy podglądzie na żywo
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()