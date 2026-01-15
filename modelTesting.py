from ultralytics import YOLO
import cv2

# Load your custom trained model weights (e.g., 'best.pt' or 'last.pt')
# The 'best.pt' file is generated in the 'runs/detect/train/weights/' folder after training.
model = YOLO("best.pt")

# Define the source for detection (image file, video file, or '0' for webcam)
# Example sources: 'test_image.jpg', 'input_video.mp4', or 0 for live webcam
source = "enhanced_test_ir_image.png"

# Run inference on the source
# The 'predict' mode automatically handles detection and visualization.
results = model.predict(source, save=True, conf=0.45)

# Optional: Display the results in a window (for real-time or video)
if source.endswith('.mp4') or isinstance(source, int):
    for result in results:
        frame = result.plot() # plot the bounding boxes on the frame
        cv2.imshow("YOLO Custom Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

print("Detection complete. Results saved in 'runs/detect/predict' directory.")
