"""
Continuous YOLOv8-Seg visualization on a single image.
Requirements:
    pip install ultralytics opencv-python
Press `q` in the display window to quit.
"""

import cv2
from ultralytics import YOLO

# ── 1. Load the pretrained segmentation model (tiny version = fast start) ──
model = YOLO("yolov8n-seg.pt")          # swap to yolov8s/m/l/x-seg.pt for higher accuracy

# ── 2. Read the image you want to segment ──
img_path = "example.png"
orig      = cv2.imread(img_path)
assert orig is not None, f"Image not found at {img_path}"

# ── 3. Run inference once (static image) ──
result    = model(orig, verbose=False)[0]    # result is ultralytics.engine.results.Results

annotated = result.plot()                    # adds masks, boxes, labels on a copy

# ── 4. Continuously show the annotated frame ──
while True:
    cv2.imshow("YOLOv8 Segmentation (press q to exit)", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
