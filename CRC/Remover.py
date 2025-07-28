import cv2
import numpy as np
from ultralytics import YOLO
import os

def process_workzone_image_cpu(image_path: str, output_dir: str = "output_workers_removed_cpu"):
    """
    Detects all workers and uses CPU-based aggressive inpainting to remove 
    non-target workers from the scene, blurring an area 10% larger than the object.
    
    Args:
        image_path (str): Path to the input image file.
        output_dir (str): Directory to save the output images.
    """
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output will be saved in the '{output_dir}/' directory.")
    print("Running on CPU...")

    # --- MODEL LOADING ON CPU ---
    try:
        model = YOLO('yolov8n-seg.pt')
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        return

    # --- IMAGE LOADING (CPU) ---
    img_cpu = cv2.imread(image_path)
    if img_cpu is None:
        print(f"Error: Could not read the image at '{image_path}'.")
        return
    
    H, W, _ = img_cpu.shape

    # --- INFERENCE AND MASK PROCESSING ON CPU ---
    results = model(img_cpu, conf=0.4, device='cpu') 
    
    persons = []
    if results[0].masks is not None:
        for i, mask in enumerate(results[0].masks.data):
            class_id = int(results[0].boxes.cls[i])
            if class_id == 0:  # 'person' class
                mask_np = mask.cpu().numpy().astype(np.uint8)
                binary_mask_cpu = cv2.resize(mask_np, (W, H), interpolation=cv2.INTER_NEAREST) * 255
                
                persons.append({
                    "box": results[0].boxes[i].xyxy[0].cpu().numpy(),
                    "mask": binary_mask_cpu,
                    "confidence": results[0].boxes[i].conf[0].item()
                })

    if not persons:
        print("No workers (persons) were detected in the image.")
        return

    print(f"Detected {len(persons)} workers. Processing each one on the CPU...")

    # --- CPU-BASED INPAINTING LOOP ---
    for i in range(len(persons)):
        target_person = persons[i]
        
        # Create a combined mask of ALL OTHER workers to be removed
        inpainting_mask = np.zeros((H, W), dtype=np.uint8)

        for j in range(len(persons)):
            if i == j:
                continue
            
            person_to_remove = persons[j]
            
            # --- ✨ NEW: ENLARGE MASK FOR THIS SPECIFIC PERSON BY 10% ---
            # 1. Get the bounding box to determine the object's size
            box = person_to_remove['box']
            x1, y1, x2, y2 = map(int, box)
            width = x2 - x1
            height = y2 - y1
            
            # 2. Calculate a dynamic dilation kernel size based on 10% of the object's smaller dimension
            kernel_size = int(min(width, height) * 0.20)
            
            # 3. Ensure kernel size is an odd number for cv2.getStructuringElement
            if kernel_size % 2 == 0:
                kernel_size += 1
            
            # 4. Create the dilation kernel and enlarge the specific person's mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            enlarged_mask = cv2.dilate(person_to_remove['mask'], kernel, iterations=1)
            
            # 5. Combine this individually enlarged mask into the main inpainting mask
            inpainting_mask = cv2.bitwise_or(inpainting_mask, enlarged_mask)
            
        # --- PERFORM INPAINTING ON CPU ---
        # The inpainting mask is now pre-enlarged for each object.
        inpaint_radius = 7
        inpainted_img = cv2.inpaint(img_cpu, inpainting_mask, inpaint_radius, cv2.INPAINT_NS)

        # --- RE-ADD THE TARGET WORKER FROM ORIGINAL IMAGE (ON CPU) ---
        final_img_cpu = inpainted_img.copy()
        target_mask_bool = target_person['mask'] == 255
        final_img_cpu[target_mask_bool] = img_cpu[target_mask_bool]

        # --- DRAW BOUNDING BOX AND SAVE (ON CPU) ---
        x1, y1, x2, y2 = map(int, target_person['box'])
        cv2.rectangle(final_img_cpu, (x1, y1), (x2, y2), (36, 255, 12), 2)
        label = f"Worker {i+1}: {target_person['confidence']:.2f}"
        cv2.putText(final_img_cpu, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (36, 255, 12), 2)

        output_path = os.path.join(output_dir, f"cpu_removed_worker_{i+1}.jpg")
        cv2.imwrite(output_path, final_img_cpu)

    print(f"\n✅ Processing complete. {len(persons)} images have been saved.")


if __name__ == "__main__":
    input_image_file = "Two.jpg"  # <-- CHANGE THIS TO YOUR IMAGE FILENAME

    if not os.path.exists(input_image_file):
        print(f"❌ Error: The input file '{input_image_file}' was not found.")
    else:
        process_workzone_image_cpu(image_path=input_image_file)