import cv2
import numpy as np

def find_camera():
    """Try to find the correct camera index."""
    for index in range(4):  # Test indices 0 to 3
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)  # Use V4L2 backend
        if cap.isOpened():
            print(f"Camera found at index {index}")
            return cap, index
        cap.release()
    return None, -1

# Initialize the USB camera
cap, camera_index = find_camera()
if cap is None:
    print("Error: Could not open any camera. Check connections and permissions.")
    print("Run 'ls /dev/video*' to verify camera devices.")
    print("Ensure user is in 'video' group: 'sudo usermod -aG video $USER'")
    exit(1)

# Set resolution to a safe default
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print(f"Camera initialized at index {camera_index}")
print(f"Resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

def process_frame(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Adaptive thresholding to detect dark patches (potential dirt)
    thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY_INV, 11, 2
    )
    
    # Morphological operations to clean up the threshold image
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
    thresh = cv2.dilate(thresh, kernel, iterations=1)
    
    # Find contours of potential dirt patches
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Sort contours by area (largest first) and limit to 10
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]
    
    # Count significant contours
    valid_contours = [c for c in contours if cv2.contourArea(c) > 100]
    num_patches = len(valid_contours)
    
    # Determine cleanliness status
    if num_patches > 5:
        status = "Dirty"
        status_color = (0, 0, 255)  # Red
    elif num_patches < 2:
        status = "Clean"
        status_color = (0, 255, 0)  # Green
    else:
        status = "Moderate"
        status_color = (0, 255, 255)  # Yellow
    
    # Draw contours and add labels
    for contour in valid_contours:
        cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)
        x, y, w, h = cv2.boundingRect(contour)
        print(f"Dirt detected at coordinates: Top-Left (x: {x}, y: {y}), Bottom-Right (x: {x+w}, y: {y+h})")
        cv2.putText(frame, f"Dirt (x:{x}, y:{y})", (x, y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Add status text to frame
    cv2.putText(frame, f"Status: {status} ({num_patches} patches)", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
    
    return frame, thresh

detecting = False

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame. Camera may be disconnected.")
            break
        
        print(f"Frame captured: {frame.shape if ret else 'None'}")
        
        if detecting:
            # Process the frame to detect dirt
            processed_frame, threshold = process_frame(frame)
            # Display the original frame with detected dirt
            cv2.imshow('Dirt Detection', processed_frame)
            # Display the threshold image for debugging
            cv2.imshow('Threshold', threshold)
        else:
            # Display the original frame without processing
            cv2.imshow('Dirt Detection', frame)
            # Only attempt to destroy the Threshold window if it exists
            if cv2.getWindowProperty('Threshold', cv2.WND_PROP_VISIBLE) >= 1:
                cv2.destroyWindow('Threshold')
        
        # Check for key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('1'):
            detecting = True
            print("Dirt detection started.")
        elif key == ord('2'):
            detecting = False
            print("Dirt detection stopped.")
        elif key == ord('q'):
            break

except KeyboardInterrupt:
    print("\nProgram terminated by user.")

finally:
    # Release the camera and close windows
    cap.release()
    cv2.destroyAllWindows()
