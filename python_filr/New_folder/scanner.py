import cv2
import webbrowser
import tkinter as tk
import os
import pandas as pd
from tkinter import Label
from PIL import Image, ImageTk

def find_usb_camera():
    """Attempt to find the correct camera index for the USB camera."""
    for index in range(10):  # Check indices 0 to 9
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)  # Use CAP_V4L2 for Linux
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                cap.release()
                return index
        cap.release()
    return None

def start_scanner():
    global cap, running
    if running:
        return  # Prevent multiple instances
    running = True

    # Find the USB camera
    camera_index = find_usb_camera()
    if camera_index is None:
        qr_label.config(text="Error: No USB camera found", fg="red")
        running = False
        return

    # Initialize the camera
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        qr_label.config(text="Error: Could not open camera", fg="red")
        running = False
        return
    
    # Set camera properties (optional, adjust as needed)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    qr_label.config(text="Scanning for QR code...", fg="black")
    scan_qr()

def stop_scanner():
    global running, cap
    running = False
    if cap:
        cap.release()
        cap = None
    cv2.destroyAllWindows()
    qr_label.config(text="Scanner stopped", fg="black")

def scan_qr():
    global cap, running
    detector = cv2.QRCodeDetector()
    
    if not running:
        return
    
    ret, frame = cap.read()
    if not ret:
        qr_label.config(text="Error: Failed to capture frame", fg="red")
        running = False
        stop_scanner()
        return

    # Try QR code detection
    data, bbox, _ = detector.detectAndDecode(frame)
    if data:
        qr_label.config(text=f"QR Code: {data}", fg="green")
        webbrowser.open(data)
        running = False  # Stop scanning after successful detection
        stop_scanner()
        return
    
    # Draw a rectangle to indicate scanning area (for debugging)
    height, width = frame.shape[:2]
    top_left = (width//4, height//4)
    bottom_right = (3*width//4, 3*height//4)
    cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

    # Convert frame to display in Tkinter
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame)
    img = ImageTk.PhotoImage(img)
    video_label.imgtk = img
    video_label.configure(image=img)
    
    if running:
        video_label.after(10, scan_qr)  # Refresh frame

def open_patient_excel():
    excel_path = "patient.xlsx"
    if not os.path.exists(excel_path):
        df = pd.DataFrame(columns=["ID", "Name", "Age", "Condition"])
        df.to_excel(excel_path, index=False)
    os.system(f'start EXCEL.EXE "{excel_path}"')

# Create UI
top = tk.Tk()
top.title("QR Code Scanner")
top.geometry("640x500")

title_label = Label(top, text="QR Code Scanner", font=("Arial", 16))
title_label.pack()

video_label = Label(top)
video_label.pack()

qr_label = Label(top, text="Scan a QR Code", font=("Arial", 12), fg="black")
qr_label.pack()

start_button = tk.Button(top, text="Start", font=("Arial", 12), command=start_scanner)
start_button.pack()

stop_button = tk.Button(top, text="Stop", font=("Arial", 12), command=stop_scanner)
stop_button.pack()

excel_button = tk.Button(top, text="Open Patient Excel", font=("Arial", 12), command=open_patient_excel)
excel_button.pack()

running = False
cap = None

top.mainloop()
