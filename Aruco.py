import cv2
import cv2.aruco as aruco
import numpy as np

def detect_aruco_markers(frame):
    # Define the Aruco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    
    # Define parameters for the Aruco detector
    parameters = aruco.DetectorParameters_create()
    
    # Convert the frame to grayscale for detection
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejected_img_points = aruco.detectMarkers(gray_frame, aruco_dict, parameters=parameters)
    
    detected_markers = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            detected_markers[marker_id] = corners[i]
        # Draw detected markers on the frame
        aruco.drawDetectedMarkers(frame, corners, ids)
    else:
        print("No Aruco markers detected.")
    
    return detected_markers, frame

def main():
    # Open a video feed (use 0 for default camera, or replace with video file path)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Unable to access the camera or video feed.")
        return

    print("Press 'q' to quit the video feed.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read from camera.")
            break
        
        # Detect Aruco markers
        detected_markers, annotated_frame = detect_aruco_markers(frame)
        
        # Display the annotated frame
        cv2.imshow("Aruco Marker Detection", annotated_frame)
        
        # Print detected marker IDs
        for marker_id, corner in detected_markers.items():
            print(f"Marker ID: {marker_id}, Corners: {corner}")
        
        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
