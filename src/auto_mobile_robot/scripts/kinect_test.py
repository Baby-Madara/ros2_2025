#!/usr/bin/env python3

import freenect
import numpy as np
import cv2

def get_video():
    """Capture the RGB video frame from Kinect."""
    frame, _ = freenect.sync_get_video()
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

def get_depth():
    """Capture the depth frame from Kinect."""
    depth, _ = freenect.sync_get_depth()
    depth_normalized = (depth / 2047.0 * 65535).astype(np.uint16)

    return depth_normalized

def main():
    while True:
        # Get frames
        rgb_frame = get_video()
        depth_frame = get_depth()

        # Display the frames
        cv2.imshow('Kinect RGB', rgb_frame)
        cv2.imshow('Kinect Depth', depth_frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
