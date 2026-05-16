#!/usr/bin/env python3

import cv2
import os

output_dir = "../media/materials/textures"
os.makedirs(output_dir, exist_ok=True)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h10)

marker_id = 0
marker_size_px = 600

img = cv2.aruco.generateImageMarker(
    aruco_dict,
    marker_id,
    marker_size_px
)

cv2.imwrite(os.path.join(output_dir, "tag_00000.png"), img)

print("Generated:", os.path.join(output_dir, "tag_00000.png"))
