#!/usr/bin/env python3
"""Run the existing OpenVINO YOLO model on one saved Orbbec RGB test frame."""

import sys
from pathlib import Path

import cv2
from ultralytics import YOLO


def main():
    if len(sys.argv) != 4:
        raise SystemExit('usage: yolo_rgb_sample.py IMAGE MODEL_DIR TEMP_DIR')
    image_path, model_dir, temp_dir = map(Path, sys.argv[1:])
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f'Could not decode RGB image: {image_path}')

    alias = temp_dir / 'blue_block_openvino_model'
    alias.symlink_to(model_dir, target_is_directory=True)
    model = YOLO(str(alias), task='segment')
    result = model.predict(
        source=image, device='cpu', conf=0.1, iou=0.45, verbose=False)[0]
    detections = []
    if result.boxes is not None:
        for box in result.boxes:
            class_id = int(box.cls.item())
            detections.append({
                'class': str(result.names.get(class_id, class_id)),
                'confidence': round(float(box.conf.item()), 4),
            })
    print(f'image={image.shape[1]}x{image.shape[0]} detections={detections}')


if __name__ == '__main__':
    main()
