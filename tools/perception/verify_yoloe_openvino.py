#!/usr/bin/env python3
"""Reproduce the YOLOE-26 OpenVINO metadata/output contract offline.

Requires OpenVINO, Ultralytics and the external model bundle. The generated
image is synthetic and tests software compatibility, not target accuracy.
"""

from __future__ import annotations

import argparse
import hashlib
import tempfile
from pathlib import Path

import numpy as np
import yaml

from offline_test_data import synthetic_blue_block_bgr


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Directory containing XML/BIN, metadata, prompts and embeddings")
    parser.add_argument("--confidence", type=float, default=0.0005)
    args = parser.parse_args()

    root = Path(args.model).expanduser().resolve()
    metadata_path = root / "metadata.yaml"
    prompt_path = root / "prompts.txt"
    embedding_path = root / "prompt_embeddings.npz"
    xmls = sorted(p for p in root.glob("*.xml") if not p.name.startswith("._"))
    if len(xmls) != 1:
        raise SystemExit(f"expected exactly one non-AppleDouble XML model in {root}; found {len(xmls)}")
    xml = xmls[0]
    binary = xml.with_suffix(".bin")
    for required in (metadata_path, prompt_path, embedding_path, binary):
        if not required.is_file():
            raise SystemExit(f"missing model bundle file: {required}")

    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    names = {int(k): str(v) for k, v in metadata.get("names", {}).items()}
    prompts = [line.strip() for line in prompt_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if metadata.get("task") != "segment" or metadata.get("head") != "YOLOESegment26":
        raise SystemExit("metadata does not identify the exported YOLOE-26 segmentation head")
    if metadata.get("version") != "8.4.164":
        raise SystemExit(f"model export version must match the pinned Ultralytics version; got {metadata.get('version')!r}")
    if metadata.get("end2end") is not True:
        raise SystemExit("metadata must retain end2end: true for the [B,300,38] exported tensor")
    if list(names.values()) != prompts:
        raise SystemExit(f"prompt/name mismatch: prompts={prompts!r}, names={names!r}")

    with np.load(embedding_path, allow_pickle=False) as archive:
        embedding_names = [str(x) for x in archive["names"].tolist()]
        embeddings = archive["embeddings"]
    if embedding_names != prompts or embeddings.ndim != 3 or embeddings.shape[:2] != (1, len(prompts)):
        raise SystemExit(
            f"prompt embedding mismatch: names={embedding_names!r}, shape={embeddings.shape}")

    from openvino import Core

    graph = Core().read_model(str(xml), weights=str(binary))
    input_shapes = [tuple(map(int, port.get_shape())) for port in graph.inputs]
    output_shapes = [tuple(map(int, port.get_shape())) for port in graph.outputs]
    expected_outputs = {(1, 300, 38), (1, 32, 160, 160)}
    if input_shapes != [(1, 3, 640, 640)] or set(output_shapes) != expected_outputs:
        raise SystemExit(f"unexpected OpenVINO interface: inputs={input_shapes} outputs={output_shapes}")

    from ultralytics import YOLO

    # Ultralytics 8.4 requires the *_openvino_model directory suffix. Match the
    # ROS node's alias behavior without writing beside the source model bundle.
    with tempfile.TemporaryDirectory(prefix="lunar-yoloe-alias-") as tmp:
        alias = Path(tmp) / f"{xml.stem}_openvino_model"
        alias.symlink_to(root, target_is_directory=True)
        model = YOLO(str(alias), task=str(metadata["task"]))
        image = synthetic_blue_block_bgr()
        results = model.predict(source=image, device="cpu", conf=args.confidence, verbose=False)

        # Regression reproducer: the previous repository sidecar omitted only
        # the exported end2end flag. Ultralytics then mistook the 300 prediction
        # rows for 300 channels (mask_dim=294), conflicting with 32 prototype
        # channels. Keep this expected failure as executable evidence.
        legacy = Path(tmp) / "legacy_missing_end2end_openvino_model"
        legacy.mkdir()
        for asset in (xml, binary):
            (legacy / asset.name).symlink_to(asset)
        legacy_metadata = dict(metadata)
        legacy_metadata.pop("end2end", None)
        (legacy / "metadata.yaml").write_text(
            yaml.safe_dump(legacy_metadata, sort_keys=False), encoding="utf-8")
        try:
            YOLO(str(legacy), task=str(metadata["task"])).predict(
                source=np.zeros((640, 640, 3), dtype=np.uint8), device="cpu", verbose=False)
        except RuntimeError as exc:
            if "13x294" not in str(exc) or "32x25600" not in str(exc):
                raise SystemExit(f"legacy metadata failed for an unexpected reason: {exc}") from exc
            print(f"missing_end2end_reproduced={exc}")
        else:
            raise SystemExit("expected the legacy missing-end2end metadata mismatch, but it did not occur")

    if len(results) != 1 or results[0].boxes is None or len(results[0].boxes) == 0:
        raise SystemExit("synthetic blue-block drawing produced no boxes at the requested smoke threshold")
    result = results[0]
    for class_id in result.boxes.cls.cpu().numpy().astype(int).tolist():
        if class_id not in names:
            raise SystemExit(f"model returned class id {class_id} absent from metadata")
    boxes = result.boxes.data.cpu().numpy()
    image_hash = hashlib.sha256(image.tobytes()).hexdigest()
    print(f"ultralytics={__import__('ultralytics').__version__} openvino={__import__('openvino').__version__}")
    print(f"input_shapes={input_shapes} output_shapes={output_shapes}")
    print(f"task={metadata['task']} head={metadata['head']} end2end={metadata['end2end']}")
    print(f"prompts={prompts} embedding_shape={tuple(embeddings.shape)} synthetic_image_sha256={image_hash}")
    for asset in (xml, binary, metadata_path, prompt_path, embedding_path):
        print(f"sha256 {asset.name}={hashlib.sha256(asset.read_bytes()).hexdigest()}")
    print(f"detections={len(boxes)} names={names}")
    print(f"first_box_xyxy_conf_class={boxes[0].tolist()}")
    print(f"masks_shape={None if result.masks is None else tuple(result.masks.data.shape)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
