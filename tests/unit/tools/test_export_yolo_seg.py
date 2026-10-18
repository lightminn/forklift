"""The YOLO segmentation export of training renders (stage 3 plan, task 3)."""
import cv2
import numpy as np

from tools import export_yolo_seg as X


def _codes():
    codes = np.zeros((120, 160), np.uint16)
    codes[30:80, 20:70] = 1          # instance 0
    codes[30:80, 90:140] = 2         # instance 1, split by an occluder below
    codes[30:80, 112:118] = 65535    # a narrow distractor in front of instance 1
    return codes


def _raster(poly, shape):
    m = np.zeros(shape, np.uint8)
    cv2.fillPoly(m, [np.round(poly).astype(np.int32)], 1)
    return m.astype(bool)


def test_one_row_per_instance_even_when_occlusion_splits_it():
    # Codex v2: an occluded instance made two contours, i.e. two pallets in the labels
    lines = X.label_lines(_codes())
    assert len(lines) == 2 and all(line.startswith("0 ") for line in lines)


def test_the_joined_polygon_covers_the_instance():
    codes = _codes()
    for k, poly in X.instance_polygons(codes):
        truth = codes == k + 1
        got = _raster(poly, codes.shape)
        iou = (got & truth).sum() / (got | truth).sum()
        assert iou >= 0.9, (k, iou)


def test_distractors_and_background_get_no_rows():
    codes = np.zeros((50, 50), np.uint16)
    codes[10:20, 10:20] = 65535
    assert X.label_lines(codes) == []


def test_the_split_is_stable_by_scene_id():
    assert X.split_of("s1_epal6_5000001", 10) == X.split_of("s1_epal6_5000001", 10)
    share = np.mean([X.split_of(f"s{k}", 10) == "val" for k in range(2000)])
    assert 0.07 < share < 0.13


def test_export_writes_images_labels_and_data_yaml(tmp_path):
    from PIL import Image

    renders = tmp_path / "renders"
    for k in range(3):
        scene = renders / f"s1_epal6_{k}"
        scene.mkdir(parents=True)
        Image.fromarray(np.zeros((120, 160, 3), np.uint8)).save(scene / "rgb.png")
        Image.fromarray(_codes()).save(scene / "instances.png")
    counts = X.export(renders, tmp_path / "out")
    assert counts["train"] + counts["val"] == 3 and counts["instances"] == 6
    assert (tmp_path / "out" / "data.yaml").read_text().count("pallet") == 1
