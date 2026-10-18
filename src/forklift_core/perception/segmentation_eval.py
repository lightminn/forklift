# src/forklift_core/perception/segmentation_eval.py
"""How well instance proposals segment the truth instances (stage 3 plan, task 1).

Order (Codex, plan v2/v3): merges first -- a proposal covering >= MERGE_COVER of each of two or more truth
instances is a merge and leaves the matching (an exact union of two truths is IoU 0.5 with each, yet no match);
then a one-to-one maximum-IoU matching over the remaining proposals and the eligible truths with IoU >= MATCH_IOU
(ties in (truth id, proposal index) order); then splits -- an unmatched eligible truth whose pixels are covered
>= SPLIT_PIECE by two or more proposals together covering >= SPLIT_TOTAL. A proposal matched to a visible but
ineligible instance is reported apart; the rest are extra, by what they cover most: an instance, a distractor
(-2) or the background (-1). Truth ids are decoded ids (recognition_dataset), aligned to the depth grid.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, field

import numpy as np

MATCH_IOU = 0.5
MERGE_COVER = 0.30
SPLIT_PIECE = 0.05
SPLIT_TOTAL = 0.50


@dataclass(frozen=True)
class SegmentationResult:
    matched: dict            # truth id -> proposal index
    ious: dict               # truth id -> IoU of its match
    merges: list             # proposal indices
    splits: list             # truth ids
    missed: list             # eligible truth ids neither matched nor split
    ineligible_matched: list  # ineligible truth ids a proposal matched
    extra: dict = field(default_factory=dict)  # kind -> count
    eligible: int = 0

    @property
    def recall(self) -> float:
        return len(self.matched) / self.eligible if self.eligible else float("nan")


def _max_matching(pairs: dict, rows: list, cols: list) -> dict:
    """Maximum total IoU one-to-one matching (rows truth ids, cols proposal indices); ties by order."""
    best, best_key = {}, None

    def walk(i, used, chosen, total):
        nonlocal best, best_key
        if i == len(rows):
            key = (round(total, 12), tuple(-c for _, c in sorted(chosen.items())))
            if best_key is None or key > best_key:
                best, best_key = dict(chosen), key
            return
        r = rows[i]
        walk(i + 1, used, chosen, total)  # r unmatched
        for c in cols:
            if c not in used and (r, c) in pairs:
                chosen[r] = c
                walk(i + 1, used | {c}, chosen, total + pairs[(r, c)])
                del chosen[r]

    walk(0, frozenset(), {}, 0.0)
    return best


def evaluate_segmentation(proposals, truth_ids, *, eligible) -> SegmentationResult:
    ids = np.asarray(truth_ids)
    masks = [np.asarray(m, dtype=bool) for m in proposals]
    if any(m.shape != ids.shape for m in masks):
        raise ValueError("every proposal mask must have the truth image's shape")
    instances = sorted(int(v) for v in np.unique(ids) if v >= 0)
    area = {k: int((ids == k).sum()) for k in instances}
    inter = {(k, j): int((masks[j] & (ids == k)).sum()) for k in instances for j in range(len(masks))}
    merges = [j for j in range(len(masks))
              if sum(1 for k in instances if area[k] and inter[(k, j)] >= MERGE_COVER * area[k]) >= 2]
    free = [j for j in range(len(masks)) if j not in merges]
    iou = {}
    for k in instances:
        for j in free:
            union = area[k] + int(masks[j].sum()) - inter[(k, j)]
            if union and inter[(k, j)] / union >= MATCH_IOU:
                iou[(k, j)] = inter[(k, j)] / union
    eligible = sorted(set(int(e) for e in eligible) & set(instances)) if eligible is not None else instances
    matched = _max_matching(iou, eligible, free)
    taken = set(matched.values())
    ineligible = [k for k in instances if k not in eligible]
    ineligible_match = _max_matching(iou, ineligible, [j for j in free if j not in taken])
    taken |= set(ineligible_match.values())
    splits = []
    for k in eligible:
        if k in matched or not area[k]:
            continue
        pieces = [j for j in free if j not in taken and inter[(k, j)] >= SPLIT_PIECE * area[k]]
        if len(pieces) >= 2 and sum(inter[(k, j)] for j in pieces) >= SPLIT_TOTAL * area[k]:
            splits.append(k)
            taken |= set(pieces)
    extra: dict = {}
    for j in free:
        if j in taken:
            continue
        m = masks[j]
        counts = {"instance": int((m & (ids >= 0)).sum()), "distractor": int((m & (ids == -2)).sum()),
                  "background": int((m & (ids == -1)).sum())}
        kind = max(counts, key=lambda c: (counts[c], c == "background"))
        extra[kind] = extra.get(kind, 0) + 1
    missed = [k for k in eligible if k not in matched and k not in splits]
    return SegmentationResult(matched, {k: iou[(k, j)] for k, j in matched.items()}, merges, splits, missed,
                              sorted(ineligible_match), extra, len(eligible))


__all__ = ["MATCH_IOU", "MERGE_COVER", "SPLIT_PIECE", "SPLIT_TOTAL", "SegmentationResult", "evaluate_segmentation"]
