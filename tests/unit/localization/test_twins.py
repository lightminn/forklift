"""The look-alike places of a placed start's preview among the shown global candidates (the lead's screen aid,
2026-10-31)."""

import math

from forklift_core.localization.twins import twin_count


def test_the_twins_of_a_preview_are_the_shown_look_alikes_away_from_it():
    a, b, c, d = (0.0, 0.0, 0.0), (0.0, 6.0, 0.0), (3.0, 0.0, 0.0), (0.0, 0.0, math.pi)
    cands = [(a, 0.90), (b, 0.89), (c, 0.50), (d, 0.885)]  # b and d within 0.98 of the top, c far below
    assert twin_count(cands, (0.05, 0.02, 0.01), 0.98) == 2  # the preview is a: b, and a turned around (d)
    assert twin_count(cands, (0.0, 6.02, 0.0), 0.98) == 2  # the preview is b: a and d
    assert twin_count(cands, (10.0, 10.0, 0.0), 0.98) == 3  # none is the preview: a, b and d
    assert twin_count(cands, (0.0, 0.0, 0.0), 1.0) == 0  # only the top itself reaches 1, and it is the preview
    assert twin_count([], (0.0, 0.0, 0.0), 0.98) is None  # no global candidates: not known
