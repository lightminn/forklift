"""Whether a bring-up record may be judged (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-2b).

A commissioning run that left a protection out (``--bypass``), or whose conditions the record cannot tell (a real
commissioning record from before the bench limits, such as the 2026-11-24 diagnostic-image runs), is not evidence for
a gate. A large run (past duty 0.15 or 1.5 s on the open image) is marked but still judged: a no-load maximum speed
with the encoder fitted is such a run.

``record_marks(rows)`` reads a bring-up JSONL (all its runs). ``eligible(marks)`` says whether its official pass
fields may be filled. Judgement entry points refuse an ineligible record unless asked (``--allow-bypassed``), and
with that they keep the marks and leave every official pass field empty -- a field procedure reads only ``pass``.

The teleop bridge's records (``m_entry``, the M summary) carry no commissioning run and are not read here.
"""

from __future__ import annotations

from forklift_core.plant.write_log import runs_of

DIAG_TOKEN_PREFIX = "diag-"  # a diagnostic image's version token (diag-commission-unlimited-1, 2026-11-24)


def diagnostic_image(product: str | None) -> bool:
    """A USB product string naming a diagnostic image (its version token starts with ``diag-``): built off main with
    protections changed, so nothing armed on it is evidence for a gate (Codex PR-2b P2-5)."""
    return bool(product) and any(t.startswith(DIAG_TOKEN_PREFIX) for t in str(product).split())


class NotForGate(ValueError):
    """An ineligible record given to a judgement entry point without --allow-bypassed."""


def run_marks(run: list[dict]) -> dict:
    """The judgement marks of one run's rows (from its header on)."""
    header = next((r for r in run if r.get("t") == "header"), {})
    marks: dict = {}
    diag = next((r["product"] for r in run if r.get("product") and diagnostic_image(r["product"])), None)
    if diag is not None:
        # the product the link read (a connection or pre-ARM row): whatever the header says
        marks["not_for_gate"], marks["diagnostic_image"] = True, diag
    if header.get("cmd") != "commission":
        return marks
    if "large" not in header:
        # a commissioning run recorded before the bench limits (no bypass/large fields): its image and arguments
        # cannot be told from the record -- the 2026-11-24 diagnostic runs are such (design 1.2)
        marks["conditions_unknown"] = True
        return marks
    if header.get("bypass"):
        marks["not_for_gate"] = True
        marks["bypassed"] = [n.strip() for n in str(header["bypass"]).split(",") if n.strip()]
    if header.get("large"):
        marks["high_limit"] = True
    return marks


def record_marks(rows: list[dict]) -> dict:
    """Every run's marks of one JSONL file together (any ineligible run makes the file ineligible)."""
    out: dict = {}
    for run in runs_of(rows):
        for k, v in run_marks(run).items():
            if k == "bypassed":
                out[k] = sorted(set(out.get(k, [])) | set(v))
            else:
                out[k] = out.get(k, False) or v
    return out


def eligible(marks: dict) -> bool:
    return not (marks.get("not_for_gate") or marks.get("conditions_unknown"))


def check(marks: dict, *, allow: bool) -> dict:
    """The marks an output carries; NotForGate if the record is ineligible and not allowed."""
    if not eligible(marks) and not allow:
        why = ("a diagnostic image" if marks.get("diagnostic_image") else
               "a protection was bypassed" if marks.get("not_for_gate") else "its run conditions are unknown")
        raise NotForGate(f"not for a gate: {why} (give --allow-bypassed to read it; its pass fields stay empty)")
    return dict(marks)


def empty_pass_fields(external: dict) -> dict:
    """External verdict entries with their pass emptied (values and limits kept for reading)."""
    return {k: ({**v, "pass": None} if isinstance(v, dict) and "pass" in v else v) for k, v in external.items()}
