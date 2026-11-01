"""Timed scalar left-hand pulse after barcode alignment (no hardware I/O)."""
import math


def pulse_fraction(elapsed, opening, phase_duration):
    if not all(math.isfinite(v) for v in (elapsed, opening, phase_duration)) or not 0 < opening <= 1 or phase_duration <= 0:
        raise ValueError("invalid hand pulse")
    phase = max(0., elapsed) / phase_duration
    if phase >= 2.:
        return 1., True
    return 1. - opening * (phase if phase <= 1. else 2. - phase), False


def close_fraction(elapsed, start, duration):
    if not all(math.isfinite(v) for v in (elapsed, start, duration)) or not 0 <= start <= 1 or duration <= 0:
        raise ValueError("invalid hand close")
    progress = min(1., max(0., elapsed / duration))
    return start + (1. - start) * progress, progress >= 1.


def stage_left_fraction(stage, ready):
    if not math.isfinite(ready) or not 0 <= ready <= 1:
        raise ValueError("invalid left hand ready fraction")
    if stage in ('ClearReader','PlaceApproach','Place','Release','PlaceRetreat','Retreat','Recover'):
        return 0.0
    return ready if stage in ('Prepare', 'ReadyPose', 'Approach', 'Descend', 'TrackGrasp', 'Close', 'Lift', 'Scan', 'ScanPose', 'AlignBarcode') else 1.
