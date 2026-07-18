#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# sstv.py - SSTV encode/decode for kv4p-web.
# Copyright (C) 2026  https://github.com/Leproide
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.  See <https://www.gnu.org/licenses/>.
#
# Author: https://github.com/Leproide
#
# Encoding uses PySSTV (all color + B/W modes). Decoding is implemented here
# for the Martin M1/M2 modes (sequential G-B-R scan) via Hilbert FM demod and
# VIS-header auto-detect. Note: over the kv4p's lossy voice codec the recovered
# image quality can be poor; robust modes (Martin M1, Robot 36) fare best.

import io

import numpy as np
from PIL import Image

from pysstv.color import (MartinM1, MartinM2, Robot36, ScottieS1, ScottieS2,
                          PD90, PD120)
from pysstv.grayscale import Robot8BW, Robot24BW

# name -> (PySSTV class, VIS code)
ENCODE_MODES = {
    "Martin M1": MartinM1,
    "Martin M2": MartinM2,
    "Scottie S1": ScottieS1,
    "Scottie S2": ScottieS2,
    "Robot 36": Robot36,
    "PD90": PD90,
    "PD120": PD120,
    "Robot 8 B/W": Robot8BW,
    "Robot 24 B/W": Robot24BW,
}

FREQ_BLACK = 1500.0
FREQ_WHITE = 2300.0
FREQ_SYNC = 1200.0
FREQ_LEADER = 1900.0

# VIS code -> Martin decode parameters (only Martin family decoded for now)
_MARTIN_DECODE = {
    MartinM1.VIS_CODE: {"name": "Martin M1", "w": 320, "h": 256,
                        "sync": 4.862, "scan": 146.432, "gap": 0.572},
    MartinM2.VIS_CODE: {"name": "Martin M2", "w": 320, "h": 256,
                        "sync": 4.862, "scan": 73.216, "gap": 0.572},
}


# --------------------------------------------------------------------------
# Encoder
# --------------------------------------------------------------------------
def _draw_callsign(img, text):
    """Burn the callsign into the picture itself.

    The FSK-ID is only shown by decoders that enable it (and some ignore it),
    so painting the callsign on the image guarantees the receiving station
    can read it whatever software they run.
    """
    from PIL import ImageDraw, ImageFont
    w, h = img.size
    size = max(12, h // 12)
    font = None
    for cand in ("arial.ttf", "Arial.ttf", "DejaVuSans-Bold.ttf", "DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                 "C:\\Windows\\Fonts\\arialbd.ttf", "C:\\Windows\\Fonts\\arial.ttf"):
        try:
            font = ImageFont.truetype(cand, size)
            break
        except Exception:
            continue
    if font is None:
        font = ImageFont.load_default()
    draw = ImageDraw.Draw(img)
    text = str(text).upper()
    try:
        box = draw.textbbox((0, 0), text, font=font)
        tw, th = box[2] - box[0], box[3] - box[1]
    except Exception:
        tw, th = draw.textsize(text, font=font)
    x, y = max(2, w // 40), max(2, h // 40)
    # dark plate behind the text so it stays readable over any picture
    draw.rectangle([x - 4, y - 2, x + tw + 6, y + th + 8], fill=(0, 0, 0))
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        draw.text((x + dx, y + dy), text, font=font, fill=(0, 0, 0))
    draw.text((x, y), text, font=font, fill=(255, 255, 255))
    return img


def encode_image(image, mode_name, sample_rate, fskid=None, overlay=None):
    """Return int16 mono numpy array of the SSTV signal for the given image.

    `image` may be a path, a PIL.Image, or raw image bytes.
    `fskid` appends the standard FSK-ID callsign after the picture.
    `overlay` paints that callsign into the picture itself.
    """
    if isinstance(image, (bytes, bytearray)):
        img = Image.open(io.BytesIO(image))
    elif isinstance(image, Image.Image):
        img = image
    else:
        img = Image.open(image)
    cls = ENCODE_MODES[mode_name]
    img = img.convert("RGB").resize((cls.WIDTH, cls.HEIGHT))
    if overlay:
        img = _draw_callsign(img, overlay)
    enc = cls(img, sample_rate, 16)
    if fskid:
        # let a failure surface: silently dropping the ID is worse than an error
        enc.add_fskid_text(str(fskid).upper()[:20])
    return np.fromiter(enc.gen_samples(), dtype=np.int16)


def mode_seconds(mode_name, sample_rate=48000):
    cls = ENCODE_MODES[mode_name]
    img = Image.new("RGB", (cls.WIDTH, cls.HEIGHT))
    n = sum(1 for _ in cls(img, sample_rate, 16).gen_samples())
    return n / sample_rate


# --------------------------------------------------------------------------
# Decoder (Martin M1/M2)
# --------------------------------------------------------------------------
def _inst_freq(samples, sr):
    from scipy.signal import hilbert
    x = samples.astype(np.float64)
    if np.max(np.abs(x)) > 0:
        x = x / np.max(np.abs(x))
    analytic = hilbert(x)
    phase = np.unwrap(np.angle(analytic))
    f = np.diff(phase) / (2.0 * np.pi) * sr
    f = np.concatenate([f, f[-1:]])
    return f


def _f2val(f):
    v = (f - FREQ_BLACK) / (FREQ_WHITE - FREQ_BLACK) * 255.0
    return np.clip(v, 0, 255)


def _find_vis(freq, sr):
    """Locate the VIS header; return (vis_code, sample_index_after_stop) or None."""
    ms = sr / 1000.0
    win = int(30 * ms)  # VIS bit length
    n = len(freq)
    # find a long leader tone near 1900 Hz (>= ~250 ms)
    lead_len = int(250 * ms)
    i = 0
    step = max(1, int(2 * ms))
    while i < n - lead_len:
        seg = freq[i:i + lead_len]
        if np.mean(np.abs(seg - FREQ_LEADER) < 120) > 0.7:
            break
        i += step
    else:
        return None
    # after leader: 300ms leader, 10ms 1200 break, 300ms leader, then VIS start bit
    # scan forward for the 1200 Hz start bit (30 ms) following the second leader
    j = i + lead_len
    limit = min(n - win * 12, j + int(700 * ms))
    while j < limit:
        seg = freq[j:j + win]
        if np.mean(np.abs(seg - FREQ_SYNC) < 120) > 0.6:
            break
        j += int(ms)
    else:
        return None
    start = j + win  # first data bit starts after the start bit
    bits = []
    for b in range(8):  # 7 data + parity
        seg = freq[start + b * win: start + (b + 1) * win]
        mf = np.median(seg)
        bits.append(1 if mf < 1200 else 0)  # 1100=1, 1300=0
    code = 0
    for k in range(7):
        code |= (bits[k] & 1) << k
    stop = start + 8 * win + win  # + stop bit
    return code, stop


def decode(samples, sr, mode_hint=None):
    """Decode an SSTV signal (Martin M1/M2). Returns (PIL.Image, mode_name) or
    (None, reason)."""
    samples = np.asarray(samples)
    if samples.dtype != np.int16 and samples.dtype != np.float64:
        samples = samples.astype(np.float64)
    freq = _inst_freq(samples, sr)
    vis = _find_vis(freq, sr)
    if vis is None:
        return None, "no VIS header found"
    code, t0 = vis
    params = _MARTIN_DECODE.get(code)
    if params is None:
        return None, f"unsupported/undecoded mode (VIS 0x{code:02x})"
    ms = sr / 1000.0
    w, h = params["w"], params["h"]
    sync, scan, gap = params["sync"], params["scan"], params["gap"]
    line_ms = sync + 4 * gap + 3 * scan
    ms = sr / 1000.0
    w, h = params["w"], params["h"]
    sync, scan, gap = params["sync"], params["scan"], params["gap"]
    line_ms = sync + 4 * gap + 3 * scan
    fclip = np.clip(freq, 1000.0, 2500.0)
    n = len(fclip)
    # Short-window sync detector. We align on the END of the sync pulse (the
    # transition into the colour scan): the VIS trailer is also 1200 Hz and runs
    # straight into the first sync, so the pulse START is not detectable, while
    # its end always is.
    near = (np.abs(fclip - FREQ_SYNC) < 130).astype(np.float32)
    k = max(3, int(0.5 * ms))
    sm = np.convolve(near, np.ones(k, np.float32) / k, "same")

    def find_sync_end(expected, win_ms):
        a = max(0, int(expected - win_ms * ms))
        b = min(len(sm) - 1, int(expected + win_ms * ms))
        if b <= a:
            return None
        above = sm[a:b] > 0.5
        idx = np.where(np.diff(above.astype(np.int8)) == -1)[0]
        return a + idx[0] + 1 if idx.size else None

    # first line: the sync end lies after the VIS trailer
    first_end = find_sync_end(t0 + 20 * ms, 25)
    if first_end is None:
        first_end = t0 + sync * ms
    ch_img = [1, 2, 0]  # Martin scans green, blue, red -> image G,B,R indices
    img = np.zeros((h, w, 3), dtype=np.uint8)
    px_samp = scan / w * ms
    for line in range(h):
        exp_end = first_end + line * line_ms * ms
        got = find_sync_end(exp_end, 4)       # per-line resync
        send = got if got is not None else exp_end
        # channels start one inter-channel gap after the sync ends
        starts = [send + gap * ms,
                  send + (gap + scan + gap) * ms,
                  send + (gap + 2 * (scan + gap)) * ms]
        for ci, base in enumerate(starts):
            for px in range(w):
                a = int(base + px * px_samp)
                b = int(base + (px + 1) * px_samp)
                if a >= n:
                    a = n - 1
                if b <= a:
                    b = a + 1
                if b > n:
                    b = n
                img[line, px, ch_img[ci]] = _f2val(np.median(fclip[a:b]))
    return Image.fromarray(img, "RGB"), params["name"]
