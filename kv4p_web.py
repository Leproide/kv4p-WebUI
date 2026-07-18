#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# kv4p-web - Web client for the kv4p HT open-source ham radio (voice + scan).
# Copyright (C) 2026  https://github.com/Leproide
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Author: https://github.com/Leproide
#
# Implements the CURRENT kv4p HT serial protocol v2.2:
#   - Transport: KISS framing (FEND 0xC0, FESC 0xDB / TFEND 0xDC / TFESC 0xDD).
#   - kv4p vendor commands ride inside KISS SETHARDWARE (0x06) frames prefixed
#     with ASCII "KV4P" + protocol version 0x01 + command byte + payload.
#   - Control uses COMMAND_HOST_DESIRED_STATE (0x0D) snapshots; the firmware
#     reports COMMAND_DEVICE_STATE (0x0B) snapshots (RSSI, squelch, freqs, flags).
#   - Handshake: COMMAND_HELLO (0x06) carries version + initial device state.
#   - Live voice audio: 16 kHz 4-bit IMA ADPCM in COMMAND_*_AUDIO (0x0C).
#     (Baud 115200. The old 0xDEADBEEF/Opus protocol is gone.)
# Layouts verified byte-for-byte against a real device HELLO frame.

import argparse
import base64
from collections import deque
import json
import os
import queue
import struct
import threading
import time
import webbrowser

try:
    import sstv as sstv_mod
    SSTV_AVAILABLE = True
    SSTV_ERR = None
except Exception as _e:
    SSTV_AVAILABLE = False
    SSTV_ERR = repr(_e)
    sstv_mod = None

from flask import Flask, jsonify, request, Response

import serial
import serial.tools.list_ports

AUDIO_AVAILABLE = True
AUDIO_IMPORT_ERR = None
try:
    import numpy as np
    import sounddevice as sd
except Exception as _e:
    AUDIO_AVAILABLE = False
    AUDIO_IMPORT_ERR = repr(_e)

OPUS_AVAILABLE = True
OPUS_ERR = None
try:
    import av  # PyAV: bundles libopus, no compiler needed
except Exception as _e:
    OPUS_AVAILABLE = False
    OPUS_ERR = repr(_e)

# ---------------- KISS transport ----------------
FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD
KISS_DATA = 0x00
KISS_SETHARDWARE = 0x06
VENDOR_PREFIX = b"KV4P" + bytes([0x01])  # "KV4P" + protocolVersion 1

# ---------------- vendor command codes ----------------
# ESP32 -> host
CMD_DEBUG_INFO = 0x01
CMD_DEBUG_ERROR = 0x02
CMD_DEBUG_WARN = 0x03
CMD_DEBUG_DEBUG = 0x04
CMD_DEBUG_TRACE = 0x05
CMD_HELLO = 0x06
CMD_RX_AUDIO_OPUS = 0x07   # legacy Opus voice stream (older firmware)
CMD_WINDOW_UPDATE = 0x09
CMD_DEVICE_STATE = 0x0B
CMD_RX_AUDIO = 0x0C        # 4-bit ADPCM voice stream (current firmware)
# host -> ESP32
CMD_HOST_TX_AUDIO = 0x0C
CMD_HOST_DESIRED_STATE = 0x0D

DEBUG_CODES = {CMD_DEBUG_INFO: "INFO", CMD_DEBUG_ERROR: "ERROR", CMD_DEBUG_WARN: "WARN",
               CMD_DEBUG_DEBUG: "DEBUG", CMD_DEBUG_TRACE: "TRACE"}

# ---------------- desired-state flags (host -> ESP32) ----------------
HOST_RADIO_CONFIG_VALID = 1 << 0
HOST_PTT_REQUESTED = 1 << 1
HOST_RX_AUDIO_OPEN = 1 << 2
HOST_HIGH_POWER = 1 << 3
HOST_RSSI_ENABLED = 1 << 4
HOST_FILTER_PRE = 1 << 5
HOST_FILTER_HIGH = 1 << 6
HOST_FILTER_LOW = 1 << 7
HOST_TX_ALLOWED = 1 << 11
HOST_ENABLE_STATUS_REPORTS = 1 << 12

# ---------------- device-state flags (ESP32 -> host) ----------------
DEV_PHYS_PTT_DOWN = 1 << 8
DEV_TX_ACTIVE = 1 << 9
DEV_SQUELCHED = 1 << 10

# struct formats (little-endian, packed). Verified against hardware.
VERSION_FMT = "<HcIBffB"          # 17 bytes
VERSION_LEN = struct.calcsize(VERSION_FMT)
DEVSTATE_FMT = "<IiHBffBBBcBBB"   # 26 bytes
DEVSTATE_LEN = struct.calcsize(DEVSTATE_FMT)
DESIRED_FMT = "<IiHBffBBB"        # 22 bytes

AUDIO_SR = 16000                  # firmware live-audio sample rate (mono)
AUDIO_FRAME = 320                 # samples per TX frame (20 ms)

DEFAULT_BAUD = 115200
AUTO_BAUDS = [115200, 921600, 230400]
APP_VERSION = "2026-07-18"   # bump when behaviour changes; shown in the console
# SSTV rides the same Opus stream as voice, but tones need far more bitrate than
# speech. 40 kbit/s of audio is ~5 kB/s on the wire, well inside the 11.5 kB/s
# the 115200-baud link provides (voice stays at the low-rate voip profile).
SSTV_OPUS_BITRATE = 40000


def kiss_escape(data):
    out = bytearray()
    for b in data:
        if b == FEND:
            out += bytes([FESC, TFEND])
        elif b == FESC:
            out += bytes([FESC, TFESC])
        else:
            out.append(b)
    return bytes(out)


def kiss_unescape(data):
    out = bytearray()
    i = 0
    while i < len(data):
        b = data[i]
        if b == FESC and i + 1 < len(data):
            n = data[i + 1]
            out.append(FEND if n == TFEND else FESC if n == TFESC else n)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


def vendor_frame(cmd, payload=b""):
    """Build a KISS SETHARDWARE vendor frame carrying a kv4p command."""
    inner = bytes([KISS_SETHARDWARE]) + VENDOR_PREFIX + bytes([cmd]) + payload
    return bytes([FEND]) + kiss_escape(inner) + bytes([FEND])


# ---------------- IMA ADPCM (4-bit) ----------------
_IMA_INDEX = [-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8]
_IMA_STEP = [7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41,
             45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190,
             209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724,
             796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272,
             2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132,
             7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350,
             22385, 24623, 27086, 29794, 32767]


def make_opus_decoder(rate):
    last = None
    for name in ("libopus", "opus"):
        try:
            c = av.CodecContext.create(name, "r")
            c.sample_rate = rate
            c.format = "s16"
            c.layout = "mono"
            return c
        except Exception as e:
            last = e
    raise RuntimeError("no Opus decoder: %r" % last)


def make_opus_encoder(rate, bitrate=24000, frame_ms=40, application="voip"):
    """Build an Opus encoder.

    Voice uses the low-rate 'voip' profile. SSTV must use 'audio' at a higher
    bitrate: the voip profile is tuned for speech and mangles the pure tones
    that carry the image and the FSK-ID.
    """
    last = None
    for name in ("libopus", "opus"):
        try:
            c = av.CodecContext.create(name, "w")
            c.sample_rate = rate
            c.format = "s16"
            c.layout = "mono"
            c.bit_rate = bitrate
            opts = {"frame_duration": str(frame_ms)}
            if name == "libopus":
                opts["application"] = application
            else:
                opts["strict"] = "experimental"
            c.options = opts
            return c
        except Exception as e:
            last = e
    raise RuntimeError("no Opus encoder: %r" % last)


class ImaAdpcm:
    """Continuous IMA/DVI 4-bit ADPCM codec (low nibble first)."""

    def __init__(self):
        self.pred = 0
        self.index = 0

    def reset(self):
        self.pred = 0
        self.index = 0

    def decode(self, data):
        pred, index = self.pred, self.index
        out = np.empty(len(data) * 2, dtype=np.int16)
        k = 0
        for byte in data:
            for nib in (byte & 0x0F, (byte >> 4) & 0x0F):
                step = _IMA_STEP[index]
                diff = step >> 3
                if nib & 1:
                    diff += step >> 2
                if nib & 2:
                    diff += step >> 1
                if nib & 4:
                    diff += step
                pred = pred - diff if (nib & 8) else pred + diff
                pred = -32768 if pred < -32768 else 32767 if pred > 32767 else pred
                index += _IMA_INDEX[nib]
                index = 0 if index < 0 else 88 if index > 88 else index
                out[k] = pred
                k += 1
        self.pred, self.index = pred, index
        return out

    def encode(self, samples):
        pred, index = self.pred, self.index
        nibbles = []
        for s in samples:
            step = _IMA_STEP[index]
            diff = int(s) - pred
            nib = 0
            if diff < 0:
                nib = 8
                diff = -diff
            tmp = step
            if diff >= tmp:
                nib |= 4
                diff -= tmp
            tmp >>= 1
            if diff >= tmp:
                nib |= 2
                diff -= tmp
            tmp >>= 1
            if diff >= tmp:
                nib |= 1
            # reconstruct predictor exactly as decoder will
            d = step >> 3
            if nib & 1:
                d += step >> 2
            if nib & 2:
                d += step >> 1
            if nib & 4:
                d += step
            pred = pred - d if (nib & 8) else pred + d
            pred = -32768 if pred < -32768 else 32767 if pred > 32767 else pred
            index += _IMA_INDEX[nib]
            index = 0 if index < 0 else 88 if index > 88 else index
            nibbles.append(nib)
        self.pred, self.index = pred, index
        out = bytearray((len(nibbles) + 1) // 2)
        for i, nib in enumerate(nibbles):
            if i & 1:
                out[i >> 1] |= (nib << 4)
            else:
                out[i >> 1] = nib
        return bytes(out)


class AudioEngine:
    def __init__(self, radio):
        self.radio = radio
        self.available = AUDIO_AVAILABLE
        self.err = AUDIO_IMPORT_ERR
        self.in_dev = None
        self.out_dev = None
        self.last_err = None
        self._dec = ImaAdpcm()
        self._enc = ImaAdpcm()
        self.rx_codec = None
        self.rx_audio_cmd = None
        self._rx_on = False
        self._rx_was = False
        self._rx_q = queue.Queue(maxsize=400)
        self._rx_thread = None
        self._rx_stop = threading.Event()
        self._tx_on = False
        self._tx_thread = None
        self._tx_stop = threading.Event()
        # SSTV
        self.sstv_tx = {"running": False, "progress": 0, "mode": None}
        self._sstv_tx_thread = None
        self._sstv_tx_stop = threading.Event()
        self.sstv_rx_on = False
        self._sstv_buf = []
        self._sstv_sr = 48000
        self.sstv_last_image = None
        self.sstv_last_mode = None
        self.sstv_status = "idle"
        if self.available:
            radio.on_rx_audio = self._on_rx_packet

    def list_devices(self):
        if not self.available:
            return {"available": False, "error": self.err, "input": [], "output": []}
        try:
            ins, outs = [], []
            for i, d in enumerate(sd.query_devices()):
                if d.get("max_input_channels", 0) > 0:
                    ins.append({"index": i, "name": d["name"]})
                if d.get("max_output_channels", 0) > 0:
                    outs.append({"index": i, "name": d["name"]})
            din, dout = sd.default.device
            return {"available": True, "input": ins, "output": outs,
                    "default_in": din, "default_out": dout}
        except Exception as e:
            return {"available": False, "error": repr(e), "input": [], "output": []}

    def set_devices(self, in_dev, out_dev):
        def _dev(v):
            if v in ("", None):
                return None
            try:
                return int(v)   # device index
            except (TypeError, ValueError):
                return v         # device name substring
        self.in_dev = _dev(in_dev)
        self.out_dev = _dev(out_dev)

    # RX
    def _on_rx_packet(self, cmd, data):
        if self._rx_on and data:
            try:
                self._rx_q.put_nowait((cmd, data))
            except queue.Full:
                pass

    def start_rx(self):
        if not self.available:
            raise RuntimeError(self.err or "audio stack unavailable")
        if self._rx_on:
            return
        self._dec.reset()
        with self._rx_q.mutex:
            self._rx_q.queue.clear()
        self._rx_stop.clear()
        self._rx_on = True
        self.radio.set_rx_audio_open(True)
        self.radio._log("INFO", f"RX monitor open, flags=0x{self.radio._flags():04x} squelch={self.radio.desired['squelch']}")
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        def _watch():
            start = self.radio.telemetry.get("rx_audio_frames", 0)
            time.sleep(3.0)
            if self._rx_on and self.radio.telemetry.get("rx_audio_frames", 0) == start:
                seen = ", ".join(f"0x{c:02x}×{n}" for c, n in sorted(self.radio._cmd_counts.items()))
                self.radio._log("WARN", "no RX_AUDIO (0x0c) frames after 3s. Vendor "
                                f"cmds seen: [{seen}]. Firmware not streaming audio.")
        threading.Thread(target=_watch, daemon=True).start()

    def test_tone(self):
        if not self.available:
            raise RuntimeError(self.err or "audio stack unavailable")

        def _play():
            try:
                t = np.arange(int(AUDIO_SR * 0.5)) / AUDIO_SR
                tone = (0.3 * np.sin(2 * np.pi * 440 * t) * 32767).astype(np.int16).reshape(-1, 1)
                sd.play(tone, samplerate=AUDIO_SR, device=self.out_dev)
                sd.wait()
                self.radio._log("EVENT", "test tone played on output device")
            except Exception as e:
                self.last_err = "TONE: " + repr(e)
                self.radio._log("ERROR", "test tone failed: " + repr(e))
        threading.Thread(target=_play, daemon=True).start()

    def _rx_loop(self):
        stream = None
        codec = None
        opusdec = None
        try:
            while not self._rx_stop.is_set():
                try:
                    cmd, pkt = self._rx_q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if codec is None:
                    # pick codec + output rate from the first audio frame
                    if cmd == CMD_RX_AUDIO_OPUS:
                        if not OPUS_AVAILABLE:
                            self.radio._log("ERROR", "Opus audio needs PyAV: pip install av")
                            return
                        codec, sr = "opus", 48000
                        opusdec = make_opus_decoder(sr)
                    else:
                        codec, sr = "adpcm", AUDIO_SR
                    self.rx_codec, self.rx_audio_cmd = codec, cmd
                    stream = sd.OutputStream(samplerate=sr, channels=1, dtype="int16",
                                             device=self.out_dev)
                    stream.start()
                    self.radio._log("EVENT", f"RX audio: {codec} @ {sr} Hz, output opened")
                    self._sstv_sr = sr
                try:
                    if codec == "opus":
                        for fr in opusdec.decode(av.Packet(pkt)):
                            pcm = fr.to_ndarray().reshape(-1)
                            stream.write(pcm.reshape(-1, 1))
                            if self.sstv_rx_on:
                                self._sstv_buf.append(pcm.copy())
                    else:
                        pcm = self._dec.decode(pkt)
                        stream.write(pcm.reshape(-1, 1))
                        if self.sstv_rx_on:
                            self._sstv_buf.append(pcm.copy())
                except Exception:
                    continue
        except Exception as e:
            self.last_err = "RX: " + repr(e)
            self.radio._log("ERROR", "RX output failed: " + repr(e))
        finally:
            if stream is not None:
                try:
                    stream.stop(); stream.close()
                except Exception:
                    pass

    def stop_rx(self):
        was = self._rx_on
        self._rx_on = False
        self._rx_stop.set()
        t = self._rx_thread
        if t and t.is_alive():
            t.join(timeout=1.0)
        self._rx_thread = None
        if was:
            self.radio.set_rx_audio_open(False)

    # TX
    def start_tx(self):
        if not self.available:
            raise RuntimeError(self.err or "audio stack unavailable")
        if self._tx_on:
            return
        if not self.radio.tx_allowed:
            raise RuntimeError("TX not allowed: enable 'TX allowed' first")
        self._rx_was = self._rx_on
        if self._rx_on:
            self.stop_rx()
        self._enc.reset()
        self._tx_stop.clear()
        self._tx_on = True
        self.radio.set_ptt(True)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()

    def _tx_loop(self):
        stream = None
        # transmit with the same codec/command the firmware uses for audio
        use_opus = (self.rx_audio_cmd == CMD_RX_AUDIO_OPUS)
        tx_cmd = CMD_RX_AUDIO_OPUS if use_opus else CMD_HOST_TX_AUDIO
        sr = 48000 if use_opus else AUDIO_SR
        frame = int(sr * 0.04) if use_opus else AUDIO_FRAME
        try:
            if use_opus:
                if not OPUS_AVAILABLE:
                    raise RuntimeError("Opus TX needs PyAV: pip install av")
                enc = make_opus_encoder(sr)
            stream = sd.InputStream(samplerate=sr, channels=1, dtype="int16",
                                    device=self.in_dev, blocksize=frame)
            stream.start()
            self.radio._log("INFO", f"TX audio: {'opus' if use_opus else 'adpcm'} "
                                    f"@ {sr} Hz on cmd 0x{tx_cmd:02x}")
            while not self._tx_stop.is_set():
                data, _ = stream.read(frame)
                if use_opus:
                    af = av.AudioFrame.from_ndarray(data.reshape(1, -1), format="s16", layout="mono")
                    af.sample_rate = sr
                    for p in enc.encode(af):
                        self.radio.send_vendor(tx_cmd, bytes(p))
                else:
                    self.radio.send_vendor(tx_cmd, self._enc.encode(data.reshape(-1)))
        except Exception as e:
            self.last_err = "TX: " + repr(e)
            self.radio._log("ERROR", "TX failed: " + repr(e))
        finally:
            if stream is not None:
                try:
                    stream.stop(); stream.close()
                except Exception:
                    pass

    def stop_tx(self):
        was = self._tx_on
        self._tx_stop.set()
        t = self._tx_thread
        if t and t.is_alive():
            t.join(timeout=1.5)
        self._tx_thread = None
        if was:
            self.radio.set_ptt(False)
        self._tx_on = False
        if self._rx_was:
            try:
                self.start_rx()
            except Exception:
                pass

    def shutdown(self):
        try:
            self.stop_sstv_tx()
        except Exception:
            pass
        try:
            self.stop_tx()
        except Exception:
            pass
        try:
            self.stop_rx()
        except Exception:
            pass

    # ---- SSTV transmit ----
    def start_sstv_tx(self, image_bytes, mode, fskid=None, overlay=None):
        if not SSTV_AVAILABLE:
            raise RuntimeError("SSTV unavailable: " + str(SSTV_ERR))
        if not self.radio.tx_allowed:
            raise RuntimeError("TX not allowed: enable 'TX allowed' first")
        if self.sstv_tx["running"] or self._tx_on:
            raise RuntimeError("busy")
        use_opus = (self.rx_audio_cmd == CMD_RX_AUDIO_OPUS)
        sr = 48000 if use_opus else AUDIO_SR
        samples = sstv_mod.encode_image(image_bytes, mode, sr, fskid=fskid,
                                        overlay=overlay)
        self.radio._log("EVENT", "SSTV TX %s, %.1f s, FSK-ID: %s, overlay: %s"
                        % (mode, len(samples) / sr, fskid if fskid else "(none)",
                           overlay if overlay else "(none)"))
        self._sstv_tx_stop.clear()
        self.sstv_tx = {"running": True, "progress": 0, "mode": mode}
        # pause RX monitor during TX (half duplex)
        self._sstv_rx_was = self._rx_on
        if self._rx_on:
            self.stop_rx()
        self._sstv_tx_thread = threading.Thread(
            target=self._sstv_tx_loop, args=(samples, sr, use_opus), daemon=True)
        self._sstv_tx_thread.start()

    def _sstv_tx_loop(self, samples, sr, use_opus):
        tx_cmd = CMD_RX_AUDIO_OPUS if use_opus else CMD_HOST_TX_AUDIO
        frame = int(sr * 0.04) if use_opus else AUDIO_FRAME
        try:
            self.radio.set_ptt(True)
            enc = (make_opus_encoder(sr, bitrate=SSTV_OPUS_BITRATE,
                                     application="audio") if use_opus else None)
            total = len(samples)
            i = 0
            t_next = time.time()
            while i < total and not self._sstv_tx_stop.is_set():
                chunk = samples[i:i + frame]
                if len(chunk) < frame:
                    chunk = np.concatenate([chunk, np.zeros(frame - len(chunk), np.int16)])
                if use_opus:
                    af = av.AudioFrame.from_ndarray(chunk.reshape(1, -1), format="s16", layout="mono")
                    af.sample_rate = sr
                    for p in enc.encode(af):
                        self.radio.send_vendor(tx_cmd, bytes(p))
                else:
                    self.radio.send_vendor(tx_cmd, self._enc.encode(chunk))
                i += frame
                self.sstv_tx["progress"] = int(i / total * 100)
                # pace to real time so we don't overrun the radio buffer
                t_next += frame / sr
                dt = t_next - time.time()
                if dt > 0:
                    time.sleep(dt)
            # The radio buffers audio ahead of us, so the last second of the
            # transmission (which is where the FSK-ID callsign lives) would be
            # cut off if we dropped PTT here. Push trailing silence, then hold
            # the carrier while the radio drains its buffer.
            if not self._sstv_tx_stop.is_set():
                silence = np.zeros(frame, dtype=np.int16)
                for _ in range(max(1, int(0.6 * sr / frame))):
                    if use_opus:
                        af = av.AudioFrame.from_ndarray(silence.reshape(1, -1),
                                                        format="s16", layout="mono")
                        af.sample_rate = sr
                        for p in enc.encode(af):
                            self.radio.send_vendor(tx_cmd, bytes(p))
                    else:
                        self.radio.send_vendor(tx_cmd, self._enc.encode(silence))
                    t_next += frame / sr
                    dt = t_next - time.time()
                    if dt > 0:
                        time.sleep(dt)
                time.sleep(1.5)   # let the radio play out what is still buffered
        except Exception as e:
            self.last_err = "SSTV TX: " + repr(e)
            self.radio._log("ERROR", "SSTV TX failed: " + repr(e))
        finally:
            self.radio.set_ptt(False)
            self.sstv_tx = {"running": False, "progress": 100, "mode": self.sstv_tx.get("mode")}
            if getattr(self, "_sstv_rx_was", False):
                try:
                    self.start_rx()
                except Exception:
                    pass

    def stop_sstv_tx(self):
        self._sstv_tx_stop.set()
        t = self._sstv_tx_thread
        if t and t.is_alive():
            t.join(timeout=2.0)
        self._sstv_tx_thread = None

    # ---- SSTV receive ----
    def set_sstv_rx(self, on):
        on = bool(on)
        if on and not SSTV_AVAILABLE:
            raise RuntimeError("SSTV unavailable: " + str(SSTV_ERR))
        self.sstv_rx_on = on
        if on:
            self._sstv_buf = []
            self.sstv_status = "listening…"
        else:
            self.sstv_status = "idle"

    def sstv_decode_now(self):
        if not SSTV_AVAILABLE:
            raise RuntimeError("SSTV unavailable")
        if not self._sstv_buf:
            raise RuntimeError("no audio captured yet")
        self.sstv_status = "decoding…"

        def _dec():
            try:
                samples = np.concatenate(self._sstv_buf).astype(np.int16)
                img, mode = sstv_mod.decode(samples, self._sstv_sr)
                if img is None:
                    self.sstv_status = "decode failed: " + str(mode)
                    return
                buf = __import__("io").BytesIO()
                img.save(buf, format="PNG")
                self.sstv_last_image = base64.b64encode(buf.getvalue()).decode()
                self.sstv_last_mode = mode
                self.sstv_status = "received: " + mode
                self.radio._log("EVENT", "SSTV image decoded: " + mode)
            except Exception as e:
                self.sstv_status = "decode error: " + repr(e)
        threading.Thread(target=_dec, daemon=True).start()

    def sstv_selftest(self, image_bytes, mode):
        if not SSTV_AVAILABLE:
            raise RuntimeError("SSTV unavailable")
        self.sstv_status = "self-test: encoding…"

        def _run():
            try:
                sr = 48000
                samples = sstv_mod.encode_image(image_bytes, mode, sr)
                self.sstv_status = "self-test: decoding…"
                img, dmode = sstv_mod.decode(samples, sr)
                if img is None:
                    self.sstv_status = "self-test decode failed: " + str(dmode)
                    return
                buf = __import__("io").BytesIO()
                img.save(buf, format="PNG")
                self.sstv_last_image = base64.b64encode(buf.getvalue()).decode()
                self.sstv_last_mode = dmode
                self.sstv_status = "self-test OK: " + dmode + " (no radio)"
            except Exception as e:
                self.sstv_status = "self-test error: " + repr(e)
        threading.Thread(target=_run, daemon=True).start()

    def sstv_decode_file(self, audio_bytes):
        if not SSTV_AVAILABLE:
            raise RuntimeError("SSTV unavailable")
        self.sstv_status = "decoding file…"

        def _dec():
            try:
                import io as _io
                from scipy.io import wavfile
                sr, data = wavfile.read(_io.BytesIO(audio_bytes))
                if data.ndim > 1:
                    data = data[:, 0]
                if data.dtype != np.int16:
                    m = np.max(np.abs(data)) or 1
                    data = (data.astype(np.float64) / m * 32767).astype(np.int16)
                img, mode = sstv_mod.decode(data, sr)
                if img is None:
                    self.sstv_status = "file decode failed: " + str(mode)
                    return
                buf = _io.BytesIO()
                img.save(buf, format="PNG")
                self.sstv_last_image = base64.b64encode(buf.getvalue()).decode()
                self.sstv_last_mode = mode
                self.sstv_status = "decoded file: " + mode
            except Exception as e:
                self.sstv_status = "file decode error: " + repr(e)
        threading.Thread(target=_dec, daemon=True).start()

    def status(self):
        return {"available": self.available, "error": self.err, "last_err": self.last_err,
                "rx_on": self._rx_on, "tx_on": self._tx_on,
                "in_dev": self.in_dev, "out_dev": self.out_dev,
                "sstv": {"available": SSTV_AVAILABLE, "tx": self.sstv_tx,
                         "rx_on": self.sstv_rx_on, "status": self.sstv_status,
                         "have_image": self.sstv_last_image is not None,
                         "mode": self.sstv_last_mode,
                         "buf_sec": round(sum(len(x) for x in self._sstv_buf) / max(1, self._sstv_sr), 1)}}


class Radio:
    def __init__(self):
        self._ser = None
        self._write_lock = threading.Lock()
        self._reader = None
        self._reader_stop = threading.Event()
        self._scan_thread = None
        self._scan_stop = threading.Event()
        self.on_rx_audio = None
        self.debug_log = deque(maxlen=500)
        self.hexdump = False
        self._cmd_counts = {}
        self.lock = threading.Lock()
        # KISS reader state
        self._in_frame = False
        self._kbuf = bytearray()
        self._rawtext = bytearray()
        # desired-state (host controlled)
        self._seq = 0
        self.tx_allowed = False
        self.high_power = False
        self.pref_tx_allowed = False   # remembered across connects (persisted)
        self.pref_high_power = False
        self.rx_audio_open = False
        self.ptt = False
        self.filt_pre = False
        self.filt_high = True
        self.filt_low = True
        self.ptt_key = "Space"
        self.sstv_callsign = ""
        self.sstv_overlay = False   # burn callsign into the picture
        self.saved_config = None   # persisted full config, applied on connect
        self.desired = {"memoryId": -1, "bw": 0, "freq_tx": 145.5, "freq_rx": 145.5,
                        "ctcss_tx": 0, "squelch": 0, "ctcss_rx": 0}
        self._reset_state()

    def _reset_state(self):
        self.port = None
        self.baud = None
        self.connected = False
        self.telemetry = {
            "rssi": None, "rssi_ts": 0.0, "phys_ptt": False, "tx_active": False,
            "squelched": True, "hello": False, "version": None, "window_size": None,
            "rf_module_type": None, "min_freq": None, "max_freq": None, "features": None,
            "mode": None, "last_error": None, "applied_seq": None,
            "cur_freq_tx": None, "cur_freq_rx": None, "rx_audio_frames": 0,
            "last_rx_ts": 0.0, "last_debug": None, "tx": False,
        }
        self.scan = {"running": False, "index": 0, "total": 0, "current": None, "results": []}

    # ---- low level ----
    def _write(self, data):
        with self._write_lock:
            if self._ser and self._ser.is_open:
                self._ser.write(data)
                self._ser.flush()

    def send_vendor(self, cmd, payload=b""):
        self._write(vendor_frame(cmd, payload))

    def _log(self, kind, text):
        self.debug_log.append({"t": time.strftime("%H:%M:%S"), "kind": kind, "text": str(text)})

    # ---- desired state ----
    def _flags(self):
        f = HOST_RADIO_CONFIG_VALID | HOST_RSSI_ENABLED | HOST_ENABLE_STATUS_REPORTS
        if self.filt_pre:
            f |= HOST_FILTER_PRE
        if self.filt_high:
            f |= HOST_FILTER_HIGH
        if self.filt_low:
            f |= HOST_FILTER_LOW
        if self.tx_allowed:
            f |= HOST_TX_ALLOWED
        if self.high_power:
            f |= HOST_HIGH_POWER
        if self.rx_audio_open:
            f |= HOST_RX_AUDIO_OPEN
        if self.ptt:
            f |= HOST_PTT_REQUESTED
        return f

    def send_desired(self):
        if not self.connected:
            return
        self._seq += 1
        d = self.desired
        payload = struct.pack(DESIRED_FMT, self._seq & 0xFFFFFFFF, d["memoryId"],
                              self._flags(), d["bw"] & 0xFF, float(d["freq_tx"]),
                              float(d["freq_rx"]), d["ctcss_tx"] & 0xFF,
                              d["squelch"] & 0xFF, d["ctcss_rx"] & 0xFF)
        self.send_vendor(CMD_HOST_DESIRED_STATE, payload)

    def tune(self, freq_rx, freq_tx=None, bw=0, squelch=1, ctcss_tx=0, ctcss_rx=0):
        self.desired.update({"freq_rx": float(freq_rx),
                             "freq_tx": float(freq_tx if freq_tx is not None else freq_rx),
                             "bw": int(bw), "squelch": int(squelch),
                             "ctcss_tx": int(ctcss_tx), "ctcss_rx": int(ctcss_rx)})
        self.persist()
        self.send_desired()

    def set_ptt(self, down):
        self.ptt = bool(down)
        with self.lock:
            self.telemetry["tx"] = self.ptt
        self.send_desired()

    def set_rx_audio_open(self, on):
        self.rx_audio_open = bool(on)
        self.send_desired()

    def persist(self):
        try:
            save_settings({
                "tx_allowed": self.pref_tx_allowed, "high_power": self.pref_high_power,
                "filt_pre": self.filt_pre, "filt_high": self.filt_high, "filt_low": self.filt_low,
                "bw": self.desired["bw"], "squelch": self.desired["squelch"],
                "freq_rx": self.desired["freq_rx"], "freq_tx": self.desired["freq_tx"],
                "ctcss_tx": self.desired["ctcss_tx"], "ctcss_rx": self.desired["ctcss_rx"],
                "ptt_key": self.ptt_key, "sstv_callsign": self.sstv_callsign,
                "sstv_overlay": self.sstv_overlay,
            })
        except Exception:
            pass

    def set_flags(self, tx_allowed=None, high_power=None, pre=None, high=None, low=None):
        if tx_allowed is not None:
            self.tx_allowed = bool(tx_allowed)
            self.pref_tx_allowed = self.tx_allowed
        if high_power is not None:
            self.high_power = bool(high_power)
            self.pref_high_power = self.high_power
        if pre is not None:
            self.filt_pre = bool(pre)
        if high is not None:
            self.filt_high = bool(high)
        if low is not None:
            self.filt_low = bool(low)
        self.persist()
        self.send_desired()

    def stop(self):
        self.ptt = False
        self.rx_audio_open = False
        with self.lock:
            self.telemetry["tx"] = False
        self.send_desired()

    # ---- connection ----
    def list_ports(self):
        return [{"device": p.device, "description": p.description or "", "hwid": p.hwid or ""}
                for p in serial.tools.list_ports.comports()]

    def _open_serial(self, port, baud):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 0.1
        ser.write_timeout = 2.0
        ser.dtr = False
        ser.rts = False
        ser.open()
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        return ser

    def _reset_esp32(self, ser):
        try:
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
            time.sleep(0.25)
        except Exception as e:
            self._log("WARN", f"reset pulse failed: {e!r}")

    def _probe(self, ser, timeout=3.0):
        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            data = ser.read(1024)
            if data:
                buf.extend(data)
                if b"KV4P" in buf or b"kv4p" in buf or FEND in buf:
                    return True
        return False

    def connect(self, port, baud, reset=True):
        self.disconnect()
        self.debug_log.clear()
        self._in_frame = False
        self._kbuf.clear()
        self._rawtext.clear()
        self._seq = 0
        self._log("INFO", f"kv4p-web build {APP_VERSION}")
        self._log("INFO", "audio stack: " + ("available" if AUDIO_AVAILABLE
                  else "UNAVAILABLE (" + str(AUDIO_IMPORT_ERR) + ")"))
        self._log("INFO", f"connecting {port} baud={baud} reset={reset}")
        chosen = None
        if str(baud).lower() == "auto":
            for b in AUTO_BAUDS:
                try:
                    ser = self._open_serial(port, b)
                except Exception as e:
                    raise RuntimeError(f"cannot open {port}: {e}")
                if reset:
                    self._reset_esp32(ser)
                if self._probe(ser):
                    self._ser = ser
                    chosen = b
                    self._log("INFO", f"auto-detect locked at {b} baud")
                    break
                self._log("WARN", f"no kv4p data at {b} baud")
                try:
                    ser.close()
                except Exception:
                    pass
            if self._ser is None:
                raise RuntimeError("auto-detect failed. Try fixed 115200.")
        else:
            chosen = int(baud)
            self._ser = self._open_serial(port, chosen)
            if reset:
                self._reset_esp32(self._ser)
        with self.lock:
            self._reset_state()
            self.port = port
            self.baud = chosen
            self.connected = True
        self._reader_stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        return chosen

    def disconnect(self):
        self.stop_scan()
        try:
            if self._ser and self._ser.is_open and self.connected:
                self.ptt = False
                self.rx_audio_open = False
                self.send_desired()
        except Exception:
            pass
        self._reader_stop.set()
        r = self._reader
        if r and r.is_alive():
            r.join(timeout=1.0)
        self._reader = None
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        with self.lock:
            self.connected = False

    # ---- reader / KISS parser ----
    def _read_loop(self):
        while not self._reader_stop.is_set():
            try:
                data = self._ser.read(4096)
            except Exception:
                break
            if not data:
                continue
            if self.hexdump:
                self._log("HEX", f"{len(data)}B " + data[:64].hex(" "))
            for b in data:
                if b == FEND:
                    if self._in_frame:
                        if self._kbuf:
                            self._dispatch(bytes(self._kbuf))
                            self._kbuf.clear()
                    else:
                        self._flush_rawtext()
                        self._in_frame = True
                else:
                    if self._in_frame:
                        self._kbuf.append(b)
                    else:
                        self._rawtext.append(b)
                        if b in (10, 13) or len(self._rawtext) > 300:
                            self._flush_rawtext()

    def _flush_rawtext(self):
        line = bytes(self._rawtext)
        self._rawtext.clear()
        line = line.rstrip(b"\r\n")
        if not line:
            return
        printable = sum(1 for c in line if 32 <= c < 127 or c == 9)
        if len(line) >= 2 and printable / len(line) > 0.6:
            self._log("RAW", line.decode("latin1", "replace"))

    def _dispatch(self, raw):
        frame = kiss_unescape(raw)
        if not frame:
            return
        ktype = frame[0]
        payload = frame[1:]
        if ktype == KISS_SETHARDWARE:
            if len(payload) >= 6 and payload[:4] == b"KV4P" and payload[4] == 0x01:
                self._handle_vendor(payload[5], payload[6:])
        # KISS_DATA (AX.25) frames are ignored for voice

    def _handle_vendor(self, cmd, data):
        now = time.time()
        cb = None
        cbarg = None
        with self.lock:
            self._cmd_counts[cmd] = self._cmd_counts.get(cmd, 0) + 1
            if cmd == CMD_RX_AUDIO or cmd == CMD_RX_AUDIO_OPUS:
                self.telemetry["rx_audio_frames"] += 1
                self.telemetry["last_rx_ts"] = now
                if self.telemetry["rx_audio_frames"] == 1:
                    codec = "Opus" if cmd == CMD_RX_AUDIO_OPUS else "ADPCM"
                    self._log("EVENT", f"first RX audio frame: cmd=0x{cmd:02x} "
                                       f"({codec}), {len(data)} bytes")
                cb = self.on_rx_audio
                cbarg = (cmd, data)
            elif cmd == CMD_HELLO:
                self._parse_hello(data)
            elif cmd == CMD_DEVICE_STATE:
                self._parse_devstate(data)
            elif cmd in DEBUG_CODES:
                txt = data.decode("utf-8", "replace")
                self.telemetry["last_debug"] = f"[{DEBUG_CODES[cmd]}] {txt}"
                self._log(DEBUG_CODES[cmd], txt)
            elif cmd == CMD_WINDOW_UPDATE:
                pass
        if cb is not None and cbarg is not None:
            try:
                cb(*cbarg)
            except Exception:
                pass

    def _parse_hello(self, data):
        if len(data) < VERSION_LEN:
            return
        ver, status, window, rftype, fmin, fmax, feat = struct.unpack(VERSION_FMT, data[:VERSION_LEN])
        try:
            status_s = status.decode("latin1")
        except Exception:
            status_s = "?"
        self.telemetry.update({
            "hello": True, "version": ver, "window_size": window,
            "rf_module_type": rftype, "min_freq": round(fmin, 4), "max_freq": round(fmax, 4),
            "features": feat, "module_status": status_s})
        self._log("EVENT", f"HELLO ver={ver} status='{status_s}' rfType={rftype} "
                           f"range={fmin:.3f}-{fmax:.3f} MHz feat=0x{feat:02x} win={window}")
        if len(data) >= VERSION_LEN + DEVSTATE_LEN:
            self._parse_devstate(data[VERSION_LEN:VERSION_LEN + DEVSTATE_LEN], seed=True)
        # kick off status reports / rssi with our desired snapshot
        threading.Thread(target=self.send_desired, daemon=True).start()

    def _parse_devstate(self, data, seed=False):
        if len(data) < DEVSTATE_LEN:
            return
        (aseq, mem, flags, bw, ftx, frx, ct, sq, cr, rms, mode, lerr, rssi) = \
            struct.unpack(DEVSTATE_FMT, data[:DEVSTATE_LEN])
        # Stay ahead of the device's applied sequence, otherwise the firmware
        # ignores our desired-state (this is why RSSI/squelch/tune didn't apply).
        if aseq >= self._seq:
            self._seq = aseq
        self.telemetry.update({
            "applied_seq": aseq, "cur_freq_tx": round(ftx, 4), "cur_freq_rx": round(frx, 4),
            "rssi": rssi, "rssi_ts": time.time(), "mode": mode, "last_error": lerr,
            "phys_ptt": bool(flags & DEV_PHYS_PTT_DOWN),
            "tx_active": bool(flags & DEV_TX_ACTIVE),
            "squelched": bool(flags & DEV_SQUELCHED),
            "dev_flags": flags,
            "dev_high_power": bool(flags & HOST_HIGH_POWER),
            "dev_pre": bool(flags & HOST_FILTER_PRE),
            "dev_tx_allowed": bool(flags & HOST_TX_ALLOWED),
            "dev_squelch": sq})
        if seed:
            self.desired.update({"freq_tx": round(ftx, 4), "freq_rx": round(frx, 4),
                                "bw": bw, "squelch": sq, "ctcss_tx": ct, "ctcss_rx": cr,
                                "memoryId": mem})
            # adopt the radio's already-applied preferences so we don't clobber
            # a working state (TX-allowed, high-power, filters set via the app)
            # adopt the radio's applied prefs, but a remembered client
            # preference for TX-allowed / high-power wins (re-enabled after HELLO)
            self.tx_allowed = bool(flags & HOST_TX_ALLOWED) or self.pref_tx_allowed
            self.high_power = bool(flags & HOST_HIGH_POWER) or self.pref_high_power
            self.filt_pre = bool(flags & HOST_FILTER_PRE)
            self.filt_high = bool(flags & HOST_FILTER_HIGH)
            self.filt_low = bool(flags & HOST_FILTER_LOW)
            # a saved client config wins so all settings persist across sessions
            sc = self.saved_config
            if sc:
                for kk in ("bw", "squelch", "ctcss_tx", "ctcss_rx"):
                    if kk in sc:
                        self.desired[kk] = int(sc[kk])
                for kk in ("freq_rx", "freq_tx"):
                    if kk in sc:
                        self.desired[kk] = float(sc[kk])
                if "filt_pre" in sc:
                    self.filt_pre = bool(sc["filt_pre"])
                if "filt_high" in sc:
                    self.filt_high = bool(sc["filt_high"])
                if "filt_low" in sc:
                    self.filt_low = bool(sc["filt_low"])

    # ---- scanning (host-driven; firmware reports squelch/rssi) ----
    def start_scan(self, start, end, step, dwell_ms=250, method="rssi",
                   rssi_threshold=64, squelch=1, bw=0, stop_on_active=False):
        if not self.connected:
            raise RuntimeError("not connected")
        self.stop_scan()
        if step <= 0:
            raise ValueError("step must be > 0")
        if end < start:
            start, end = end, start
        # clamp to the radio module's usable range (from HELLO)
        mn = self.telemetry.get("min_freq")
        mx = self.telemetry.get("max_freq")
        if mn is not None and mx is not None:
            cs, ce = max(mn, min(start, mx)), max(mn, min(end, mx))
            if (cs, ce) != (start, end):
                self._log("INFO", f"scan range clamped to module {mn}-{mx} MHz "
                                  f"(was {start}-{end})")
            start, end = cs, ce
        n = int(round((end - start) / step)) + 1
        freqs = [round(start + i * step, 5) for i in range(n)]
        self._scan_stop.clear()
        with self.lock:
            self.scan = {"running": True, "index": 0, "total": len(freqs),
                         "current": None, "results": []}
        self._scan_thread = threading.Thread(
            target=self._scan_loop,
            args=(freqs, dwell_ms, method, rssi_threshold, squelch, bw, stop_on_active),
            daemon=True)
        self._scan_thread.start()

    def _scan_loop(self, freqs, dwell_ms, method, rssi_threshold, squelch, bw, stop_on_active):
        dwell = max(0.05, dwell_ms / 1000.0)
        # For squelch-based detection we need the squelch to GATE (close on no
        # signal); squelch 0 = monitor = always open, which flags everything.
        scan_sq = squelch
        if method == "squelch" and scan_sq < 1:
            scan_sq = 4
        for i, f in enumerate(freqs):
            if self._scan_stop.is_set():
                break
            with self.lock:
                self.scan["index"] = i
                self.scan["current"] = f
            self.tune(f, f, bw=bw, squelch=scan_sq)
            t0 = time.time()
            peak = 0
            open_seen = False
            while time.time() < t0 + dwell:
                if self._scan_stop.is_set():
                    break
                with self.lock:
                    rssi = self.telemetry["rssi"] or 0
                    squelched = self.telemetry["squelched"]
                if rssi > peak:
                    peak = rssi
                if not squelched:
                    open_seen = True
                time.sleep(0.03)
            active = open_seen if method == "squelch" else (peak >= rssi_threshold)
            with self.lock:
                self.scan["results"].append({"freq": f, "rssi_peak": peak,
                                             "rx_frames": int(open_seen), "active": bool(active)})
            if active and stop_on_active:
                break
        with self.lock:
            self.scan["running"] = False
            self.scan["current"] = None

    def stop_scan(self):
        self._scan_stop.set()
        t = self._scan_thread
        if t and t.is_alive():
            t.join(timeout=2.0)
        self._scan_thread = None
        with self.lock:
            self.scan["running"] = False

    def status(self):
        with self.lock:
            tel = dict(self.telemetry)
            scan = {"running": self.scan["running"], "index": self.scan["index"],
                    "total": self.scan["total"], "current": self.scan["current"],
                    "results": list(self.scan["results"])}
            connected, port, baud = self.connected, self.port, self.baud
            log = list(self.debug_log)
        tel["rssi_fresh"] = (tel["rssi"] is not None and (time.time() - tel["rssi_ts"]) < 2.0)
        return {"connected": connected, "port": port, "baud": baud, "telemetry": tel,
                "scan": scan, "log": log,
                "ctrl": {"tx_allowed": self.tx_allowed, "high_power": self.high_power,
                         "filt_pre": self.filt_pre, "filt_high": self.filt_high,
                         "filt_low": self.filt_low, "squelch": self.desired["squelch"],
                         "bw": self.desired["bw"], "ctcss_tx": self.desired["ctcss_tx"],
                         "ctcss_rx": self.desired["ctcss_rx"], "ptt_key": self.ptt_key,
                         "sstv_callsign": self.sstv_callsign,
                         "sstv_overlay": self.sstv_overlay}}


radio = Radio()
audio = AudioEngine(radio)
app = Flask(__name__)

# ---- frequency memories, persisted to a JSON file ----
MEM_FILE = os.path.join(os.path.expanduser("~"), "kv4p_web_memories.json")
_mem_lock = threading.Lock()


def load_memories():
    try:
        with open(MEM_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return []


def save_memories(mems):
    try:
        with open(MEM_FILE, "w", encoding="utf-8") as f:
            json.dump(mems, f, indent=2)
        return True
    except Exception as e:
        radio._log("ERROR", f"could not save memories: {e!r}")
        return False


memories = load_memories()

# ---- small persisted client settings (remembered TX-allowed / high-power) ----
SETTINGS_FILE = os.path.join(os.path.expanduser("~"), "kv4p_web_settings.json")


def load_settings():
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_settings(s):
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(s, f, indent=2)
    except Exception:
        pass


_settings = load_settings()
radio.pref_tx_allowed = bool(_settings.get("tx_allowed", False))
radio.pref_high_power = bool(_settings.get("high_power", False))
if "filt_pre" in _settings:
    radio.filt_pre = bool(_settings["filt_pre"])
if "filt_high" in _settings:
    radio.filt_high = bool(_settings["filt_high"])
if "filt_low" in _settings:
    radio.filt_low = bool(_settings["filt_low"])
radio.ptt_key = _settings.get("ptt_key", "Space")
radio.sstv_callsign = _settings.get("sstv_callsign", "")
radio.sstv_overlay = bool(_settings.get("sstv_overlay", False))
for _k in ("bw", "squelch", "ctcss_tx", "ctcss_rx"):
    if _k in _settings:
        radio.desired[_k] = int(_settings[_k])
for _k in ("freq_rx", "freq_tx"):
    if _k in _settings:
        radio.desired[_k] = float(_settings[_k])
radio.saved_config = dict(_settings) if _settings else None


@app.route("/api/settings/pttkey", methods=["POST"])
def api_pttkey():
    d = request.get_json(force=True)
    radio.ptt_key = str(d.get("key", "Space"))
    radio.persist()
    return jsonify({"ok": True})


@app.route("/api/memories", methods=["GET"])
def api_memories_get():
    with _mem_lock:
        return jsonify(memories)


@app.route("/api/memories", methods=["POST"])
def api_memories_add():
    d = request.get_json(force=True)
    with _mem_lock:
        mem = {
            "id": (max([m.get("id", 0) for m in memories], default=0) + 1),
            "name": str(d.get("name", "")).strip() or "unnamed",
            "freq_rx": float(d.get("freq_rx", radio.desired["freq_rx"])),
            "freq_tx": float(d.get("freq_tx", d.get("freq_rx", radio.desired["freq_tx"]))),
            "bw": int(d.get("bw", radio.desired["bw"])),
            "squelch": int(d.get("squelch", radio.desired["squelch"])),
            "ctcss_tx": int(d.get("ctcss_tx", radio.desired["ctcss_tx"])),
            "ctcss_rx": int(d.get("ctcss_rx", radio.desired["ctcss_rx"])),
        }
        memories.append(mem)
        save_memories(memories)
    return jsonify({"ok": True, "memory": mem})


@app.route("/api/memories/delete", methods=["POST"])
def api_memories_delete():
    d = request.get_json(force=True)
    mid = int(d.get("id", -1))
    with _mem_lock:
        before = len(memories)
        memories[:] = [m for m in memories if m.get("id") != mid]
        save_memories(memories)
    return jsonify({"ok": True, "removed": before - len(memories)})


@app.route("/api/memories/rename", methods=["POST"])
def api_memories_rename():
    d = request.get_json(force=True)
    mid = int(d.get("id", -1))
    with _mem_lock:
        for m in memories:
            if m.get("id") == mid:
                m["name"] = str(d.get("name", "")).strip() or m["name"]
        save_memories(memories)
    return jsonify({"ok": True})


@app.route("/api/ports")
def api_ports():
    return jsonify(radio.list_ports())


@app.route("/api/connect", methods=["POST"])
def api_connect():
    d = request.get_json(force=True)
    try:
        b = radio.connect(d["port"], d.get("baud", DEFAULT_BAUD), bool(d.get("reset", True)))

        def _auto_monitor():
            time.sleep(1.3)  # let HELLO arrive and the sequence sync
            if radio.connected and audio.available:
                try:
                    audio.start_rx()
                    radio._log("INFO", "auto-started RX monitor")
                except Exception as e:
                    radio._log("WARN", "auto RX monitor failed: " + repr(e))
        threading.Thread(target=_auto_monitor, daemon=True).start()
        return jsonify({"ok": True, "baud": b})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    audio.shutdown()
    radio.disconnect()
    return jsonify({"ok": True})


@app.route("/api/status")
def api_status():
    s = radio.status()
    s["audio"] = audio.status()
    return jsonify(s)


@app.route("/api/tune", methods=["POST"])
def api_tune():
    d = request.get_json(force=True)
    if not radio.connected:
        return jsonify({"ok": False, "error": "not connected"}), 400
    try:
        radio.tune(float(d["freq_rx"]),
                   float(d["freq_tx"]) if d.get("freq_tx") not in (None, "") else None,
                   int(d.get("bw", 0)), int(d.get("squelch", 1)),
                   int(d.get("ctcss_tx", 0)), int(d.get("ctcss_rx", 0)))
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/flags", methods=["POST"])
def api_flags():
    d = request.get_json(force=True)
    radio.set_flags(d.get("tx_allowed"), d.get("high_power"),
                    d.get("pre"), d.get("high"), d.get("low"))
    return jsonify({"ok": True})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    audio.stop_tx()
    radio.stop()
    return jsonify({"ok": True})


@app.route("/api/audio/devices")
def api_audio_devices():
    return jsonify(audio.list_devices())


@app.route("/api/audio/config", methods=["POST"])
def api_audio_config():
    d = request.get_json(force=True)
    audio.set_devices(d.get("input"), d.get("output"))
    return jsonify({"ok": True})


@app.route("/api/audio/test", methods=["POST"])
def api_audio_test():
    try:
        audio.test_tone()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/audio/rx", methods=["POST"])
def api_audio_rx():
    d = request.get_json(force=True)
    try:
        audio.start_rx() if bool(d.get("on")) else audio.stop_rx()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/voice/ptt", methods=["POST"])
def api_voice_ptt():
    d = request.get_json(force=True)
    if not radio.connected:
        return jsonify({"ok": False, "error": "not connected"}), 400
    try:
        audio.start_tx() if bool(d.get("down")) else audio.stop_tx()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/debug/hexdump", methods=["POST"])
def api_hexdump():
    d = request.get_json(force=True)
    radio.hexdump = bool(d.get("on"))
    radio._log("INFO", "raw hex dump " + ("ON" if radio.hexdump else "OFF"))
    return jsonify({"ok": True})


@app.route("/api/debug/clear", methods=["POST"])
def api_debug_clear():
    radio.debug_log.clear()
    return jsonify({"ok": True})


@app.route("/api/sstv/modes")
def api_sstv_modes():
    if not SSTV_AVAILABLE:
        return jsonify({"available": False, "error": str(SSTV_ERR), "modes": []})
    return jsonify({"available": True, "modes": list(sstv_mod.ENCODE_MODES.keys())})


@app.route("/api/sstv/callsign", methods=["POST"])
def api_sstv_callsign():
    d = request.get_json(force=True)
    radio.sstv_callsign = str(d.get("callsign", "")).strip()
    radio.persist()
    return jsonify({"ok": True})


@app.route("/api/sstv/overlay", methods=["POST"])
def api_sstv_overlay():
    d = request.get_json(force=True)
    radio.sstv_overlay = bool(d.get("on"))
    radio.persist()
    return jsonify({"ok": True})


@app.route("/api/sstv/wav", methods=["POST"])
def api_sstv_wav():
    """Render the SSTV signal (image + FSK-ID) to a WAV, no radio involved.

    Lets the signal be fed straight into MMSSTV/QSSTV to tell a decoder-side
    problem apart from an over-the-air one.
    """
    d = request.get_json(force=True)
    try:
        import io as _io
        from scipy.io import wavfile
        b = d["image"]
        if "," in b:
            b = b.split(",", 1)[1]
        cs = str(d.get("callsign", radio.sstv_callsign or "")).strip()
        samples = sstv_mod.encode_image(base64.b64decode(b),
                                        d.get("mode", "Martin M1"),
                                        48000, fskid=cs or None,
                                        overlay=(cs or None) if d.get("overlay") else None)
        buf = _io.BytesIO()
        wavfile.write(buf, 48000, samples)
        radio._log("EVENT", "SSTV WAV exported (%.1f s, FSK-ID: %s)"
                   % (len(samples) / 48000, cs or "(none)"))
        return jsonify({"ok": True, "wav": base64.b64encode(buf.getvalue()).decode()})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/sstv/selftest", methods=["POST"])
def api_sstv_selftest():
    d = request.get_json(force=True)
    try:
        b = d["image"]
        if "," in b:
            b = b.split(",", 1)[1]
        audio.sstv_selftest(base64.b64decode(b), d.get("mode", "Martin M1"))
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/sstv/decodefile", methods=["POST"])
def api_sstv_decodefile():
    d = request.get_json(force=True)
    try:
        b = d["audio"]
        if "," in b:
            b = b.split(",", 1)[1]
        audio.sstv_decode_file(base64.b64decode(b))
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/sstv/tx", methods=["POST"])
def api_sstv_tx():
    d = request.get_json(force=True)
    if not radio.connected:
        return jsonify({"ok": False, "error": "not connected"}), 400
    try:
        img_b64 = d["image"]
        if "," in img_b64:
            img_b64 = img_b64.split(",", 1)[1]
        # the UI sends the callsign with the request so it can never lag behind
        # what is typed in the field
        if "callsign" in d:
            cs = str(d.get("callsign", "")).strip()
            if cs != radio.sstv_callsign:
                radio.sstv_callsign = cs
                radio.persist()
        audio.start_sstv_tx(base64.b64decode(img_b64), d.get("mode", "Martin M1"),
                            fskid=radio.sstv_callsign or None,
                            overlay=(radio.sstv_callsign or None)
                            if d.get("overlay") else None)
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/sstv/tx/stop", methods=["POST"])
def api_sstv_tx_stop():
    audio.stop_sstv_tx()
    return jsonify({"ok": True})


@app.route("/api/sstv/rx", methods=["POST"])
def api_sstv_rx():
    d = request.get_json(force=True)
    try:
        audio.set_sstv_rx(bool(d.get("on")))
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/sstv/decode", methods=["POST"])
def api_sstv_decode():
    try:
        audio.sstv_decode_now()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/sstv/image")
def api_sstv_image():
    return jsonify({"image": audio.sstv_last_image, "mode": audio.sstv_last_mode})


@app.route("/api/scan/start", methods=["POST"])
def api_scan_start():
    d = request.get_json(force=True)
    try:
        radio.start_scan(float(d["start"]), float(d["end"]), float(d["step"]),
                         int(d.get("dwell_ms", 250)), d.get("method", "rssi"),
                         int(d.get("rssi_threshold", 64)), int(d.get("squelch", 1)),
                         int(d.get("bw", 0)), bool(d.get("stop_on_active", False)))
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/scan/stop", methods=["POST"])
def api_scan_stop():
    radio.stop_scan()
    return jsonify({"ok": True})


@app.route("/")
def index():
    return Response(INDEX_HTML, mimetype="text/html")


INDEX_HTML = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>kv4p HT - Web Client</title>
<style>
:root{--bg:#0e1116;--panel:#171b22;--line:#262c36;--fg:#e6e9ee;--mut:#8a93a2;--acc:#39d353;--err:#f85149;--blue:#4f9dff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
header{padding:14px 18px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:10px;flex-wrap:wrap}
header h1{font-size:16px;margin:0;font-weight:600}#dot{width:10px;height:10px;border-radius:50%;background:var(--err)}#dot.on{background:var(--acc)}
main{display:grid;grid-template-columns:340px 1fr;gap:16px;padding:16px;max-width:1320px;margin:0 auto}@media(max-width:820px){main{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:16px}
.panel h2{font-size:12px;text-transform:uppercase;letter-spacing:.06em;color:var(--mut);margin:0 0 12px}
label{display:block;font-size:12px;color:var(--mut);margin:8px 0 3px}
input,select{width:100%;background:#0c0f14;color:var(--fg);border:1px solid var(--line);border-radius:7px;padding:8px;font-size:13px}
.row{display:flex;gap:8px}.row>*{flex:1}
button{background:#222835;color:var(--fg);border:1px solid var(--line);border-radius:7px;padding:9px 12px;font-size:13px;cursor:pointer}
button:hover{border-color:#3a424f}button.primary{background:var(--blue);border-color:var(--blue);color:#001;font-weight:600}
button.danger{background:#3a1d1d;border-color:#5a2a2a;color:#ffb4b4}button.go{background:var(--acc);border-color:var(--acc);color:#001;font-weight:600}
button.tx{background:#4a1414;border-color:#7a2222;color:#ffd0d0;font-weight:600}button.tx.active{background:var(--err);color:#fff}
button:disabled{opacity:.45;cursor:not-allowed}.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.meterwrap{height:20px;background:#0c0f14;border:1px solid var(--line);border-radius:6px;overflow:hidden}
#meter{height:100%;width:0%;background:linear-gradient(90deg,#2ea043,#e3b341,#f85149);transition:width .12s}
.kv{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px dashed #20262f;font-size:12.5px}.kv span:first-child{color:var(--mut)}
table{width:100%;border-collapse:collapse;font-size:12.5px}th,td{text-align:left;padding:5px 8px;border-bottom:1px solid var(--line)}
th{color:var(--mut);font-weight:500;position:sticky;top:0;background:var(--panel)}tr.active td{color:var(--acc);font-weight:600}
.scanbox{max-height:280px;overflow:auto;border:1px solid var(--line);border-radius:8px;margin-top:10px}
.hint{font-size:11.5px;color:var(--mut);margin-top:6px}.progress{height:6px;background:#0c0f14;border-radius:4px;overflow:hidden;margin-top:8px}.progress>div{height:100%;background:var(--blue);width:0%}
.badge{font-size:11px;padding:2px 7px;border-radius:20px;background:#20262f;color:var(--mut)}.badge.on{background:#13351d;color:var(--acc)}.badge.tx{background:#3a1414;color:#ff8f8f}
#dbg{font-size:11px;color:var(--mut);margin-top:8px;word-break:break-all;min-height:14px}
pre#console{margin:0;height:220px;overflow:auto;background:#0c0f14;border:1px solid var(--line);border-radius:8px;padding:10px;font:11.5px/1.4 ui-monospace,Consolas,monospace;color:#cdd3db;white-space:pre-wrap}
.chk{display:flex;gap:14px;flex-wrap:wrap}.chk label{margin:0;color:var(--fg)}
.freqbig{font:700 46px/1 ui-monospace,Consolas,monospace;text-align:center;letter-spacing:1px;color:var(--fg);margin:2px 0}
.freqbig small{font-size:16px;color:var(--mut);font-weight:500;letter-spacing:0}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:8px;border-radius:5px;
 background:linear-gradient(90deg,#1d3b8a,#4f9dff);border:1px solid var(--line);padding:0;margin:10px 0}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;
 background:var(--blue);border:2px solid #0c0f14;cursor:pointer}
input[type=range]::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:var(--blue);border:2px solid #0c0f14;cursor:pointer}
.stepbtn{font:600 18px/1 system-ui;min-width:52px;padding:12px 0}
.keycap{display:inline-block;min-width:60px;text-align:center;padding:6px 12px;border:1px solid var(--line);
 border-bottom-width:3px;border-radius:7px;background:#0c0f14;font:600 13px ui-monospace,Consolas,monospace;color:var(--blue)}
.rangelbl{display:flex;justify-content:space-between;font-size:11px;color:var(--mut)}
.tabbtn{background:#141922;color:var(--mut);border:1px solid var(--line);border-bottom:none;
 border-radius:7px 7px 0 0;padding:7px 14px;font-size:12.5px}
.tabbtn.tabon{background:var(--panel);color:var(--fg);border-color:var(--blue);font-weight:600}
</style></head><body>
<header><div id="dot"></div><h1>kv4p HT — Web Client</h1>
<span id="call-badge" class="badge on" style="display:none">📻 —</span>
<span id="lbl" class="badge">disconnected</span>
<span id="txb" class="badge">TX: off</span><span id="rxb" class="badge">RX: off</span>
<span id="sqb" class="badge">SQ: —</span><span id="pptb" class="badge" style="margin-left:auto">PHYS PTT: off</span></header>
<main>
<div>
 <div class="panel"><h2>Connection</h2>
  <label>Serial port (COM)</label><div class="row"><select id="port"></select><button style="flex:0 0 auto" onclick="loadPorts()">↻</button></div>
  <label>Baud (editable)</label><input id="baud" list="bauds" value="115200">
  <datalist id="bauds"><option value="115200"><option value="921600"><option value="auto"></datalist>
  <label style="margin-top:8px"><input type="checkbox" id="reset_esp" checked style="width:auto"> reset ESP32 on connect</label>
  <div class="btns"><button id="bc" class="primary" onclick="connect()">Connect</button>
  <button id="bd" class="danger" onclick="disconnect()" disabled>Disconnect</button></div>
  <label style="margin-top:8px">Station callsign (optional — your station ID, used everywhere)</label>
  <input id="callsign" placeholder="e.g. IZ0ABC" onchange="setCall()" style="text-transform:uppercase">
  <div class="hint">Shown in the header and on every transmission; sent as the
   FSK-ID on SSTV. Voice FM carries no automatic ID — announce it by voice.</div>
  <div id="dbg"></div></div>
 <div class="panel"><h2>Voice audio</h2>
  <div id="au" class="hint" style="display:none;color:var(--err)"></div>
  <label>Input (mic)</label><select id="in_dev"></select>
  <label>Output (speaker)</label><select id="out_dev"></select>
  <div class="btns"><button style="flex:0 0 auto" onclick="loadAudioDevices()">↻</button>
   <button onclick="testTone()">🔊 Test tone</button>
   <button id="rxt" class="go" onclick="toggleRx()">Monitor RX ▶</button></div>
  <button id="pttbtn" class="tx" style="width:100%;margin-top:8px;padding:16px"
   onmousedown="ptt(true)" onmouseup="ptt(false)" onmouseleave="ptt(false)"
   ontouchstart="event.preventDefault();ptt(true)" ontouchend="ptt(false)">🎙 HOLD TO TALK</button>
  <div class="row" style="align-items:center;margin-top:8px">
   <div style="flex:2">Keyboard PTT: <span class="keycap" id="pttkeycap">Space</span></div>
   <button id="setkeybtn" onclick="bindPttKey()">Set key</button>
  </div>
  <div class="hint">Hold the key (default Space) to transmit. Ignored while typing in a field.</div>
  <div class="hint">Audio is Opus/ADPCM (firmware native). Half-duplex.</div></div>
 <div class="panel"><h2>Device</h2>
  <div class="kv"><span>Firmware ver</span><span id="d-ver">—</span></div>
  <div class="kv"><span>RF module</span><span id="d-rf">—</span></div>
  <div class="kv"><span>Range (MHz)</span><span id="d-rng">—</span></div>
  <div class="kv"><span>Mode / err</span><span id="d-mode">—</span></div>
  <div class="kv"><span>Applied seq</span><span id="d-seq">—</span></div>
  <div class="kv"><span>Applied</span><span id="d-applied">—</span></div>
  <div class="kv"><span>Baud</span><span id="d-baud">—</span></div></div>
 <div class="panel"><h2>S-meter (RSSI)</h2><div class="meterwrap"><div id="meter"></div></div>
  <div class="kv" style="margin-top:8px"><span>RSSI (0–255)</span><span id="d-rssi">—</span></div>
  <div class="kv"><span>Squelch state</span><span id="d-sq">—</span></div>
  <div class="kv"><span>RX audio frames</span><span id="d-rx">0</span></div></div>
</div>
<div>
 <div class="panel"><h2>Tuning</h2>
  <div class="freqbig" id="freq-big">—<small> MHz</small></div>
  <input type="range" id="fslider" min="134" max="174" step="0.005" value="145.5"
   oninput="sliderTune(this.value)">
  <div class="rangelbl"><span id="rlo">—</span><span id="rhi">—</span></div>
  <div class="row" style="align-items:end;margin-top:6px">
   <button class="stepbtn primary" onclick="stepFreq(-1)">−</button>
   <div style="flex:2"><label>Step (MHz)</label><input id="fstep" value="0.025"></div>
   <button class="stepbtn primary" onclick="stepFreq(1)">+</button>
  </div>
  <div class="btns" style="margin-top:8px">
   <button onclick="setStep(0.0125)">12.5k</button><button onclick="setStep(0.025)">25k</button>
   <button onclick="setStep(0.1)">100k</button><button onclick="setStep(1)">1M</button>
  </div>
  <div class="hint">Slider and ± retune the radio live within the module range.</div></div>
 <div class="panel"><h2>Manual tune</h2>
  <div class="row"><div><label>RX freq (MHz)</label><input id="freq_rx" value="145.500"></div>
   <div><label>TX freq (blank=same)</label><input id="freq_tx" placeholder="same"></div></div>
  <div class="row"><div><label>Bandwidth</label><select id="bw"><option value="0">Wide (25k)</option><option value="1">Narrow (12.5k)</option></select></div>
   <div><label>Squelch threshold: <b id="sqval">0</b> (0=open)</label>
    <input id="squelch" type="range" min="0" max="8" value="0"
     oninput="g('sqval').textContent=this.value" onchange="tune()"></div></div>
  <div class="row"><div><label>CTCSS TX</label><select id="ctcss_tx" onchange="tune()"></select></div>
   <div><label>CTCSS RX</label><select id="ctcss_rx" onchange="tune()"></select></div></div>
  <div class="btns"><button class="go" onclick="tune()">Tune / Listen</button><button onclick="stopRadio()">Stop</button></div>
  <label style="margin-top:10px">Control flags (sent live)</label>
  <div class="chk">
   <label><input type="checkbox" id="tx_allowed" onchange="flags()" style="width:auto"> <b>TX allowed</b></label>
   <label><input type="checkbox" id="high_power" onchange="flags()" style="width:auto"> high power</label>
   <label><input type="checkbox" id="f_pre" onchange="flags()" style="width:auto"> pre-emph</label>
   <label><input type="checkbox" id="f_high" checked onchange="flags()" style="width:auto"> high-pass</label>
   <label><input type="checkbox" id="f_low" checked onchange="flags()" style="width:auto"> low-pass</label>
  </div>
  <div class="hint"><b>TX allowed</b> must be ON to transmit (safety flag, persisted in the radio).</div></div>
 <div class="panel"><h2>Memories</h2>
  <div class="row"><input id="mem_name" placeholder="name (e.g. R0 repeater)">
   <button class="go" style="flex:0 0 auto" onclick="memSave()">★ Save current</button></div>
  <div class="scanbox" style="max-height:220px"><table>
   <thead><tr><th>Name</th><th>RX</th><th>SQ</th><th></th></tr></thead><tbody id="mem_rows"></tbody></table></div>
  <div class="hint">Saved to a JSON file in your home folder. Click a row to load &amp; tune.</div></div>
 <div class="panel"><h2>Frequency scan</h2>
  <div class="row"><div><label>Start</label><input id="s_start" value="144.000"></div>
   <div><label>End</label><input id="s_end" value="146.000"></div><div><label>Step</label><input id="s_step" value="0.025"></div></div>
  <div class="hint" id="s_range">module range: —</div>
  <div class="row"><div><label>Dwell (ms)</label><input id="s_dwell" type="number" value="250"></div>
   <div><label>Detect by</label><select id="s_method"><option value="squelch">Squelch open</option><option value="rssi">RSSI threshold</option></select></div>
   <div><label>RSSI thr</label><input id="s_thr" type="number" value="64"></div></div>
  <label style="margin-top:6px"><input type="checkbox" id="s_stop" style="width:auto"> stop on first active</label>
  <div class="btns"><button id="sg" class="go" onclick="scanStart()">Start scan</button>
   <button id="ss" class="danger" onclick="scanStop()" disabled>Stop</button><button onclick="clr()">Clear</button></div>
  <div class="progress"><div id="sp"></div></div><div class="hint" id="sst">idle</div>
  <div class="btns" style="margin:8px 0 0">
   <button id="tab-all" class="tabbtn tabon" onclick="scanTab('all')">All results</button>
   <button id="tab-act" class="tabbtn" onclick="scanTab('act')">Active <span id="act-count">0</span></button></div>
  <div id="pane-all" class="scanbox"><table><thead><tr><th>Freq</th><th>RSSI peak</th><th>Open</th><th>Status</th></tr></thead><tbody id="sr"></tbody></table></div>
  <div id="pane-act" class="scanbox" style="display:none"><table><thead><tr><th>Freq (MHz)</th><th>RSSI peak</th><th>Detected by</th></tr></thead><tbody id="sr_act"></tbody></table></div></div>
 <div class="panel"><h2>SSTV (images)</h2>
  <div id="sstv-unavail" class="hint" style="display:none;color:var(--err)"></div>
  <label>Mode</label><select id="sstv_mode"></select>
  <div class="hint">FSK-ID uses the <b>Station callsign</b> set in Connection (empty = no ID).</div>
  <label style="margin-top:6px"><input type="checkbox" id="sstv_overlay" onchange="setOverlay()" style="width:auto">
   burn callsign into the image (optional — readable by any decoder)</label>
  <div class="btns">
   <input type="file" id="sstv_file" accept="image/*" style="flex:2;padding:5px">
   <button class="tx" onclick="sstvSend()">📤 Send image</button></div>
  <div class="progress"><div id="sstv_prog"></div></div>
  <div class="hint" id="sstv_txstat">TX idle</div>
  <div class="btns" style="margin-top:8px">
   <button id="sstv_rxbtn" class="go" onclick="sstvRx()">📥 Receive ▶</button>
   <button onclick="sstvDecode()">Decode now</button></div>
  <label style="margin-top:8px">Test without a radio</label>
  <div class="btns">
   <button class="go" onclick="sstvSelftest()">🔄 Self-test (encode→decode selected image)</button>
   <button onclick="sstvWav()">💾 Save as WAV</button></div>
  <a id="sstv_wavdl" style="display:none;font-size:12px;color:var(--blue)" download="sstv_tx.wav">⬇ download sstv_tx.wav</a>
  <div class="btns" style="margin-top:6px"><input type="file" id="sstv_wav" accept="audio/*,.wav" style="flex:2;padding:5px">
   <button onclick="sstvDecodeFile()">Decode WAV</button></div>
  <div class="hint" id="sstv_rxstat">RX idle</div>
  <img id="sstv_img" style="width:100%;border:1px solid var(--line);border-radius:8px;margin-top:8px;display:none">
  <a id="sstv_dl" style="display:none;font-size:12px;color:var(--blue)" download="sstv.png">⬇ save image</a>
  <div class="hint">Sends/receives images over the radio. Note: the kv4p voice codec
   is lossy, so quality is limited; Martin M1 / Robot 36 are the most robust.
   Decode currently supports Martin M1/M2; other modes decode is experimental.</div></div>
 <div class="panel"><h2>Debug console</h2>
  <div class="btns" style="margin:0 0 8px"><button onclick="clrDbg()">Clear</button>
   <label style="margin:0;align-self:center"><input type="checkbox" id="dauto" checked style="width:auto"> autoscroll</label>
   <label style="margin:0;align-self:center"><input type="checkbox" id="dhex" onchange="toggleHex()" style="width:auto"> raw hex</label></div>
  <pre id="console"></pre>
  <div class="hint">HELLO/DEVICE_STATE events + firmware debug + boot text.</div></div>
</div></main>
<script>
let connected=false,txHeld=false,rxOn=false,txAllowed=false;
const CTCSS=[67.0,71.9,74.4,77.0,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100.0,103.5,107.2,110.9,114.8,118.8,123.0,127.3,131.8,136.5,141.3,146.2,151.4,156.7,162.2,167.9,173.8,179.9,186.2,192.8,203.5,210.7,218.1,225.7,233.6,241.8,250.3];
function fillCtcss(id){const e=document.getElementById(id);e.innerHTML='<option value="0">None</option>';
 CTCSS.forEach((hz,i)=>{const o=document.createElement('option');o.value=i+1;o.textContent=hz.toFixed(1)+' Hz';e.appendChild(o)})}
const g=id=>document.getElementById(id);
async function jp(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});return r.json()}
async function jg(u){return (await fetch(u)).json()}
async function loadPorts(){const p=await jg('/api/ports');const s=g('port');const c=s.value;s.innerHTML='';
 if(!p.length){const o=document.createElement('option');o.textContent='(none)';o.value='';s.appendChild(o)}
 p.forEach(x=>{const o=document.createElement('option');o.value=x.device;o.textContent=x.device+' — '+x.description;s.appendChild(o)});if(c)s.value=c}
async function loadAudioDevices(){const d=await jg('/api/audio/devices');const w=g('au');
 if(!d.available){w.style.display='block';w.textContent='Audio off: '+(d.error||'')+' — pip install sounddevice numpy';return}
 w.style.display='none';const fill=(id,l)=>{const e=g(id);e.innerHTML='';const o=document.createElement('option');o.value='';o.textContent='(default)';e.appendChild(o);
  l.forEach(x=>{const t=document.createElement('option');t.value=x.index;t.textContent=x.index+': '+x.name;e.appendChild(t)})};fill('in_dev',d.input);fill('out_dev',d.output)}
async function acfg(){await jp('/api/audio/config',{input:g('in_dev').value,output:g('out_dev').value})}
async function connect(){g('dbg').textContent='connecting…';const r=await jp('/api/connect',{port:g('port').value,baud:g('baud').value,reset:g('reset_esp').checked});
 g('dbg').textContent=r.ok?('connected @ '+r.baud):('ERROR: '+r.error);if(r.ok){acfg();flags()}}
async function disconnect(){await jp('/api/disconnect')}
async function tune(){await jp('/api/tune',{freq_rx:g('freq_rx').value,freq_tx:g('freq_tx').value,bw:parseInt(g('bw').value),
 squelch:parseInt(g('squelch').value),ctcss_tx:parseInt(g('ctcss_tx').value),ctcss_rx:parseInt(g('ctcss_rx').value)})}
let curFreq=145.5,freqLo=134,freqHi=174,sliderTimer=null,userSliding=false,pttKey='Space',kbHeld=false,bindingKey=false,rangePrefilled=false,ctrlPrefilled=false,pttKeyLoaded=false;
function clampF(v){return Math.min(freqHi,Math.max(freqLo,v))}
function fmt(v){return (Math.round(v*1000)/1000).toFixed(4)}
function setFreqUI(v){curFreq=clampF(v);g('freq-big').innerHTML=fmt(curFreq)+'<small> MHz</small>';g('freq_rx').value=fmt(curFreq)}
function sliderTune(v){userSliding=true;curFreq=clampF(parseFloat(v));g('freq-big').innerHTML=fmt(curFreq)+'<small> MHz</small>';g('freq_rx').value=fmt(curFreq);
 clearTimeout(sliderTimer);sliderTimer=setTimeout(()=>{tune();setTimeout(()=>userSliding=false,300)},120)}
function stepFreq(dir){const st=parseFloat(g('fstep').value)||0.025;setFreqUI(curFreq+dir*st);g('fslider').value=curFreq;tune()}
function setStep(s){g('fstep').value=s}
function keyName(c){return c==='Space'?'Space':c.replace('Key','').replace('Digit','').replace('Arrow','')||c}
function bindPttKey(){bindingKey=true;g('setkeybtn').textContent='press a key…'}
document.addEventListener('keydown',e=>{
 if(bindingKey){e.preventDefault();pttKey=e.code;g('pttkeycap').textContent=keyName(pttKey);g('setkeybtn').textContent='Set key';bindingKey=false;pttKeyLoaded=true;jp('/api/settings/pttkey',{key:pttKey});return}
 const tag=(document.activeElement&&document.activeElement.tagName)||'';
 if(tag==='INPUT'||tag==='SELECT'||tag==='TEXTAREA')return;
 if(e.code===pttKey){e.preventDefault();if(!kbHeld){kbHeld=true;ptt(true)}}});
document.addEventListener('keyup',e=>{if(e.code===pttKey){e.preventDefault();if(kbHeld){kbHeld=false;ptt(false)}}});
async function stopRadio(){await jp('/api/stop')}
async function flags(){await jp('/api/flags',{tx_allowed:g('tx_allowed').checked,high_power:g('high_power').checked,
 pre:g('f_pre').checked,high:g('f_high').checked,low:g('f_low').checked})}
async function toggleRx(){const r=await jp('/api/audio/rx',{on:!rxOn});if(!r.ok)g('dbg').textContent='RX ERR: '+r.error}
async function testTone(){await acfg();const r=await jp('/api/audio/test');g('dbg').textContent=r.ok?'test tone sent to output device':('tone ERR: '+r.error)}
async function ptt(down){if(!connected)return;
 if(down&&!txAllowed){g('dbg').textContent='⚠ Enable "TX allowed" to transmit';
  const cb=g('tx_allowed');cb.style.outline='2px solid var(--err)';setTimeout(()=>cb.style.outline='',1500);return}
 if(down){if(txHeld)return;txHeld=true;await acfg();const r=await jp('/api/voice/ptt',{down:true});
 if(!r.ok){txHeld=false;g('dbg').textContent='TX ERR: '+r.error}}else{if(!txHeld)return;txHeld=false;await jp('/api/voice/ptt',{down:false})}}
async function scanStart(){const r=await jp('/api/scan/start',{start:g('s_start').value,end:g('s_end').value,step:g('s_step').value,
 dwell_ms:parseInt(g('s_dwell').value),method:g('s_method').value,rssi_threshold:parseInt(g('s_thr').value),
 squelch:parseInt(g('squelch').value),bw:parseInt(g('bw').value),stop_on_active:g('s_stop').checked});if(!r.ok)g('sst').textContent='ERR: '+r.error}
async function scanStop(){await jp('/api/scan/stop')}
function clr(){g('sr').innerHTML=''}
async function clrDbg(){await jp('/api/debug/clear');g('console').textContent='';_dn=-1}
async function toggleHex(){await jp('/api/debug/hexdump',{on:g('dhex').checked})}
let sstvRxOn=false,_lastSv='',callLoaded=false;
async function sstvModes(){const d=await jg('/api/sstv/modes');const u=g('sstv-unavail');
 if(!d.available){u.style.display='block';u.textContent='SSTV off: '+(d.error||'')+' — pip install PySSTV Pillow scipy';return}
 u.style.display='none';const e=g('sstv_mode');e.innerHTML='';d.modes.forEach(m=>{const o=document.createElement('option');o.value=m;o.textContent=m;e.appendChild(o)})}
async function sstvSend(){const f=g('sstv_file').files[0];if(!f){g('sstv_txstat').textContent='pick an image first';return}
 if(!txAllowed){g('sstv_txstat').textContent='enable "TX allowed" first';return}
 const rd=new FileReader();rd.onload=async()=>{const r=await jp('/api/sstv/tx',
   {image:rd.result,mode:g('sstv_mode').value,callsign:g('callsign').value,overlay:g('sstv_overlay').checked});
  g('sstv_txstat').textContent=r.ok?'transmitting…':('ERR: '+r.error)};rd.readAsDataURL(f)}
async function sstvRx(){const r=await jp('/api/sstv/rx',{on:!sstvRxOn});if(!r.ok)g('sstv_rxstat').textContent='ERR: '+r.error}
async function sstvDecode(){const r=await jp('/api/sstv/decode');if(!r.ok)g('sstv_rxstat').textContent='ERR: '+r.error}
async function setCall(){await jp('/api/sstv/callsign',{callsign:g('callsign').value})}
async function setOverlay(){await jp('/api/sstv/overlay',{on:g('sstv_overlay').checked})}
async function sstvWav(){const f=g('sstv_file').files[0];if(!f){g('sstv_rxstat').textContent='pick an image first';return}
 g('sstv_rxstat').textContent='rendering WAV…';
 const rd=new FileReader();rd.onload=async()=>{
  const r=await jp('/api/sstv/wav',{image:rd.result,mode:g('sstv_mode').value,callsign:g('callsign').value,overlay:g('sstv_overlay').checked});
  if(!r.ok){g('sstv_rxstat').textContent='ERR: '+r.error;return}
  const a=g('sstv_wavdl');a.href='data:audio/wav;base64,'+r.wav;a.style.display='';
  g('sstv_rxstat').textContent='WAV ready — feed it to MMSSTV to test the FSK-ID';
  a.click()};rd.readAsDataURL(f)}
async function sstvSelftest(){const f=g('sstv_file').files[0];if(!f){g('sstv_rxstat').textContent='pick an image first';return}
 const rd=new FileReader();rd.onload=async()=>{const r=await jp('/api/sstv/selftest',{image:rd.result,mode:g('sstv_mode').value});
  g('sstv_rxstat').textContent=r.ok?'self-test running…':('ERR: '+r.error)};rd.readAsDataURL(f)}
async function sstvDecodeFile(){const f=g('sstv_wav').files[0];if(!f){g('sstv_rxstat').textContent='pick a WAV file';return}
 const rd=new FileReader();rd.onload=async()=>{const r=await jp('/api/sstv/decodefile',{audio:rd.result});
  g('sstv_rxstat').textContent=r.ok?'decoding file…':('ERR: '+r.error)};rd.readAsDataURL(f)}
async function sstvImg(){const d=await jg('/api/sstv/image');if(d.image){const src='data:image/png;base64,'+d.image;
 const im=g('sstv_img');im.src=src;im.style.display='';const a=g('sstv_dl');a.href=src;a.style.display=''}}
async function memLoad(){const m=await jg('/api/memories');const tb=g('mem_rows');tb.innerHTML='';
 m.forEach(x=>{const tr=document.createElement('tr');
  tr.innerHTML='<td>'+x.name+'</td><td>'+x.freq_rx.toFixed(4)+'</td><td>'+x.squelch+'</td>'
   +'<td><button style="padding:3px 8px" onclick="event.stopPropagation();memDel('+x.id+')">✕</button></td>';
  tr.style.cursor='pointer';tr.onclick=()=>memApply(x);tb.appendChild(tr)})}
async function memSave(){const body={name:g('mem_name').value,freq_rx:parseFloat(g('freq_rx').value),
  freq_tx:g('freq_tx').value?parseFloat(g('freq_tx').value):parseFloat(g('freq_rx').value),
  bw:parseInt(g('bw').value),squelch:parseInt(g('squelch').value),
  ctcss_tx:parseInt(g('ctcss_tx').value),ctcss_rx:parseInt(g('ctcss_rx').value)};
 await jp('/api/memories',body);g('mem_name').value='';memLoad()}
async function memDel(id){await jp('/api/memories/delete',{id});memLoad()}
async function memApply(x){g('freq_rx').value=x.freq_rx.toFixed(4);
 g('freq_tx').value=(x.freq_tx&&x.freq_tx!==x.freq_rx)?x.freq_tx.toFixed(4):'';
 g('bw').value=x.bw;g('squelch').value=x.squelch;g('sqval').textContent=x.squelch;
 g('ctcss_tx').value=x.ctcss_tx;g('ctcss_rx').value=x.ctcss_rx;setFreqUI(x.freq_rx);g('fslider').value=x.freq_rx;tune()}
function tuneTo(f){g('freq_rx').value=f.toFixed(4);setFreqUI(f);g('fslider').value=f;tune()}
function scanTab(w){const a=w==='all';g('pane-all').style.display=a?'':'none';g('pane-act').style.display=a?'none':'';
 g('tab-all').classList.toggle('tabon',a);g('tab-act').classList.toggle('tabon',!a)}
function renderScan(s){g('sg').disabled=s.running||!connected;g('ss').disabled=!s.running;
 g('sp').style.width=(s.total?Math.round((s.index+1)/s.total*100):0)+'%';
 const act=s.results.filter(r=>r.active);
 g('act-count').textContent=act.length;
 g('sst').textContent=s.running?('scanning '+(s.current!=null?s.current.toFixed(4):'')+' ('+(s.index+1)+'/'+s.total+')')
  :(s.results.length?('done — '+act.length+' active/'+s.results.length):'idle');
 const tb=g('sr');if(tb.childElementCount!==s.results.length){tb.innerHTML='';s.results.forEach(r=>{const t=document.createElement('tr');
  if(r.active)t.className='active';t.innerHTML='<td>'+r.freq.toFixed(4)+'</td><td>'+r.rssi_peak+'</td><td>'+(r.rx_frames?'●':'—')+'</td><td>'+(r.active?'● ACTIVE':'—')+'</td>';
  t.style.cursor='pointer';t.onclick=()=>tuneTo(r.freq);tb.appendChild(t)});tb.scrollTop=tb.scrollHeight}
 const ta=g('sr_act');if(ta.childElementCount!==act.length){ta.innerHTML='';
  if(!act.length){ta.innerHTML='<tr><td colspan="3" style="color:var(--mut)">no active channels yet</td></tr>';}
  act.forEach(r=>{const t=document.createElement('tr');t.className='active';
   t.innerHTML='<td>'+r.freq.toFixed(4)+'</td><td>'+r.rssi_peak+'</td><td>'+(r.rx_frames?'squelch open':'RSSI')+'</td>';
   t.style.cursor='pointer';t.onclick=()=>tuneTo(r.freq);ta.appendChild(t)})}}
let _dn=-1;
function renderConsole(log){if(log.length===_dn)return;_dn=log.length;const e=g('console');
 e.textContent=log.map(x=>x.t+'  ['+x.kind+']  '+x.text).join('\n');if(g('dauto').checked)e.scrollTop=e.scrollHeight}
async function poll(){try{const s=await jg('/api/status');connected=s.connected;
 g('dot').className=connected?'on':'';const l=g('lbl');l.textContent=connected?('connected '+(s.port||'')):'disconnected';l.className='badge'+(connected?' on':'');
 g('bc').disabled=connected;g('bd').disabled=!connected;g('pttbtn').disabled=!connected;
 const t=s.telemetry||{};
 g('d-ver').textContent=t.version??'—';
 g('d-rf').textContent=(t.rf_module_type==null?'—':('type '+t.rf_module_type+' ('+(t.module_status||'?')+')'));
 g('d-rng').textContent=(t.min_freq!=null?t.min_freq+'–'+t.max_freq:'—');
 if(t.min_freq!=null){g('s_range').textContent='module range: '+t.min_freq+'–'+t.max_freq+' MHz (scan clamped to this)';
  ['s_start','s_end','freq_rx','freq_tx'].forEach(id=>{const e=g(id);e.min=t.min_freq;e.max=t.max_freq});
  freqLo=t.min_freq;freqHi=t.max_freq;const sl=g('fslider');sl.min=t.min_freq;sl.max=t.max_freq;
  g('rlo').textContent=t.min_freq;g('rhi').textContent=t.max_freq;
  if(!rangePrefilled){g('s_start').value=t.min_freq.toFixed(3);g('s_end').value=t.max_freq.toFixed(3);rangePrefilled=true}}
 const c2=s.ctrl||{};
 g('d-applied').textContent='HP:'+(t.dev_high_power?'on':'off')+' · pre:'+(t.dev_pre?'on':'off')
  +' · TXok:'+(t.dev_tx_allowed?'on':'off')+' · SQ:'+(t.dev_squelch!=null?t.dev_squelch:'—');
 if(t.cur_freq_rx!=null){curFreq=t.cur_freq_rx;
  g('freq-big').innerHTML=t.cur_freq_rx.toFixed(4)+'<small> MHz</small>';
  if(!userSliding)g('fslider').value=t.cur_freq_rx;}
 g('d-mode').textContent=(t.mode==null?'—':(t.mode+' / err '+t.last_error));
 g('d-seq').textContent=t.applied_seq??'—';g('d-baud').textContent=s.baud??'—';
 const rssi=t.rssi_fresh?t.rssi:null;g('d-rssi').textContent=(t.rssi==null?'—':t.rssi)+(t.rssi_fresh?'':' (stale)');
 g('meter').style.width=(rssi!=null?(rssi/255*100).toFixed(0):0)+'%';
 g('d-sq').textContent=t.squelched?'SQUELCHED (quiet)':'OPEN (signal)';g('d-rx').textContent=t.rx_audio_frames??0;
 const sqb=g('sqb');sqb.textContent='SQ: '+(t.squelched?'closed':'OPEN');sqb.className='badge'+(t.squelched?'':' on');
 const ppt=g('pptb');ppt.textContent='PHYS PTT: '+(t.phys_ptt?'ON':'off');ppt.className='badge'+(t.phys_ptt?' on':'');
 const a=s.audio||{};rxOn=a.rx_on;g('rxt').textContent=rxOn?'Monitor RX ⏸':'Monitor RX ▶';
 const sv=a.sstv||{};sstvRxOn=!!sv.rx_on;
 g('sstv_rxbtn').textContent=sv.rx_on?('📥 Receiving ⏸ ('+(sv.buf_sec||0)+'s)'):'📥 Receive ▶';
 const tx=sv.tx||{};g('sstv_prog').style.width=(tx.progress||0)+'%';
 g('sstv_txstat').textContent=tx.running?('transmitting '+tx.progress+'% ('+(tx.mode||'')+')'):'TX idle';
 g('sstv_rxstat').textContent='RX: '+(sv.status||'idle');
 if(sv.have_image&&sv.status!==_lastSv){_lastSv=sv.status;sstvImg()}
 const rb=g('rxb');rb.textContent='RX: '+(rxOn?'ON':'off');rb.className='badge'+(rxOn?' on':'');
 const tb=g('txb');const txon=a.tx_on||t.tx_active||t.tx;const _cs=(s.ctrl||{}).sstv_callsign;
 tb.textContent='TX: '+(txon?('ON'+(_cs?' · '+_cs:'')):'off');tb.className='badge'+(txon?' tx':'');
 g('pttbtn').classList.toggle('active',!!(a.tx_on||t.tx_active));
 const c=s.ctrl||{};g('tx_allowed').checked=!!c.tx_allowed;g('high_power').checked=!!c.high_power;txAllowed=!!c.tx_allowed;
 g('f_pre').checked=!!c.filt_pre;g('f_high').checked=!!c.filt_high;g('f_low').checked=!!c.filt_low;
 if(!pttKeyLoaded&&c.ptt_key){pttKey=c.ptt_key;g('pttkeycap').textContent=keyName(pttKey);pttKeyLoaded=true}
 if(!callLoaded&&c.sstv_callsign!=null){g('callsign').value=c.sstv_callsign;
  g('sstv_overlay').checked=!!c.sstv_overlay;callLoaded=true}
 const cb=g('call-badge');if(c.sstv_callsign){cb.textContent='📻 '+c.sstv_callsign;cb.style.display=''}else{cb.style.display='none'}
 if(!ctrlPrefilled&&c.squelch!=null){g('squelch').value=c.squelch;g('sqval').textContent=c.squelch;
  g('bw').value=c.bw;g('ctcss_tx').value=c.ctcss_tx;g('ctcss_rx').value=c.ctcss_rx;ctrlPrefilled=true}
 g('pttbtn').textContent=txAllowed?'🎙 HOLD TO TALK':'🎙 HOLD TO TALK (enable TX allowed)';
 if(a.last_err)g('dbg').textContent=a.last_err;
 renderConsole(s.log||[]);renderScan(s.scan||{running:false,index:0,total:0,results:[]});
}catch(e){}}
fillCtcss('ctcss_tx');fillCtcss('ctcss_rx');sstvModes();loadPorts();loadAudioDevices();memLoad();setInterval(poll,300);poll();
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description="kv4p HT web client (protocol v2.2)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()
    url = f"http://{args.host}:{args.port}/"
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    print(f"kv4p-web build {APP_VERSION} running at {url}  (Ctrl+C to quit)")
    if not AUDIO_AVAILABLE:
        print(f"NOTE: voice disabled: {AUDIO_IMPORT_ERR}  -> pip install sounddevice numpy")
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        audio.shutdown()
        radio.disconnect()


if __name__ == "__main__":
    main()
