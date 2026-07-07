#!/usr/bin/env python3
"""
Parse the unified binary recording from the vital-signs data logger
and save as a pickle file.

Usage:
    python parse_recording.py  recording.bin  [output.pkl]

Binary format:
    [32-byte file header]
    [tagged records...]

Record types:
    0x01  ECG + sensor     (1 + 88 = 89 bytes)
    0x02  Vital signs      (1 + 20 = 21 bytes)
    0x03  Disease prediction (1 + 80 = 81 bytes)
"""

import struct
import sys
import pickle
import numpy as np
from pathlib import Path

# ── Constants ──────────────────────────────────────────────────

FILE_MAGIC  = 0x48534456
HEADER_SIZE = 32

RECORD_ECG  = 0x01
RECORD_VIT  = 0x02
RECORD_PRED = 0x03

NUM_CLASSES = 17

# Struct format strings (little-endian, after type byte is consumed)
#
# pkt_sample_t assumed layout (88 bytes, no padding):
#   uint32  timestamp
#   float32 ecg[8]
#   float32 imu1: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
#   float32 imu2: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
#   float32 temp
#
# *** VERIFY imu_sample_t field order matches your data_types.h ***

ECG_FMT  = '<I 8f 6f 6f f'           # 4 + 32 + 24 + 24 + 4 = 88
VIT_FMT  = '<I 4f'                    # 4 + 16 = 20
PRED_FMT = '<I I f 17f'               # 4 + 4 + 4 + 68 = 80

ECG_PAYLOAD  = struct.calcsize(ECG_FMT)    # 88
VIT_PAYLOAD  = struct.calcsize(VIT_FMT)    # 20
PRED_PAYLOAD = struct.calcsize(PRED_FMT)   # 80

DISEASE_LABELS = [
    "426783006",  # Sinus rhythm
    "426177001",  # Sinus bradycardia
    "164934002",  # T wave abnormal
    "427393009",  # Sinus arrhythmia
    "713426002",  # Short QT
    "427084000",  # Sinus tachycardia
    "59118001",   # RBBB
    "164889003",  # Atrial fibrillation
    "59931005",   # Inverted T wave
    "47665007",   # Right axis deviation
    "445118002",  # Left anterior fascicular block
    "39732003",   # Left axis deviation
    "164890007",  # Atrial flutter
    "164909002",  # LBBB
    "270492004",  # 1st degree AV block
    "251146004",  # Low QRS voltages
    "284470004",  # Premature atrial complex
]

LABEL_NAMES = [
    "Sinus rhythm", "Sinus bradycardia", "T wave abnormal",
    "Sinus arrhythmia", "Short QT", "Sinus tachycardia",
    "RBBB", "Atrial fibrillation", "Inverted T wave",
    "Right axis deviation", "LAFB", "Left axis deviation",
    "Atrial flutter", "LBBB", "1st degree AV block",
    "Low QRS voltages", "PAC",
]


def parse_header(f):
    """Read and validate the 32-byte file header."""
    raw = f.read(HEADER_SIZE)
    if len(raw) < HEADER_SIZE:
        raise ValueError("File too short for header")

    magic, version, ecg_rate, ecg_ch, num_imu, num_cls = \
        struct.unpack_from('<I H H B B B', raw)

    if magic != FILE_MAGIC:
        raise ValueError(f"Bad magic: 0x{magic:08X} (expected 0x{FILE_MAGIC:08X})")

    return {
        'magic':               magic,
        'version':             version,
        'ecg_rate_hz':         ecg_rate,
        'ecg_channels':        ecg_ch,
        'num_imu':             num_imu,
        'num_disease_classes':  num_cls,
    }


def parse_records(f):
    """Parse all records after the header. Returns lists of dicts."""
    ecg_list, vit_list, pred_list = [], [], []

    while True:
        tag = f.read(1)
        if not tag:
            break
        rtype = tag[0]

        if rtype == RECORD_ECG:
            raw = f.read(ECG_PAYLOAD)
            if len(raw) < ECG_PAYLOAD:
                break
            vals = struct.unpack(ECG_FMT, raw)
            ecg_list.append(vals)

        elif rtype == RECORD_VIT:
            raw = f.read(VIT_PAYLOAD)
            if len(raw) < VIT_PAYLOAD:
                break
            vals = struct.unpack(VIT_FMT, raw)
            vit_list.append(vals)

        elif rtype == RECORD_PRED:
            raw = f.read(PRED_PAYLOAD)
            if len(raw) < PRED_PAYLOAD:
                break
            vals = struct.unpack(PRED_FMT, raw)
            pred_list.append(vals)

        else:
            print(f"[WARN] Unknown record type 0x{rtype:02X} at offset "
                  f"{f.tell() - 1}, stopping.")
            break

    return ecg_list, vit_list, pred_list


def build_output(header, ecg_list, vit_list, pred_list):
    """Convert parsed lists into a dict of numpy arrays."""
    result = {'header': header, 'disease_labels': DISEASE_LABELS,
              'label_names': LABEL_NAMES}

    # ── ECG records ────────────────────────────────────────────
    if ecg_list:
        arr = np.array(ecg_list, dtype=np.float64)
        result['ecg'] = {
            'timestamp_ms': arr[:, 0].astype(np.uint32),
            'ecg':          arr[:, 1:9],          # (N, 8)
            'imu1_accel':   arr[:, 9:12],         # (N, 3)
            'imu1_gyro':    arr[:, 12:15],        # (N, 3)
            'imu2_accel':   arr[:, 15:18],        # (N, 3)
            'imu2_gyro':    arr[:, 18:21],        # (N, 3)
            'temperature':  arr[:, 21],           # (N,)
        }
    else:
        result['ecg'] = None

    # ── Vitals records ─────────────────────────────────────────
    if vit_list:
        arr = np.array(vit_list, dtype=np.float64)
        result['vitals'] = {
            'timestamp_ms':    arr[:, 0].astype(np.uint32),
            'heart_rate_bpm':  arr[:, 1],
            'hrv_sdnn_ms':     arr[:, 2],
            'resp_rate_bpm':   arr[:, 3],
            'temperature_c':   arr[:, 4],
        }
    else:
        result['vitals'] = None

    # ── Prediction records ─────────────────────────────────────
    if pred_list:
        arr = np.array(pred_list, dtype=np.float64)
        result['predictions'] = {
            'timestamp_ms':  arr[:, 0].astype(np.uint32),
            'window_index':  arr[:, 1].astype(np.uint32),
            'inference_ms':  arr[:, 2],
            'probs':         arr[:, 3:3 + NUM_CLASSES],   # (M, 17)
        }
    else:
        result['predictions'] = None

    return result


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <recording.bin> [output.pkl]")
        sys.exit(1)

    in_path  = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 \
               else in_path.with_suffix('.pkl')

    print(f"Parsing: {in_path}")

    with open(in_path, 'rb') as f:
        header = parse_header(f)
        print(f"  Format v{header['version']}, "
              f"{header['ecg_channels']}ch ECG @ {header['ecg_rate_hz']} Hz, "
              f"{header['num_disease_classes']} disease classes")

        ecg_list, vit_list, pred_list = parse_records(f)

    print(f"  ECG samples:   {len(ecg_list)}")
    print(f"  Vitals records: {len(vit_list)}")
    print(f"  Predictions:    {len(pred_list)}")

    if ecg_list:
        duration_s = len(ecg_list) / header['ecg_rate_hz']
        print(f"  Duration:       {duration_s:.1f} s ({duration_s/60:.1f} min)")

    result = build_output(header, ecg_list, vit_list, pred_list)

    with open(out_path, 'wb') as f:
        pickle.dump(result, f, protocol=pickle.HIGHEST_PROTOCOL)

    print(f"Saved: {out_path} ({out_path.stat().st_size / 1024:.1f} KB)")


if __name__ == '__main__':
    main()