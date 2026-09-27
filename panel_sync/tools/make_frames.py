#!/usr/bin/env python3
"""
GIF -> frames.bin for the LED wall's "frames" flash partition.

Usage (from panel_sync/):
    python3 tools/make_frames.py mygif.gif                 # 32px, GIF's own speed
    python3 tools/make_frames.py mygif.gif --size 16 --fps 20

Writes assets/frames.bin (format in main/frames.h) and assets/preview.png.
Transparent areas become black; shape is kept (the panel crops to fill the wall).
"""
import argparse, os, struct, sys, zlib
from PIL import Image, ImageSequence

PARTITION_SIZE = 0x200000        # must match "frames" in partitions.csv
HEADER = struct.Struct("<4sBBHHHHHI")   # magic, ver, rsv, count, w, h, fps, rsv, crc32 = 20 bytes

ap = argparse.ArgumentParser()
ap.add_argument("gif")
ap.add_argument("--size", type=int, default=32, help="longest side in pixels (default 32)")
ap.add_argument("--fps", type=int, help="override playback fps")
ap.add_argument("--out", default="assets/frames.bin")
ap.add_argument("--clock", type=int, default=30, help="Pico frame clock (FPS in ledwall_sync), default 30")
a = ap.parse_args()
MASTER_FPS = a.clock             # can't play faster than the Pico's frame clock

img = Image.open(a.gif)
w0, h0 = img.size
k = min(1.0, a.size / max(w0, h0))   # shrink big GIFs, never blow up small ones
W, H = max(1, round(w0 * k)), max(1, round(h0 * k))

frames, durs = [], []
for fr in ImageSequence.Iterator(img):
    rgba = fr.convert("RGBA")
    rgb = Image.alpha_composite(Image.new("RGBA", rgba.size, (0, 0, 0, 255)), rgba).convert("RGB")
    frames.append(rgb.resize((W, H), Image.BOX))
    durs.append(fr.info.get("duration", img.info.get("duration", 100)) or 100)

src_count = len(frames)
if a.fps:
    fps = a.fps
elif len(set(durs)) == 1:
    fps = max(1, round(1000 / durs[0]))
else:
    # Frames have different hold times: pick a rate fine enough for the shortest one.
    fps = max(1, round(1000 / min(durs)))
avg_fps = src_count * 1000 / sum(durs)
capped = fps > MASTER_FPS
if capped:
    if avg_fps > MASTER_FPS + 0.5:
        print(f"note: GIF runs at {avg_fps:.0f} fps, Pico clock is {MASTER_FPS}; "
              f"playing at real speed by skipping frames")
    fps = MASTER_FPS

if (len(set(durs)) > 1 or capped) and not a.fps:
    # Resample onto a fixed fps grid so long holds stay long and short frames stay short.
    starts, t = [], 0
    for d in durs:
        starts.append(t); t += d
    n_out = max(1, round(t * fps / 1000))
    out, j = [], 0
    for k in range(n_out):
        tk = k * 1000 / fps
        while j + 1 < len(starts) and starts[j + 1] <= tk:
            j += 1
        out.append(frames[j])
    frames = out
    print(f"note: resampled {src_count} frames to {len(frames)} at {fps} fps (keeps real timing)")

pixels = b"".join(f.tobytes() for f in frames)
total = HEADER.size + len(pixels)
if total > PARTITION_SIZE:
    sys.exit(f"Too big: {total // 1024}KB, partition is {PARTITION_SIZE // 1024}KB. Use --size 16 or fewer frames.")

os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
with open(a.out, "wb") as f:
    f.write(HEADER.pack(b"LEDA", 1, 0, len(frames), W, H, fps, 0, zlib.crc32(pixels)))
    f.write(pixels)
frames[0].resize((W * 8, H * 8), Image.NEAREST).save(os.path.join(os.path.dirname(a.out) or ".", "preview.png"))
orig_s = sum(durs) / 1000
loop_s = len(frames) / fps
print(f"{os.path.basename(a.gif)}")
timing = f"{durs[0]} ms/frame" if len(set(durs)) == 1 else f"{min(durs)}-{max(durs)} ms/frame"
print(f"  original : {w0}x{h0}, {src_count} frame{'s' if src_count != 1 else ''}, {orig_s:.2f} s ({timing})")
print(f"  on panel : {W}x{H}, {len(frames)} frame{'s' if len(frames) != 1 else ''} @ {fps} fps = {loop_s:.2f} s loop"
      + (f"  (speed x{orig_s / loop_s:.2f} vs original)" if abs(orig_s - loop_s) > 0.05 else ""))
def human(n):
    return f"{n / 1024 / 1024:.2f} MB" if n >= 1024 * 1024 else f"{n / 1024:.1f} KB"
print(f"  GIF file : {human(os.path.getsize(a.gif))} on your PC")
print(f"  on ESP32 : {human(total)} used, {human(PARTITION_SIZE - total)} free "
      f"of {human(PARTITION_SIZE)} ({100 * total / PARTITION_SIZE:.1f}% used)")
print(f"  -> {a.out}")
