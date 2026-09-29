#!/usr/bin/env python3
"""Render recorded frames into a README demo GIF.

    python3 tools/render_demo.py FRAMES docs/media/explore.gif --rooms
    python3 tools/render_demo.py FRAMES docs/media/places.gif \\
        --title 'going to named places' --places ~/.casabot/places.yaml

Frames come from tools/record_demo.py: one .npz per second of sim time with the
occupancy grid, robot pose, Nav2's current plan and the latest lidar points,
all in the map frame. Drawn top-down, the way RViz shows it.

--rooms   finish by colouring the map by room, the way explore names them
--places  draw the saved places, and show which one the robot is heading to
"""

import argparse
import glob
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from casabot.rooms import segment_rooms  # noqa: E402

UNKNOWN = (58, 63, 71)
FREE = (242, 244, 247)
WALL = (13, 17, 23)
TRAIL = (79, 157, 255)
PLAN = (34, 197, 94)
SCAN = (255, 77, 77)
ROBOT = (31, 111, 235)
HEADER = (22, 27, 34)
TEXT = (230, 237, 243)
PIN = (234, 88, 12)
ROOMS = [(252, 205, 205), (200, 222, 252), (205, 238, 208), (253, 230, 190),
         (228, 210, 250), (196, 238, 238), (250, 212, 232), (226, 226, 200)]
PALETTE = [UNKNOWN, FREE, WALL, TRAIL, PLAN, SCAN, ROBOT, HEADER, TEXT, PIN, (255, 255, 255)] + ROOMS

WIDTH = 760          # px of map; height follows the map's aspect ratio
BAR = 34             # header bar
FPS = 12
MAX_FRAMES = 330     # ~27 s of GIF, whatever the run length


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('frames')
    ap.add_argument('out')
    ap.add_argument('--title', default='autonomous mapping')
    ap.add_argument('--rooms', action='store_true')
    ap.add_argument('--places')
    args = ap.parse_args()

    frames = [dict(np.load(f)) for f in sorted(glob.glob(f'{args.frames}/*.npz'))]
    places = {}
    if args.places:
        with open(os.path.expanduser(args.places)) as fh:
            places = yaml.safe_load(fh) or {}

    # One canvas for the whole run: the union of every map extent, plus a margin.
    x0 = min(f['origin'][0] for f in frames) - 0.3
    y0 = min(f['origin'][1] for f in frames) - 0.3
    x1 = max(f['origin'][0] + f['grid'].shape[1] * f['res'] for f in frames) + 0.3
    y1 = max(f['origin'][1] + f['grid'].shape[0] * f['res'] for f in frames) + 0.3
    scale = WIDTH / (x1 - x0)
    height = int((y1 - y0) * scale)
    us = x0 + (np.arange(WIDTH) + 0.5) / scale
    vs = y1 - (np.arange(height) + 0.5) / scale

    def px(x, y):
        return ((x - x0) * scale, BAR + (y1 - y) * scale)

    def sample(f, layer):
        """layer (shaped like f's grid) resampled onto the canvas; -1 outside it."""
        res, (ox, oy) = float(f['res']), f['origin']
        cv, rv = np.meshgrid(np.floor((us - ox) / res).astype(int),
                             np.floor((vs - oy) / res).astype(int))
        inside = (cv >= 0) & (cv < layer.shape[1]) & (rv >= 0) & (rv < layer.shape[0])
        out = np.full(cv.shape, -1, dtype=np.int16)
        out[inside] = layer[rv[inside], cv[inside]]
        return out

    try:
        font = ImageFont.load_default(size=17)
        small = ImageFont.load_default(size=14)
    except TypeError:
        font = small = ImageFont.load_default()

    pal_img = Image.new('P', (1, 1))
    pal_img.putpalette([c for rgb in PALETTE for c in rgb] + [0] * (768 - 3 * len(PALETTE)))

    def base(f, room_labels=None):
        cells = sample(f, f['grid'])
        rgb = np.empty(cells.shape + (3,), dtype=np.uint8)
        rgb[:] = UNKNOWN
        rgb[(cells >= 0) & (cells < 50)] = FREE
        if room_labels is not None:
            rooms = sample(f, room_labels)
            for k in range(1, room_labels.max() + 1):
                rgb[(rooms == k) & (cells >= 0) & (cells < 50)] = ROOMS[(k - 1) % len(ROOMS)]
        rgb[cells >= 50] = WALL
        im = Image.new('RGB', (WIDTH, BAR + height), HEADER)
        im.paste(Image.fromarray(rgb), (0, BAR))
        return im, ImageDraw.Draw(im)

    def header(d, right):
        d.text((12, 8), f'casabot  ·  {args.title}', fill=TEXT, font=font)
        d.text((WIDTH - 12 - d.textlength(right, font=font), 8), right, fill=TEXT, font=font)

    def pin(d, x, y, name):
        u, v = px(x, y)
        d.ellipse([u - 5, v - 5, u + 5, v + 5], fill=PIN, outline=(255, 255, 255), width=2)
        d.text((u + 8, v - 9), name, fill=WALL, font=small, stroke_width=2, stroke_fill=(255, 255, 255))

    step = max(1, math.ceil(len(frames) / MAX_FRAMES))
    shown = list(range(0, len(frames), step))
    if shown[-1] != len(frames) - 1:
        shown.append(len(frames) - 1)
    t0 = float(frames[0]['t'])
    trail = [tuple(f['pose'][:2]) for f in frames]

    images = []
    for k in shown:
        f = frames[k]
        im, d = base(f)
        if k > 0:
            d.line([px(*p) for p in trail[:k + 1]], fill=TRAIL, width=2)
        plan = f['plan']
        target = None
        if len(plan) > 1:
            d.line([px(*p) for p in plan], fill=PLAN, width=3)
            gx, gy = px(*plan[-1])
            d.ellipse([gx - 6, gy - 6, gx + 6, gy + 6], outline=PLAN, width=3)
            if places:
                target = min(places, key=lambda n: math.hypot(places[n]['x'] - plan[-1][0],
                                                              places[n]['y'] - plan[-1][1]))
        for name, p in places.items():
            pin(d, p['x'], p['y'], name)
        for sx, sy in f['scan'][::2]:
            u, v = px(sx, sy)
            d.rectangle([u - 1, v - 1, u + 1, v + 1], fill=SCAN)
        rx, ry, ryaw = f['pose']
        u, v = px(rx, ry)
        r = 0.16 * scale
        d.ellipse([u - r, v - r, u + r, v + r], fill=ROBOT, outline=(255, 255, 255), width=2)
        d.line([u, v, u + r * math.cos(ryaw), v - r * math.sin(ryaw)], fill=(255, 255, 255), width=3)

        secs = int(float(f['t']) - t0)
        clock = f'{secs // 60}:{secs % 60:02d} sim time'
        if places:
            header(d, (f'heading to {target}   ' if target else '') + clock)
        else:
            g = f['grid']
            header(d, f'{clock}   {int(((g >= 0) & (g < 50)).sum()) * float(f["res"]) ** 2:5.1f} m2 of floor mapped')
        images.append(im.quantize(palette=pal_img, dither=Image.Dither.NONE))

    hold = images[-1]
    if args.rooms:
        # The finished map, split into the rooms explore saves as places.
        f = frames[-1]
        labels, rooms = segment_rooms(f['grid'], float(f['res']))
        im, d = base(f, labels)
        res, (ox, oy) = float(f['res']), f['origin']
        for k, (row, col, area) in enumerate(rooms, start=1):
            pin(d, ox + (col + 0.5) * res, oy + (row + 0.5) * res, f'room_{k}')
        header(d, f'{len(rooms)} rooms found, saved as places')
        hold = im.quantize(palette=pal_img, dither=Image.Dither.NONE)
    images += [hold] * (FPS * 5)
    images[0].save(args.out, save_all=True, append_images=images[1:],
                   duration=int(1000 / FPS), loop=0, optimize=True)
    print(f'{len(frames)} recorded frames -> {len(images)} GIF frames, {WIDTH}x{BAR + height}px: {args.out}')


if __name__ == '__main__':
    main()
