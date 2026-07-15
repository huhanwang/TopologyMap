#!/usr/bin/env python3
import argparse
import csv
import html
import json
import math
from pathlib import Path


def read_csv(path):
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def to_float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except ValueError:
        return default


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("debug_dir", nargs="?", default="offline_replay/out/fusedstatic_gnss_sdroute/global_debug")
    args = parser.parse_args()

    debug_dir = Path(args.debug_dir)
    gnss_rows = read_csv(debug_dir / "gnss.csv")
    route_rows = read_csv(debug_dir / "sd_route.csv")

    gnss = [
        {
            "lon": to_float(r, "longitude_deg"),
            "lat": to_float(r, "latitude_deg"),
            "yaw": to_float(r, "yaw_rad"),
            "frame": r.get("frame_id", ""),
        }
        for r in gnss_rows
    ]
    routes = []
    for r in route_rows:
        routes.append({
            "source": r.get("source", ""),
            "group": int(float(r.get("group_index", 0))),
            "idx": int(float(r.get("point_index", 0))),
            "lon": to_float(r, "longitude_deg"),
            "lat": to_float(r, "latitude_deg"),
        })

    all_points = [(p["lon"], p["lat"]) for p in gnss] + [(p["lon"], p["lat"]) for p in routes]
    min_lon = min(p[0] for p in all_points)
    max_lon = max(p[0] for p in all_points)
    min_lat = min(p[1] for p in all_points)
    max_lat = max(p[1] for p in all_points)

    data = {
        "gnss": gnss,
        "routes": routes,
        "bounds": {
            "minLon": min_lon,
            "maxLon": max_lon,
            "minLat": min_lat,
            "maxLat": max_lat,
        },
    }

    out = debug_dir / "global_debug.html"
    out.write_text(f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Global Route Debug</title>
  <style>
    body {{ margin: 0; font-family: sans-serif; background: #111; color: #ddd; }}
    #bar {{ padding: 10px 14px; background: #1b1e22; display: flex; gap: 18px; align-items: center; }}
    canvas {{ display: block; width: 100vw; height: calc(100vh - 44px); }}
    .gnss {{ color: #56ccf2; }}
    .link {{ color: #ff9628; }}
    .nav {{ color: #ff28a0; }}
  </style>
</head>
<body>
  <div id="bar">
    <b>Global Route Debug</b>
    <span class="gnss">GNSS trajectory: {len(gnss)}</span>
    <span class="link">SD links</span>
    <span class="nav">Navigation segments</span>
    <span>drag: pan, wheel: zoom, double click: fit</span>
  </div>
  <canvas id="c"></canvas>
  <script>
const data = {json.dumps(data)};
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
let view = {{ x: 0, y: 0, scale: 1, dragging: false, lastX: 0, lastY: 0 }};

function resize() {{
  const dpr = window.devicePixelRatio || 1;
  const r = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(r.width * dpr));
  canvas.height = Math.max(1, Math.floor(r.height * dpr));
}}

function merc(p) {{
  const lat = p.lat * Math.PI / 180;
  return {{ x: p.lon, y: Math.log(Math.tan(Math.PI / 4 + lat / 2)) * 180 / Math.PI }};
}}

function screen(p) {{
  const dpr = window.devicePixelRatio || 1;
  const m = merc(p);
  return [
    (m.x - view.x) * view.scale * dpr + canvas.width / 2,
    canvas.height / 2 - (m.y - view.y) * view.scale * dpr
  ];
}}

function fit() {{
  resize();
  const minP = merc({{ lon: data.bounds.minLon, lat: data.bounds.minLat }});
  const maxP = merc({{ lon: data.bounds.maxLon, lat: data.bounds.maxLat }});
  view.x = (minP.x + maxP.x) / 2;
  view.y = (minP.y + maxP.y) / 2;
  const sx = canvas.width / Math.max(1e-9, maxP.x - minP.x);
  const sy = canvas.height / Math.max(1e-9, maxP.y - minP.y);
  view.scale = Math.min(sx, sy) * 0.42;
  draw();
}}

function drawLine(points, color, width, dash) {{
  if (points.length < 2) return;
  ctx.beginPath();
  points.forEach((p, i) => {{
    const [x, y] = screen(p);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }});
  ctx.strokeStyle = color;
  ctx.lineWidth = width * (window.devicePixelRatio || 1);
  ctx.setLineDash(dash ? [8, 8] : []);
  ctx.stroke();
  ctx.setLineDash([]);
}}

function grouped(source) {{
  const m = new Map();
  for (const p of data.routes.filter(r => r.source === source)) {{
    const arr = m.get(p.group) || [];
    arr.push(p);
    m.set(p.group, arr);
  }}
  return [...m.values()].map(a => a.sort((a, b) => a.idx - b.idx));
}}

function drawYawArrows() {{
  const step = Math.max(1, Math.floor(data.gnss.length / 120));
  ctx.strokeStyle = '#ffffff';
  ctx.fillStyle = '#ffffff';
  ctx.lineWidth = 1.2 * (window.devicePixelRatio || 1);
  for (let i = 0; i < data.gnss.length; i += step) {{
    const p = data.gnss[i];
    const [x, y] = screen(p);
    const len = 16 * (window.devicePixelRatio || 1);
    const ex = x + Math.cos(p.yaw) * len;
    const ey = y - Math.sin(p.yaw) * len;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(ex, ey);
    ctx.stroke();
  }}
}}

function draw() {{
  resize();
  ctx.fillStyle = '#101214';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawLine(data.gnss, '#56ccf2', 2.0, false);
  for (const line of grouped('sd_link')) drawLine(line, '#ff9628', 2.4, false);
  for (const line of grouped('navigation_segment')) drawLine(line, '#ff28a0', 2.0, true);
  drawYawArrows();
}}

canvas.addEventListener('wheel', e => {{
  e.preventDefault();
  const k = Math.exp(-e.deltaY * 0.001);
  view.scale *= k;
  draw();
}}, {{ passive: false }});
canvas.addEventListener('pointerdown', e => {{ view.dragging = true; view.lastX = e.clientX; view.lastY = e.clientY; canvas.setPointerCapture(e.pointerId); }});
canvas.addEventListener('pointermove', e => {{
  if (!view.dragging) return;
  const dpr = window.devicePixelRatio || 1;
  view.x -= (e.clientX - view.lastX) * dpr / (view.scale * dpr);
  view.y += (e.clientY - view.lastY) * dpr / (view.scale * dpr);
  view.lastX = e.clientX; view.lastY = e.clientY;
  draw();
}});
canvas.addEventListener('pointerup', () => {{ view.dragging = false; }});
canvas.addEventListener('dblclick', fit);
window.addEventListener('resize', draw);
fit();
  </script>
</body>
</html>
""", encoding="utf-8")
    print(out)


if __name__ == "__main__":
    main()
