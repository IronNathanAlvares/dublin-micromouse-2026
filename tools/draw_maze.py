"""draw_maze.py - draws a maze file as a clean schematic PNG with the start,
the centre goal and the fastest route (same turn-aware cost as the solver).

    python tools/draw_maze.py mazes/dublin2026.txt            # writes mazes/dublin2026.png
    python tools/draw_maze.py mazes/dublin2026.txt --grid     # route without diagonals
    python tools/draw_maze.py mazes/dublin2026.txt --mark 13,6 --mark 13,4
        (--mark x,y circles cells you want to double-check on the real maze)
"""
import heapq
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_mms import (COST_CELL, COST_TURN90, COST_TURN180, DX, DY, N, E, S, W, best_diag_route,
                      in_goal, load_maze)

CELL, PAD = 44, 40


def fastest_route(walls):
    """Cells along the cheapest route from (0,0) facing north to the centre."""
    start = (0, 0, N)
    dist, prev = {start: 0}, {}
    pq = [(0, start)]
    while pq:
        c, s = heapq.heappop(pq)
        x, y, d = s
        if c > dist[s]:
            continue
        if in_goal(walls, x, y):
            route = [s]
            while route[-1] in prev:
                route.append(prev[route[-1]])
            cells = []
            for (cx, cy, _) in reversed(route):
                if not cells or cells[-1] != (cx, cy):
                    cells.append((cx, cy))
            return cells, c
        nxt = [((x, y, (d + 3) % 4), COST_TURN90), ((x, y, (d + 1) % 4), COST_TURN90),
               ((x, y, (d + 2) % 4), COST_TURN180)]
        if d not in walls[x][y]:
            nxt.append(((x + DX[d], y + DY[d], d), COST_CELL))
        for n, step in nxt:
            if c + step < dist.get(n, 1 << 30):
                dist[n], prev[n] = c + step, s
                heapq.heappush(pq, (c + step, n))
    return [], None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    marks = [tuple(map(int, v.split(","))) for f, v in zip(sys.argv, sys.argv[1:]) if f == "--mark"]
    path = args[0]
    walls = load_maze(path)
    w, h = len(walls), len(walls[0])
    img = Image.new("RGB", (w * CELL + 2 * PAD, h * CELL + 2 * PAD + 30), "white")
    g = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    def corner(x, y):  # grid corner (x, y) with y=0 at the bottom
        return PAD + x * CELL, PAD + (h - y) * CELL

    def centre(x, y):
        return PAD + x * CELL + CELL // 2, PAD + (h - 1 - y) * CELL + CELL // 2

    for x in range(w):
        for y in range(h):
            if in_goal(walls, x, y):
                g.rectangle([corner(x, y + 1), corner(x + 1, y)], fill="#ffe9a8")
    g.rectangle([corner(0, 1), corner(1, 0)], fill="#bfe8c4")

    cells, cost = fastest_route(walls)
    diagonal = "--grid" not in sys.argv
    if diagonal:
        dtime, points = best_diag_route(walls)
        g.line([(PAD + px * CELL / 2, PAD + (2 * h - py) * CELL / 2) for px, py in points],
               fill="#2f6fdf", width=5, joint="curve")
    elif cells:
        g.line([centre(*c) for c in cells], fill="#2f6fdf", width=5, joint="curve")

    for x in range(w):
        for y in range(h):
            if N in walls[x][y]: g.line([corner(x, y + 1), corner(x + 1, y + 1)], fill="black", width=4)
            if S in walls[x][y]: g.line([corner(x, y), corner(x + 1, y)], fill="black", width=4)
            if W in walls[x][y]: g.line([corner(x, y), corner(x, y + 1)], fill="black", width=4)
            if E in walls[x][y]: g.line([corner(x + 1, y), corner(x + 1, y + 1)], fill="black", width=4)
    for x in range(w + 1):
        for y in range(h + 1):
            cx, cy = corner(x, y)
            g.rectangle([cx - 2, cy - 2, cx + 2, cy + 2], fill="#e0622a")
    for (x, y) in marks:
        cx, cy = centre(x, y)
        g.ellipse([cx - 17, cy - 17, cx + 17, cy + 17], outline="#d4145a", width=3)

    g.text((centre(0, 0)[0] - 3, centre(0, 0)[1] - 5), "S", fill="black", font=font)
    gx = [x for x in range(w) if in_goal(walls, x, h // 2)]
    gy = [y for y in range(h) if in_goal(walls, w // 2, y)]
    mx = (centre(min(gx), 0)[0] + centre(max(gx), 0)[0]) // 2
    my = (centre(0, min(gy))[1] + centre(0, max(gy))[1]) // 2
    g.text((mx - 3, my - 5), "G", fill="black", font=font)
    for i in range(w):
        g.text((centre(i, 0)[0] - 3, PAD + h * CELL + 6), str(i), fill="gray", font=font)
    for j in range(h):
        g.text((PAD - 18, centre(0, j)[1] - 5), str(j), fill="gray", font=font)
    turns = sum(1 for a, b, c in zip(cells, cells[1:], cells[2:])
                if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]))
    if diagonal:
        label = f"blue = fastest route with diagonals: mms time {dtime:.0f}"
        out = os.path.splitext(path)[0] + "_diagonal.png"
    else:
        label = f"blue = fastest route on the grid: {len(cells) - 1} cells, {turns} turns"
        out = os.path.splitext(path)[0] + ".png"
    g.text((PAD, h * CELL + 2 * PAD + 8),
           f"{os.path.basename(path)}  S = start (facing north)  G = centre goal  {label}",
           fill="black", font=font)
    img.save(out)
    print(f"wrote {out}: {label}")


if __name__ == "__main__":
    main()
