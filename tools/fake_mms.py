"""fake_mms.py - a headless stand-in for the mms simulator, for quick testing.

Runs sim/mouse.exe against random mazes or real maze files, speaking the mms
protocol, and checks the mouse never crashes, reaches the centre, and that the
speed run it plans is as fast as the best possible route.

    python tools/fake_mms.py            # 100 random 16x16 mazes
    python tools/fake_mms.py 20 8       # 20 random 8x8 mazes
    python tools/fake_mms.py 1 16 -v    # one maze, print it and the mouse's log
    python tools/fake_mms.py --files tools/mazefiles/classic   # every maze file in a folder
"""
import heapq
import os
import random
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
MOUSE = os.path.join(HERE, "..", "sim", "mouse.exe" if os.name == "nt" else "mouse")

N, E, S, W = 0, 1, 2, 3
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
COST_CELL, COST_TURN90, COST_TURN180 = 2, 3, 5  # must match mm::Costs


def centre(size):
    return {size // 2} if size % 2 else {size // 2 - 1, size // 2}


def make_maze(size, rng, loopiness=0.08):
    """walls[x][y] = set of blocked directions."""
    walls = [[{N, E, S, W} for _ in range(size)] for _ in range(size)]

    def knock(x, y, d):
        walls[x][y].discard(d)
        walls[x + DX[d]][y + DY[d]].discard((d + 2) % 4)

    # Recursive backtracker = a "perfect" maze, then add loops like real mazes.
    seen = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        x, y = stack[-1]
        options = [d for d in range(4)
                   if 0 <= x + DX[d] < size and 0 <= y + DY[d] < size
                   and (x + DX[d], y + DY[d]) not in seen]
        if not options:
            stack.pop()
            continue
        d = rng.choice(options)
        knock(x, y, d)
        seen.add((x + DX[d], y + DY[d]))
        stack.append((x + DX[d], y + DY[d]))
    for x in range(size):
        for y in range(size):
            for d in (N, E):
                if d in walls[x][y] and 0 <= x + DX[d] < size and 0 <= y + DY[d] < size:
                    if rng.random() < loopiness:
                        knock(x, y, d)
    cx, cy = sorted(centre(size)), sorted(centre(size))
    for x in cx:
        for y in cy:
            if x + 1 in cx:
                knock(x, y, E)
            if y + 1 in cy:
                knock(x, y, N)
    return walls


def load_maze(path):
    """Reads the mms / micromouseonline text format ("o---o" and "|   |" rows)."""
    lines = [l.rstrip("\n") for l in open(path, encoding="utf-8", errors="replace") if l.strip()]
    height = (len(lines) - 1) // 2
    width = (len(lines[0].rstrip()) - 1) // 4
    walls = [[set() for _ in range(height)] for _ in range(width)]
    for y in range(height):
        top, mid, bottom = (lines[(height - 1 - y) * 2 + i].ljust(4 * width + 1) for i in range(3))
        for x in range(width):
            if top[4 * x + 1] == "-": walls[x][y].add(N)
            if bottom[4 * x + 1] == "-": walls[x][y].add(S)
            if mid[4 * x] == "|": walls[x][y].add(W)
            if mid[4 * x + 4] == "|": walls[x][y].add(E)
    return walls


def in_goal(walls, x, y):
    return x in centre(len(walls)) and y in centre(len(walls[0]))


def best_cost(walls):
    """Same cost model as Maze::planPath, but on the fully known maze."""
    start = (0, 0, N)
    dist = {start: 0}
    pq = [(0, start)]
    while pq:
        c, (x, y, d) = heapq.heappop(pq)
        if c > dist[(x, y, d)]:
            continue
        if in_goal(walls, x, y):
            return c
        nxt = [((x, y, (d + 3) % 4), COST_TURN90), ((x, y, (d + 1) % 4), COST_TURN90),
               ((x, y, (d + 2) % 4), COST_TURN180)]
        if d not in walls[x][y]:
            nxt.append(((x + DX[d], y + DY[d], d), COST_CELL))
        for s, step in nxt:
            if c + step < dist.get(s, 1 << 30):
                dist[s] = c + step
                heapq.heappush(pq, (c + step, s))
    return None


def draw(walls):
    width, height = len(walls), len(walls[0])
    rows = []
    for y in range(height - 1, -1, -1):
        rows.append("+" + "".join("---+" if N in walls[x][y] else "   +" for x in range(width)))
        rows.append("".join(("|" if W in walls[x][y] else " ") + "   " for x in range(width)) + "|")
    rows.append("+" + "---+" * width)
    return "\n".join(rows)


# ---- mms geometry: half-cell "semi-positions" and 8 headings (clockwise from N) ----
DX8 = [0, 1, 1, 1, 0, -1, -1, -1]
DY8 = [1, 1, 0, -1, -1, -1, 0, 1]
T_HALF, T_DIAG, T_TURN45 = 50.0, 70.71, 16.66  # mms animation time per motion


def mms_blocked(walls, sx, sy, h):
    """Written straight from mms Window::isWall(semiPos, semiDir)."""
    w, hgt = len(walls), len(walls[0])
    cx, cy = sx // 2, sy // 2
    if sx % 2 == 1 and sy % 2 == 1:
        return h % 2 == 1 or (h // 2) in walls[cx][cy]
    if sx % 2 == 0 and sy % 2 == 1:
        if h in (0, 4): return True
        if h in (2, 6): return False
        if h in (1, 3) and sx == 2 * w: return False
        if h in (7, 5) and sx == 0: return False
        return {1: N in walls[cx][cy], 3: S in walls[cx][cy],
                7: N in walls[cx - 1][cy], 5: S in walls[cx - 1][cy]}[h]
    if sx % 2 == 1 and sy % 2 == 0:
        if h in (2, 6): return True
        if h in (0, 4): return False
        if h in (1, 7) and sy == 2 * hgt: return False
        if h in (3, 5) and sy == 0: return False
        return {1: E in walls[cx][cy], 7: W in walls[cx][cy],
                3: E in walls[cx][cy - 1], 5: W in walls[cx][cy - 1]}[h]
    return True


def best_diag_time(walls):
    """Fastest mms time from the start to the middle of a centre cell, with diagonals."""
    return best_diag_route(walls)[0]


def best_diag_route(walls):
    """(time, [semi-positions along the route]) for the fastest diagonal run."""
    start = (1, 1, 0)
    dist, prev = {start: 0.0}, {}
    pq = [(0.0, start)]
    while pq:
        c, s = heapq.heappop(pq)
        sx, sy, h = s
        if c > dist[s] + 1e-9:
            continue
        if sx % 2 == 1 and sy % 2 == 1 and h % 2 == 0 and in_goal(walls, sx // 2, sy // 2):
            route = [s]
            while route[-1] in prev:
                route.append(prev[route[-1]])
            points = []
            for (px, py, _) in reversed(route):
                if not points or points[-1] != (px, py):
                    points.append((px, py))
            return c, points
        nxt = [((sx, sy, (h + 1) % 8), T_TURN45), ((sx, sy, (h + 7) % 8), T_TURN45)]
        if not mms_blocked(walls, sx, sy, h):
            nxt.append(((sx + DX8[h], sy + DY8[h], h), T_DIAG if h % 2 else T_HALF))
        for n, step in nxt:
            if c + step < dist.get(n, 1e18) - 1e-9:
                dist[n], prev[n] = c + step, s
                heapq.heappush(pq, (c + step, n))
    return None, []


def run(walls, verbose):
    sx, sy, h = 1, 1, 0  # centre of (0,0), facing north
    half_steps = 0
    goal_entries = 0
    args = [os.environ.get("ROUNDS", "4"), "nodiag" if os.environ.get("NODIAG") else "diag"]
    if os.environ.get("TURN_PENALTY"):
        args.append(os.environ["TURN_PENALTY"])
    with tempfile.TemporaryFile("w+") as err:
        p = subprocess.Popen([MOUSE] + args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=err, text=True, bufsize=1)

        def reply(s):
            p.stdin.write(s + "\n")
            p.stdin.flush()

        def cell_wall(rel):  # only asked at a cell centre facing along the grid
            return str(((h // 2) + rel) % 4 in walls[sx // 2][sy // 2]).lower()

        crashed = False
        for line in p.stdout:
            parts = line.split()
            if not parts:
                continue
            cmd = parts[0]
            if cmd == "mazeWidth":
                reply(str(len(walls)))
            elif cmd == "mazeHeight":
                reply(str(len(walls[0])))
            elif cmd in ("wallFront", "wallRight", "wallBack", "wallLeft"):
                reply(cell_wall(["wallFront", "wallRight", "wallBack", "wallLeft"].index(cmd)))
            elif cmd in ("turnLeft", "turnLeft90", "turnRight", "turnRight90", "turnLeft45", "turnRight45"):
                h = (h + {"turnLeft": 6, "turnLeft90": 6, "turnRight": 2, "turnRight90": 2,
                          "turnLeft45": 7, "turnRight45": 1}[cmd]) % 8
                reply("ack")
            elif cmd in ("moveForward", "moveForwardHalf"):
                n = int(parts[1]) if len(parts) > 1 else 1
                if cmd == "moveForward":
                    n *= 2
                for _ in range(n):
                    if mms_blocked(walls, sx, sy, h):
                        crashed = True
                        break
                    was_in_goal = in_goal(walls, sx // 2, sy // 2)
                    sx, sy = sx + DX8[h], sy + DY8[h]
                    half_steps += 1
                    if in_goal(walls, sx // 2, sy // 2) and not was_in_goal:
                        goal_entries += 1
                reply("crash" if crashed else "ack")
                if crashed:
                    break
            elif cmd == "wasReset":
                reply("false")
            elif cmd == "ackReset":
                reply("ack")
            # setWall / setColor / setText / clear* need no reply
        p.stdin.close()
        p.wait(timeout=30)
        err.seek(0)
        log = err.read()
    if verbose:
        print(log)
    costs = [int(c) for c in re.findall(r"Speed run \d+: \d+ moves, cost (\d+)", log)]
    diag_times = [int(t) for t in re.findall(r"Speed run \d+.*mms time (\d+)", log)]  # all speed runs
    return crashed, p.returncode, half_steps // 2, goal_entries, costs, diag_times, log


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    if not os.path.exists(MOUSE):
        sys.exit(f"Build the simulator mouse first: {MOUSE} not found (see README)")

    if "--files" in sys.argv:
        paths = []
        for a in args:
            paths += sorted(os.path.join(a, f) for f in os.listdir(a)) if os.path.isdir(a) else [a]
        mazes = [(os.path.basename(p), load_maze(p)) for p in paths if p.endswith(".txt")]
        mazes = [(n, w) for n, w in mazes if len(w) <= 16 and len(w[0]) <= 16]  # solver max
    else:
        count = int(args[0]) if args else 100
        size = int(args[1]) if len(args) > 1 else 16
        mazes = [(f"random-{size}x{size}-{seed}", make_maze(size, random.Random(seed)))
                 for seed in range(count)]

    failures = optimal = skipped = 0
    total_cells = 0
    diagonal = not os.environ.get("NODIAG")
    for name, walls in mazes:
        best = best_cost(walls)
        if best is None:
            skipped += 1  # no route to the centre in this file
            continue
        if verbose:
            print(draw(walls))
        crashed, code, cells, entries, costs, diag_times, log = run(walls, verbose)
        runs = len(costs) + len(diag_times)
        ok = not crashed and code == 0 and runs >= 1 and entries >= 2
        if diagonal:
            fastest = best_diag_time(walls)
            first = diag_times[0] if diag_times else None
            if first is not None and first < fastest - 1:
                ok = False  # faster than physically possible = the map is wrong
        elif costs and costs[0] < best:
            ok = False
        if not ok:
            failures += 1
            print(f"{name}: FAIL crashed={crashed} exit={code} goal_entries={entries} "
                  f"costs={costs} diag={diag_times} best={best}\n{log}")
            continue
        total_cells += cells
        is_optimal = (abs(first - fastest) <= 1) if diagonal and first is not None else (costs and costs[0] == best)
        if is_optimal:
            optimal += 1
        elif verbose or "--files" in sys.argv:
            print(f"{name}: first speed run {diag_times or costs}, best possible "
                  f"{round(fastest) if diagonal else best}")
    tested = len(mazes) - skipped
    passed = tested - failures
    print(f"{passed}/{tested} mazes passed; speed run optimal in {optimal}/{passed}"
          f"{' (diagonals, mms time)' if diagonal else ''}; "
          f"avg cells driven {total_cells / max(passed, 1):.0f}"
          + (f"; {skipped} skipped (no route to centre)" if skipped else ""))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
