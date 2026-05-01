"""
Realistic Evacuation AI
========================
Controls:
  W  — wall mode       (left click to draw, right click to erase)
  S  — set start
  E  — set end (exit)
  F  — place fire
  C  — clear entire grid
  R  — reset path only
  SPACE — pause / resume fire spread

Algorithms used:
  • A* with Manhattan heuristic
  • Risk-weighted cost function (avoids fire & heat zones)
  • Dynamic fire spreading (probabilistic cellular automaton)
  • Heat/smoke propagation
"""

import pygame
import heapq
import random

# ─────────────────────────── CONFIG ───────────────────────────
WIDTH       = 700
HEIGHT      = 740          # extra 40px for HUD
ROWS        = 25
CELL        = WIDTH // ROWS

FPS         = 12           # simulation speed
FIRE_EVERY  = 3            # spread fire every N frames
FIRE_PROB   = 0.07         # probability fire jumps to neighbour
HEAT_PROB   = 0.18         # probability heat spreads further
RISK_WEIGHT = 0.9          # how much A* avoids danger (0 = ignores fire)

# Cell types
EMPTY  = 0
WALL   = 1
START  = 2
END    = 3
PATH   = 4
FIRE   = 5
HEAT   = 6

# Colours
C_WHITE      = (245, 245, 245)
C_GRAY       = (190, 190, 190)
C_BLACK      = (20,  20,  20)
C_WALL       = (50,  50,  60)
C_START      = (30,  200, 80)
C_END        = (220, 50,  50)
C_PATH       = (80,  160, 255)
C_PATH_DARK  = (40,  100, 200)
C_FIRE       = (255, 100, 0)
C_FIRE2      = (255, 50,  0)
C_HEAT       = (255, 210, 160)
C_HUD        = (28,  28,  36)
C_TEXT       = (220, 220, 230)
C_ACCENT     = (80,  160, 255)
C_WARN       = (255, 160, 40)
C_DANGER     = (255, 70,  70)

MODE_COLORS = {
    "wall":  C_WALL,
    "start": C_START,
    "end":   C_END,
    "fire":  C_FIRE,
}

# ─────────────────────────── INIT ─────────────────────────────
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Evacuation AI  —  A* + Risk Heuristic")
clock  = pygame.time.Clock()
font_sm = pygame.font.SysFont("Segoe UI", 13)
font_md = pygame.font.SysFont("Segoe UI", 15, bold=True)
font_lg = pygame.font.SysFont("Segoe UI", 18, bold=True)

# ─────────────────────────── STATE ────────────────────────────
grid   = [[EMPTY] * ROWS for _ in range(ROWS)]
start  = None
end    = None
mode   = "wall"
paused = False
frame  = 0
path   = []
status_msg = "Place START (S), END (E), walls (W), fire (F) then watch the AI escape."

# ─────────────────────────── HELPERS ──────────────────────────
def cell_rect(r, c):
    return pygame.Rect(c * CELL, r * CELL, CELL, CELL)

def get_pos(mouse_pos):
    x, y = mouse_pos
    if y >= WIDTH:
        return None, None
    return y // CELL, x // CELL

def set_cell(r, c, val):
    global start, end
    if not (0 <= r < ROWS and 0 <= c < ROWS):
        return
    if val == START:
        if start:
            grid[start[0]][start[1]] = EMPTY
        start = (r, c)
    elif val == END:
        if end:
            grid[end[0]][end[1]] = EMPTY
        end = (r, c)
    grid[r][c] = val

def clear_grid():
    global grid, start, end, path
    grid  = [[EMPTY] * ROWS for _ in range(ROWS)]
    start = None
    end   = None
    path  = []

def clear_path():
    for r in range(ROWS):
        for c in range(ROWS):
            if grid[r][c] == PATH:
                grid[r][c] = EMPTY

# ─────────────────────────── HEURISTIC ────────────────────────
def heuristic(a, b):
    """Manhattan distance."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ─────────────────────────── RISK MAP ─────────────────────────
def get_risk(r, c):
    """
    Returns a danger score for cell (r,c).
    A* adds this to the cost so the path avoids fire & heat.
    """
    if grid[r][c] == FIRE:
        return 120          # walking into fire = very bad

    risk = 0
    # Adjacent fire cells
    for dr in range(-2, 3):
        for dc in range(-2, 3):
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS and 0 <= nc < ROWS:
                if grid[nr][nc] == FIRE:
                    dist = abs(dr) + abs(dc)
                    risk += max(0, 10 - dist * 3)   # closer = more risk

    # Heat zone penalty
    if grid[r][c] == HEAT:
        risk += 6

    return risk

# ─────────────────────────── A* ───────────────────────────────
def astar(start_node, end_node):
    """
    A* search.
    Cost = g (steps taken) + h (Manhattan to goal) + risk * RISK_WEIGHT
    Returns list of (r,c) tuples from start→end, or [].
    """
    if not start_node or not end_node:
        return []

    open_set  = []
    came_from = {}
    g_score   = {start_node: 0}

    h0 = heuristic(start_node, end_node)
    heapq.heappush(open_set, (h0, start_node))

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end_node:
            # Reconstruct path
            path_out = []
            while current in came_from:
                path_out.append(current)
                current = came_from[current]
            path_out.append(start_node)
            path_out.reverse()
            return path_out

        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < ROWS and 0 <= nc < ROWS):
                continue
            if grid[nr][nc] == WALL:
                continue

            step_cost = 1 + get_risk(nr, nc) * RISK_WEIGHT
            new_g = g_score[current] + step_cost
            nb = (nr, nc)

            if nb not in g_score or new_g < g_score[nb]:
                g_score[nb]   = new_g
                f              = new_g + heuristic(nb, end_node)
                came_from[nb] = current
                heapq.heappush(open_set, (f, nb))

    return []   # no path found

# ─────────────────────────── FIRE ─────────────────────────────
def spread_fire():
    """Probabilistic fire & heat spread (cellular automaton)."""
    new_fire = []
    new_heat = []

    for r in range(ROWS):
        for c in range(ROWS):
            if grid[r][c] == FIRE:
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nr, nc = r + dr, c + dc
                    if not (0 <= nr < ROWS and 0 <= nc < ROWS):
                        continue
                    cell = grid[nr][nc]
                    if cell in (EMPTY, PATH, HEAT) and random.random() < FIRE_PROB:
                        new_fire.append((nr, nc))
                    elif cell == EMPTY and random.random() < HEAT_PROB:
                        new_heat.append((nr, nc))

    for r, c in new_fire:
        if (r, c) != start and (r, c) != end:
            grid[r][c] = FIRE
    for r, c in new_heat:
        if grid[r][c] == EMPTY:
            grid[r][c] = HEAT

# ─────────────────────────── DRAW ─────────────────────────────
def lerp_color(c1, c2, t):
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))

def draw_grid():
    for r in range(ROWS):
        for c in range(ROWS):
            rect = cell_rect(r, c)
            val  = grid[r][c]

            if   val == WALL:  color = C_WALL
            elif val == START: color = C_START
            elif val == END:   color = C_END
            elif val == FIRE:
                t = (frame % 6) / 6
                color = lerp_color(C_FIRE, C_FIRE2, t)
            elif val == HEAT:  color = C_HEAT
            elif val == PATH:
                # gradient: lighter near end
                color = C_PATH
            else:              color = C_WHITE

            pygame.draw.rect(screen, color, rect)

            # Grid lines
            pygame.draw.rect(screen, C_GRAY, rect, 1)

    # Draw path as filled circles for clarity
    for i, (r, c) in enumerate(path):
        if grid[r][c] not in (START, END):
            cx = c * CELL + CELL // 2
            cy = r * CELL + CELL // 2
            t  = i / max(len(path), 1)
            col = lerp_color(C_PATH, C_PATH_DARK, t)
            pygame.draw.circle(screen, col, (cx, cy), CELL // 4)

    # Start / End labels
    if start:
        r, c = start
        cx, cy = c * CELL + CELL // 2, r * CELL + CELL // 2
        lbl = font_sm.render("S", True, C_BLACK)
        screen.blit(lbl, lbl.get_rect(center=(cx, cy)))
    if end:
        r, c = end
        cx, cy = c * CELL + CELL // 2, r * CELL + CELL // 2
        lbl = font_sm.render("EXIT", True, C_WHITE)
        screen.blit(lbl, lbl.get_rect(center=(cx, cy)))

def draw_hud():
    pygame.draw.rect(screen, C_HUD, (0, WIDTH, WIDTH, HEIGHT - WIDTH))

    # Mode indicator
    mode_col = MODE_COLORS.get(mode, C_TEXT)
    mode_lbl = font_md.render(f"Mode: {mode.upper()}", True, mode_col)
    screen.blit(mode_lbl, (10, WIDTH + 8))

    # Pause indicator
    if paused:
        p_lbl = font_md.render("PAUSED", True, C_WARN)
        screen.blit(p_lbl, (WIDTH - 90, WIDTH + 8))

    # Path length
    path_info = f"Path: {max(0, len(path)-1)} steps" if path else "Path: —"
    p_lbl = font_sm.render(path_info, True, C_TEXT)
    screen.blit(p_lbl, (200, WIDTH + 8))

    # Fire on path warning
    on_fire = any(grid[r][c] == FIRE for r, c in path) if path else False
    if on_fire:
        w_lbl = font_md.render("WARNING: PATH ON FIRE — rerouting!", True, C_DANGER)
        screen.blit(w_lbl, (10, WIDTH + 24))
    else:
        # Controls reminder
        hint = "W wall | S start | E exit | F fire | C clear | SPACE pause"
        h_lbl = font_sm.render(hint, True, (130, 130, 150))
        screen.blit(h_lbl, (10, WIDTH + 24))

# ─────────────────────────── MAIN LOOP ────────────────────────
running = True
mouse_held = False

while running:
    clock.tick(FPS)
    frame += 1

    # Fire spreads every N frames
    if not paused and frame % FIRE_EVERY == 0:
        spread_fire()

    # Recompute A* every frame (dynamic replanning)
    if start and end:
        clear_path()
        path = astar(start, end)
        for r, c in path:
            if grid[r][c] == EMPTY or grid[r][c] == HEAT:
                grid[r][c] = PATH

    # ── Draw ──
    screen.fill(C_BLACK)
    draw_grid()
    draw_hud()

    # ── Events ──
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if   event.key == pygame.K_w:      mode = "wall"
            elif event.key == pygame.K_s:      mode = "start"
            elif event.key == pygame.K_e:      mode = "end"
            elif event.key == pygame.K_f:      mode = "fire"
            elif event.key == pygame.K_c:      clear_grid()
            elif event.key == pygame.K_r:      clear_path(); path = []
            elif event.key == pygame.K_SPACE:  paused = not paused
            elif event.key == pygame.K_ESCAPE: running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_held = True
            r, c = get_pos(pygame.mouse.get_pos())
            if r is None:
                continue

            if event.button == 1:
                if   mode == "wall":  set_cell(r, c, WALL)
                elif mode == "start": set_cell(r, c, START)
                elif mode == "end":   set_cell(r, c, END)
                elif mode == "fire":  set_cell(r, c, FIRE)

            elif event.button == 3:   # right click = erase
                if (r, c) == start:   start = None
                elif (r, c) == end:   end   = None
                grid[r][c] = EMPTY

        elif event.type == pygame.MOUSEBUTTONUP:
            mouse_held = False

        elif event.type == pygame.MOUSEMOTION and mouse_held:
            r, c = get_pos(pygame.mouse.get_pos())
            if r is None:
                continue
            if mode == "wall":
                set_cell(r, c, WALL)
            elif mode == "fire":
                set_cell(r, c, FIRE)

    pygame.display.flip()

pygame.quit()
