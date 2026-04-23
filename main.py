import pygame
import heapq
import random

pygame.init()

WIDTH = 600
HEIGHT = 600
ROWS = 20
CELL_SIZE = WIDTH // ROWS

# Colors
WHITE = (255, 255, 255)
GRAY = (200, 200, 200)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)      # Start
RED = (255, 0, 0)        # End
BLUE = (0, 0, 255)       # Walls
YELLOW = (255, 255, 0)   # Path
ORANGE = (255, 165, 0)   # Fire
LIGHT_RED = (255, 200, 200)  # Heat zone (smoke)

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Realistic Evacuation AI")

clock = pygame.time.Clock()

grid = [[0 for _ in range(ROWS)] for _ in range(ROWS)]

start = None
end = None
mode = "wall"

# ---------------- DRAW ----------------
def draw_grid():
    for r in range(ROWS):
        for c in range(ROWS):
            rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)

            if grid[r][c] == 1:
                pygame.draw.rect(screen, BLUE, rect)
            elif grid[r][c] == 2:
                pygame.draw.rect(screen, GREEN, rect)
            elif grid[r][c] == 3:
                pygame.draw.rect(screen, RED, rect)
            elif grid[r][c] == 4:
                pygame.draw.rect(screen, YELLOW, rect)
            elif grid[r][c] == 5:
                pygame.draw.rect(screen, ORANGE, rect)
            elif grid[r][c] == 6:
                pygame.draw.rect(screen, LIGHT_RED, rect)
            else:
                pygame.draw.rect(screen, WHITE, rect)

            pygame.draw.rect(screen, GRAY, rect, 1)

# ---------------- CLICK ----------------
def get_clicked_pos(pos):
    x, y = pos
    return y // CELL_SIZE, x // CELL_SIZE

# ---------------- HEURISTIC ----------------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ---------------- RISK ----------------
def get_risk(grid, r, c):
    risk = 0

    # Immediate fire = very high danger
    if grid[r][c] == 5:
        return 100

    # Nearby fire (strong)
    for i in range(-1, 2):
        for j in range(-1, 2):
            nr, nc = r + i, c + j
            if 0 <= nr < ROWS and 0 <= nc < ROWS:
                if grid[nr][nc] == 5:
                    risk += 8

    # Heat/smoke zone
    if grid[r][c] == 6:
        risk += 5

    return risk

# ---------------- A* ----------------
def astar(grid, start, end):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        r, c = current
        neighbors = [(r+1,c),(r-1,c),(r,c+1),(r,c-1)]

        for n in neighbors:
            nr, nc = n
            if 0 <= nr < ROWS and 0 <= nc < ROWS:

                if grid[nr][nc] == 1:  # wall
                    continue

                temp_g = g_score[current] + 1

                if n not in g_score or temp_g < g_score[n]:
                    g_score[n] = temp_g
                    risk = get_risk(grid, nr, nc)

                    f = temp_g + heuristic(n, end) + 0.8 * risk

                    heapq.heappush(open_set, (f, n))
                    came_from[n] = current

    return []

# ---------------- FIRE + HEAT ----------------
def spread_fire(grid):
    new_fire = []
    new_heat = []

    for r in range(ROWS):
        for c in range(ROWS):
            if grid[r][c] == 5:
                neighbors = [(r+1,c),(r-1,c),(r,c+1),(r,c-1)]

                for nr, nc in neighbors:
                    if 0 <= nr < ROWS and 0 <= nc < ROWS:
                        # 🔥 Fire spreads through empty + path + heat
                        if grid[nr][nc] in [0, 4, 6]:
                            if random.random() < 0.06:
                                new_fire.append((nr, nc))

                        # 🌫️ Heat spreads further
                        if grid[nr][nc] == 0:
                            if random.random() < 0.15:
                                new_heat.append((nr, nc))

    for r, c in new_fire:
        if (r, c) != start and (r, c) != end:
            grid[r][c] = 5

    for r, c in new_heat:
        if grid[r][c] == 0:
            grid[r][c] = 6

# ---------------- MAIN ----------------
running = True
frame = 0

while running:
    clock.tick(10)
    frame += 1

    if frame % 3 == 0:
        spread_fire(grid)

    # Dynamic path
    if start and end:
        path = astar(grid, start, end)

        # Clear old path
        for r in range(ROWS):
            for c in range(ROWS):
                if grid[r][c] == 4:
                    grid[r][c] = 0

        # Draw new path
        for r, c in path:
            if (r, c) != start and (r, c) != end:
                grid[r][c] = 4

    screen.fill(BLACK)
    draw_grid()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Mode switching
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                mode = "wall"
            elif event.key == pygame.K_s:
                mode = "start"
            elif event.key == pygame.K_e:
                mode = "end"
            elif event.key == pygame.K_f:
                mode = "fire"

        # Mouse
        if event.type == pygame.MOUSEBUTTONDOWN:
            r, c = get_clicked_pos(pygame.mouse.get_pos())

            if event.button == 1:
                if mode == "wall":
                    grid[r][c] = 1

                elif mode == "start":
                    if start:
                        grid[start[0]][start[1]] = 0
                    start = (r, c)
                    grid[r][c] = 2

                elif mode == "end":
                    if end:
                        grid[end[0]][end[1]] = 0
                    end = (r, c)
                    grid[r][c] = 3

                elif mode == "fire":
                    grid[r][c] = 5

            elif event.button == 3:
                grid[r][c] = 0
                if (r, c) == start:
                    start = None
                elif (r, c) == end:
                    end = None

    pygame.display.update()

pygame.quit()