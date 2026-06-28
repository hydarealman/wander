"""Generate a 4K pixel-art sunset landscape for the wander blog."""
import math, random
from PIL import Image

# 4K pixel-art sunset landscape
W, H = 1920, 1080
SCALE = 4  # pixel size for pixel-art look
PW, PH = W // SCALE, H // SCALE  # pixel grid: 480 x 270

img = Image.new("RGB", (W, H))
pixels = img.load()

# Sunset sky palette (top to horizon)
sky_palette = [
    (10, 5, 25),     # top - deep indigo
    (15, 8, 40),
    (25, 12, 55),
    (40, 15, 60),
    (60, 20, 60),
    (80, 25, 50),
    (100, 30, 40),
    (130, 35, 30),
    (160, 45, 25),
    (190, 60, 20),
    (210, 80, 20),
    (230, 100, 20),
    (245, 130, 20),
    (250, 160, 20),
    (255, 180, 30),
    (255, 200, 50),
]

# Mountain silhouette colors
dark_colors = [
    (25, 12, 30),
    (30, 15, 35),
    (35, 18, 40),
    (40, 20, 45),
    (50, 25, 50),
]

# Sun colors (center to edge)
sun_colors = [
    (255, 240, 200),
    (255, 220, 120),
    (255, 200, 60),
    (255, 170, 30),
    (240, 140, 20),
]


def draw_pixel_block(px, py, color, w=1, h=1):
    for dy in range(h * SCALE):
        for dx in range(w * SCALE):
            x, y = px * SCALE + dx, py * SCALE + dy
            if 0 <= x < W and 0 <= y < H:
                pixels[x, y] = color


def lerp_color(c1, c2, t):
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


# --- Mountain generation ---
random.seed(42)
mountain_heights = []
for x in range(PW):
    h = (math.sin(x * 0.008) * 50
         + math.sin(x * 0.023 + 1.5) * 35
         + math.sin(x * 0.05 + 3) * 20
         + math.cos(x * 0.012 + 2) * 30
         + random.randint(-3, 3))
    mountain_heights.append(int(max(20, min(100, h + 70))))

random.seed(123)
mountain2_heights = []
for x in range(PW):
    h = (math.sin(x * 0.012 + 1) * 40
         + math.sin(x * 0.028 + 0.5) * 25
         + math.cos(x * 0.015 + 2) * 30
         + random.randint(-2, 2))
    mountain2_heights.append(int(max(10, min(80, h + 55))))

# --- Sky gradient with pixel-art dithering ---
for py in range(PH):
    t = py / PH
    palette_t = t * (len(sky_palette) - 1)
    idx = int(palette_t)
    frac = palette_t - idx
    if idx >= len(sky_palette) - 1:
        color = sky_palette[-1]
    else:
        color = lerp_color(sky_palette[idx], sky_palette[idx + 1], frac)

    for px in range(PW):
        if random.random() < 0.02:
            r = max(0, min(255, color[0] + random.randint(-8, 8)))
            g = max(0, min(255, color[1] + random.randint(-8, 8)))
            b = max(0, min(255, color[2] + random.randint(-8, 8)))
            draw_pixel_block(px, py, (r, g, b))
        else:
            draw_pixel_block(px, py, color)

# --- Sun ---
sun_cx, sun_cy = PW // 2, 145
sun_radius = 22

for dy in range(-sun_radius, sun_radius + 1):
    for dx in range(-sun_radius, sun_radius + 1):
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= sun_radius:
            px, py = sun_cx + dx, sun_cy + dy
            if 0 <= px < PW and 0 <= py < PH:
                t = dist / sun_radius
                if t < 0.3:
                    color = sun_colors[0]
                elif t < 0.5:
                    color = sun_colors[1]
                elif t < 0.7:
                    color = sun_colors[2]
                elif t < 0.85:
                    color = sun_colors[3]
                else:
                    color = sun_colors[4]
                draw_pixel_block(px, py, color)

# --- Sun rays (horizontal streaks) ---
random.seed(77)
for i in range(12):
    ray_y = sun_cy + random.randint(-10, 10)
    ray_len = random.randint(15, 45)
    ray_x_start = random.choice([sun_cx - sun_radius - 4, sun_cx + sun_radius + 1])
    ray_dir = 1 if ray_x_start < sun_cx else -1
    for step in range(ray_len):
        px = ray_x_start + step * ray_dir
        py = ray_y + random.randint(-1, 1)
        if random.random() < 0.6 and 0 <= px < PW and 0 <= py < PH:
            r = 255
            g = 180 + random.randint(0, 50)
            b = random.randint(5, 30)
            draw_pixel_block(px, py, (r, g, b))

# --- Clouds ---
clouds = [
    (80, 40, 25, 4), (140, 55, 30, 3), (200, 35, 28, 5),
    (300, 50, 22, 3), (360, 42, 20, 4), (100, 70, 18, 3),
    (250, 65, 24, 4), (330, 60, 16, 3), (150, 85, 20, 3),
    (400, 45, 22, 4), (50, 60, 20, 3), (420, 55, 18, 4),
]

random.seed(55)
for cx, cy, cw, ch in clouds:
    for dy in range(ch):
        for dx in range(cw):
            px, py = cx + dx, cy + dy
            if 0 <= px < PW and 0 <= py < PH:
                edge_dist = min(dx, cw - dx - 1) / (cw / 2) if cw > 1 else 0
                vert_dist = abs(dy - ch // 2) / (ch / 2) if ch > 1 else 0
                if random.random() < 0.65 - 0.3 * edge_dist - 0.2 * vert_dist:
                    r = 200 + random.randint(0, 55)
                    g = 120 + random.randint(0, 60)
                    b = 60 + random.randint(0, 40)
                    draw_pixel_block(px, py, (r, g, b))
                elif random.random() < 0.2:
                    r = 150 + random.randint(0, 40)
                    g = 60 + random.randint(0, 30)
                    b = 30 + random.randint(0, 30)
                    draw_pixel_block(px, py, (r, g, b))

# --- Mountain back layer ---
random.seed(42)
for px in range(PW):
    mh = mountain_heights[px]
    for py in range(PH - mh, PH):
        if 0 <= py < PH:
            if random.random() < 0.1:
                c_idx = random.randint(2, 4)
            else:
                c_idx = 1
            draw_pixel_block(px, py, dark_colors[min(c_idx, len(dark_colors) - 1)])

# --- Mountain front layer (darker) ---
random.seed(123)
for px in range(PW):
    mh = mountain2_heights[px]
    for py in range(PH - mh, PH):
        if 0 <= py < PH and random.random() < 0.88:
            draw_pixel_block(px, py, dark_colors[0])

# --- Stars at top ---
random.seed(99)
for i in range(200):
    sx = random.randint(0, PW - 1)
    sy = random.randint(0, 42)
    brightness = 130 + random.randint(0, 125)
    if random.random() < 0.3:
        draw_pixel_block(sx, sy, (brightness, brightness, brightness), 2, 2)
    else:
        draw_pixel_block(sx, sy, (brightness, brightness, brightness))

# --- Birds (pixel V shapes) ---
birds = [(50, 30), (65, 28), (380, 35), (395, 33), (420, 30), (180, 25)]
for bx, by in birds:
    for i in range(3):
        draw_pixel_block(bx + i, by - i, (20, 10, 25))
        draw_pixel_block(bx + 3 + i, by - 2 + i, (20, 10, 25))

# --- Subtle reflection shimmer at very bottom ---
random.seed(77)
for py in range(PH - 30, PH):
    for px in range(PW):
        if random.random() < 0.06:
            r = min(255, dark_colors[0][0] + random.randint(0, 30))
            g = min(255, dark_colors[0][1] + random.randint(0, 15))
            b = dark_colors[0][2]
            draw_pixel_block(px, py, (r, g, b))

img.save("static/images/sunset-bg.png", "PNG", optimize=False)
print(f"Saved sunset-bg.png ({W}x{H})")
print("Done!")
