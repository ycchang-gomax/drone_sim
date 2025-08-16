# make_map_bg.py  (pure-stdlib PPM generator)
from math import sin, pi
W, H = 1600, 1000
img = bytearray(W*H*3)

for y in range(H):
    for x in range(W):
        i = (y*W + x)*3
        # base ground
        r, g, b = 40, 70, 40
        # checker texture
        if ((x//40 + y//40) & 1) == 0:
            g = min(255, g+10)

        # roads (grid)
        if x % 220 < 8 or y % 220 < 8:
            r = g = b = 185

        # diagonal road
        if abs((y - x//2) % 220) < 4:
            r = g = b = 185

        # river (sine path)
        y0 = int(H*0.55 + 80*sin(2*pi*x/700.0) + 20*sin(2*pi*x/130.0))
        d = abs(y - y0)
        if d < 12:
            r, g, b = 40, 120, 210        # water
        elif d == 12:
            r, g, b = 220, 230, 250       # shoreline

        img[i]   = r
        img[i+1] = g
        img[i+2] = b

with open('assets/map_bg.ppm','wb') as f:
    f.write(f'P6\n{W} {H}\n255\n'.encode('ascii'))
    f.write(img)

print('Wrote assets/map_bg.ppm')
