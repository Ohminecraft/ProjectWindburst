"""Draws packaging/windburst.ico (the program's icon): the page's favicon - a quad's X frame on carbon, a blue hub - at every
size Windows asks for. Needs Pillow. python packaging/make_icon.py"""
import os

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))


def frame(size):
    s = size * 4                                         # drawn large, scaled down: smooth edges
    im = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle([0, 0, s - 1, s - 1], radius=s * 7 // 32, fill=(23, 28, 33, 255))
    w = max(4, s * 3 // 32)
    a, b = s * 9 // 32, s * 23 // 32
    d.line([(a, a), (b, b)], fill=(195, 204, 211, 255), width=w)
    d.line([(b, a), (a, b)], fill=(195, 204, 211, 255), width=w)
    for x, y in ((a, a), (b, a), (a, b), (b, b)):        # the four motors
        r = s * 3 // 32
        d.ellipse([x - r, y - r, x + r, y + r], outline=(195, 204, 211, 255), width=max(2, w // 2))
    r = s * 7 // 64
    d.ellipse([s // 2 - r, s // 2 - r, s // 2 + r, s // 2 + r], fill=(93, 134, 255, 255))
    return im.resize((size, size), Image.LANCZOS)


if __name__ == "__main__":
    sizes = [16, 24, 32, 48, 64, 128, 256]
    big = frame(256)
    big.save(os.path.join(HERE, "windburst.ico"), sizes=[(n, n) for n in sizes])
    big.save(os.path.join(HERE, "windburst.png"))
    print("ok")
