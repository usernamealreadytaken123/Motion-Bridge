"""Create an updated, hardware-neutral report diagram (PNG and SVG)."""

from html import escape
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parent
W, H, SCALE = 1800, 1190, 2
image = Image.new("RGB", (W * SCALE, H * SCALE), "white")
draw = ImageDraw.Draw(image)
svg = [
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
    '<rect width="100%" height="100%" fill="white"/>',
]


def font(size, bold=False):
    return ImageFont.truetype(
        "C:/Windows/Fonts/" + ("timesbd.ttf" if bold else "times.ttf"),
        size * SCALE,
    )


def text(x, y, value, size=27, bold=False, background=False):
    face = font(size, bold)
    if background:
        left, top, right, bottom = draw.textbbox(
            (x * SCALE, y * SCALE), value, font=face, anchor="mm"
        )
        draw.rectangle((left - 8, top - 5, right + 8, bottom + 5), fill="white")
        svg.append(
            f'<rect x="{(left-8)/SCALE}" y="{(top-5)/SCALE}" '
            f'width="{(right-left+16)/SCALE}" height="{(bottom-top+10)/SCALE}" fill="white"/>'
        )
    draw.text((x * SCALE, y * SCALE), value, font=face, fill="black", anchor="mm")
    weight = "bold" if bold else "normal"
    svg.append(
        f'<text x="{x}" y="{y}" font-family="Times New Roman,serif" '
        f'font-size="{size}" font-weight="{weight}" text-anchor="middle" '
        f'dominant-baseline="middle">{escape(value)}</text>'
    )


def line(points, arrow=True, dashed=False):
    for (a, b), (c, d) in zip(points, points[1:]):
        length = ((c - a) ** 2 + (d - b) ** 2) ** 0.5
        if dashed:
            for start in range(0, int(length), 20):
                end = min(start + 11, length)
                draw.line(
                    tuple(v * SCALE for v in (
                        a + (c-a)*start/length, b + (d-b)*start/length,
                        a + (c-a)*end/length, b + (d-b)*end/length,
                    )), fill="black", width=2*SCALE,
                )
        else:
            draw.line((a*SCALE, b*SCALE, c*SCALE, d*SCALE), fill="black", width=2*SCALE)
    coords = " ".join(f"{x},{y}" for x, y in points)
    dash = ' stroke-dasharray="11 9"' if dashed else ""
    svg.append(f'<polyline points="{coords}" fill="none" stroke="black" stroke-width="2"{dash}/>')
    if arrow:
        (a, b), (c, d) = points[-2:]
        length = ((c-a)**2 + (d-b)**2)**0.5
        ux, uy = (c-a)/length, (d-b)/length
        tip = [(c,d), (c-15*ux-7*uy,d-15*uy+7*ux), (c-15*ux+7*uy,d-15*uy-7*ux)]
        draw.polygon([(x*SCALE,y*SCALE) for x,y in tip], fill="black")
        coords = " ".join(f"{x},{y}" for x,y in tip)
        svg.append(f'<polygon points="{coords}" fill="black"/>')


def box(x, y, width, height, rows=(), dashed=False):
    line([(x,y),(x+width,y),(x+width,y+height),(x,y+height),(x,y)], False, dashed)
    size = 27
    while rows and max(font(size).getlength(row)/SCALE for row in rows) > width-28:
        size -= 1
    for index, row in enumerate(rows):
        text(x+width/2, y+height/2+(index-(len(rows)-1)/2)*35, row, size)


text(900, 48, "Функциональная схема программного комплекса", 38, True)
box(30,280,680,830)
box(780,280,990,830)
text(370,310,"Устройство с камерой и датчиками",29,True)
text(1275,310,"Персональный компьютер",30,True)

box(410,130,270,110,["Источник высоты", "LiDAR (опционально)"],True)
box(1290,130,390,110,["Видеозапись MP4"])
box(80,350,240,110,["Камера", "Последовательность", "кадров"])
box(390,350,260,110,["Датчики:", "ориентация, гироскоп,", "линейное ускорение"])
box(190,540,360,120,["Сбор исходных данных", "Привязка ко времени"])
box(80,730,260,130,["Локальная оценка", "ориентации и", "перемещения X, Y"])
box(390,730,260,130,["Передача кадров", "и данных датчиков"])
box(820,730,330,130,["Приём исходных данных", "Расчёт одометрии на ПК", "с учётом ориентации"])
box(820,960,330,100,["Приём готовых результатов", "и телеметрии"])
box(1290,350,390,140,["Загрузка и декодирование MP4", "Оценка плоского перемещения", "без компенсации наклона"])
box(1270,730,430,330,["Отображение и хранение:", "координаты и 2D-траектория;", "3D-ориентация и графики*;", "запись данных и результатов;", "просмотр и повторный анализ", "", "* при наличии нужных данных"])

line([(200,460),(200,500),(290,500),(290,540)])
line([(520,460),(520,500),(450,500),(450,540)])
line([(680,185),(695,185),(695,600),(550,600)],dashed=True)
line([(370,660),(370,690)],arrow=False)
line([(370,690),(210,690),(210,730)])
line([(370,690),(520,690),(520,730)])
draw.ellipse((367*SCALE,687*SCALE,373*SCALE,693*SCALE),fill="black")
svg.append('<circle cx="370" cy="690" r="3" fill="black"/>')
line([(650,795),(820,795)])
text(735,774,"TCP",22,background=True)
line([(210,860),(210,1010),(820,1010)])
text(745,989,"UDP",22,background=True)
line([(1150,795),(1270,795)])
line([(1150,1010),(1270,1010)])
line([(1485,240),(1485,350)])
line([(1485,490),(1485,730)])
text(1485,607,"Координаты и траектория",23,background=True)
text(900,1148,"Пунктиром обозначен дополнительный источник измерений высоты.",24)

svg.append("</svg>")
(OUT / "functional_scheme_2_1.svg").write_text("\n".join(svg),encoding="utf-8")
image.save(OUT / "functional_scheme_2_1.png",dpi=(300,300))
print(OUT / "functional_scheme_2_1.png")
print(OUT / "functional_scheme_2_1.svg")
