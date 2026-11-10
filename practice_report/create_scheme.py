from pathlib import Path
from html import escape
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
W, H, SCALE = 1800, 1180, 2
image = Image.new('RGB', (W*SCALE, H*SCALE), 'white')
draw = ImageDraw.Draw(image)
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">', '<rect width="100%" height="100%" fill="white"/>']

def text(x, y, value, size=27, bold=False, align='center'):
    font = ImageFont.truetype('C:/Windows/Fonts/'+('timesbd.ttf' if bold else 'times.ttf'), size*SCALE)
    draw.text((x*SCALE,y*SCALE),value,font=font,fill='black',anchor='mm' if align=='center' else 'lm')
    weight = 'bold' if bold else 'normal'
    anch = 'middle' if align=='center' else 'start'
    svg.append(f'<text x="{x}" y="{y}" font-family="Times New Roman,serif" font-size="{size}" font-weight="{weight}" text-anchor="{anch}" dominant-baseline="middle">{escape(value)}</text>')

def line(points, arrow=True, dashed=False):
    for (a,b),(c,d) in zip(points,points[1:]):
        length = ((c-a)**2+(d-b)**2)**.5
        if dashed:
            for start in range(0,int(length),20):
                end=min(start+11,length)
                draw.line(((a+(c-a)*start/length)*SCALE,(b+(d-b)*start/length)*SCALE,(a+(c-a)*end/length)*SCALE,(b+(d-b)*end/length)*SCALE),fill='black',width=2*SCALE)
        else:
            draw.line((a*SCALE,b*SCALE,c*SCALE,d*SCALE),fill='black',width=2*SCALE)
    coords=' '.join(f'{x},{y}' for x,y in points)
    dash=' stroke-dasharray="11 9"' if dashed else ''
    svg.append(f'<polyline points="{coords}" fill="none" stroke="black" stroke-width="2"{dash}/>')
    if arrow:
        (a,b),(c,d)=points[-2:]
        length=((c-a)**2+(d-b)**2)**.5
        ux,uy=(c-a)/length,(d-b)/length
        tip=[(c,d),(c-15*ux-7*uy,d-15*uy+7*ux),(c-15*ux+7*uy,d-15*uy-7*ux)]
        draw.polygon([(x*SCALE,y*SCALE) for x,y in tip],fill='black')
        coords=' '.join(f'{x},{y}' for x,y in tip)
        svg.append(f'<polygon points="{coords}" fill="black"/>')

def label(x,y,value,size=22,align='center'):
    font=ImageFont.truetype('C:/Windows/Fonts/times.ttf',size*SCALE)
    bounds=draw.textbbox((x*SCALE,y*SCALE),value,font=font,anchor='mm' if align=='center' else 'lm')
    l,t,r,b=bounds
    draw.rectangle((l-5*SCALE,t-3*SCALE,r+5*SCALE,b+3*SCALE),fill='white')
    svg.append(f'<rect x="{l/SCALE-5}" y="{t/SCALE-3}" width="{(r-l)/SCALE+10}" height="{(b-t)/SCALE+6}" fill="white"/>')
    text(x,y,value,size,False,align)

def box(x,y,w,h,rows=(),dashed=False):
    line([(x,y),(x+w,y),(x+w,y+h),(x,y+h),(x,y)],arrow=False,dashed=dashed)
    for i,row in enumerate(rows):
        size=27
        while ImageFont.truetype('C:/Windows/Fonts/times.ttf',size).getlength(row)>w-24:
            size-=1
        text(x+w/2,y+h/2+(i-(len(rows)-1)/2)*35,row,size)

text(W/2,48,'Функциональная схема программного комплекса',39,True)
box(30,280,1130,830)
box(1240,280,530,830)
text(595,309,'Android-приложение',30,True)
text(1505,309,'Приложение для ПК',30,True)
box(420,130,310,110,['iPhone: LiDAR','phyphox (опционально)'],True)
box(70,350,260,130,['Датчики Android:','ориентация, гироскоп,','линейное ускорение'])
box(420,350,310,130,['Сбор данных датчиков','и обработка','ориентации'])
box(70,590,260,130,['Камера Android','Видеокадры'])
box(420,575,310,170,['Камерная 2D-одометрия','Компенсация наклона','и учёт высоты','Координаты X, Y'])
box(70,845,260,110,['Ручной ввод','высоты камеры'])
box(420,845,310,130,['Получение глубины','и времени измерения','от phyphox'],True)
box(840,555,280,205,['Формирование','JSON-пакетов:','данные датчиков,','ориентация и X, Y'])
box(1290,350,430,145,['Визуализация:','3D-ориентация, 2D-траектория,','графики датчиков'])
box(1290,555,430,150,['Приём UDP-пакетов','Проверка и разбор данных','Контроль состояния связи'])
box(1290,805,430,105,['Запись измерений','в файл JSONL'])
box(1290,980,430,90,['Воспроизведение записи'])
line([(330,415),(420,415)])
line([(575,480),(575,575)])
label(585,530,'ориентация',align='left')
line([(330,655),(420,655)])
line([(730,415),(980,415),(980,555)])
label(858,396,'датчики и ориентация')
line([(730,655),(840,655)])
line([(330,900),(375,900),(375,705),(420,705)])
line([(575,845),(575,745)],dashed=True)
line([(730,185),(785,185),(785,910),(730,910)],dashed=True)
label(792,812,'HTTP / Wi-Fi',align='left')
line([(1120,630),(1290,630)])
label(1205,609,'UDP / Wi-Fi')
line([(1505,555),(1505,495)])
line([(1505,705),(1505,805)])
line([(1505,910),(1505,980)])
line([(1720,1025),(1742,1025),(1742,422),(1720,422)])
text(W/2,1148,'Пунктиром обозначен дополнительный источник данных LiDAR.',24)
svg.append('</svg>')
(OUT/'functional_scheme.svg').write_text('\n'.join(svg),encoding='utf-8')
image.save(OUT/'functional_scheme.png',dpi=(300,300))
intro='''Современные мобильные устройства оснащены камерой и датчиками, данные которых позволяют оценивать ориентацию и перемещение устройства. Разработка программных средств для обработки и наглядного отображения этих данных имеет практическое значение для исследования методов определения движения и создания измерительных систем.

Целью практики является разработка прототипа программного комплекса для оценки и визуализации ориентации и перемещения мобильного устройства. Комплекс включает Android-приложение для сбора данных датчиков и оценки перемещения по изображениям камеры, а также приложение для ПК, обеспечивающее приём, отображение и запись результатов. Обмен данными осуществляется по протоколу UDP в локальной сети. Дополнительно предусмотрено использование измерений LiDAR через приложение phyphox для учёта изменения высоты камеры при оценке перемещения над горизонтальной поверхностью.
'''
(OUT/'introduction.md').write_text(intro,encoding='utf-8')
print(OUT/'functional_scheme.png')
print(OUT/'functional_scheme.svg')
