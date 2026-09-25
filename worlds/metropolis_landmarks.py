"""Recognizable neighborhood destinations and the delivery drone's home depot.

Signs are original block-letter geometry, baked into COLLADA (no downloaded logos
or raster textures). The campus is a fictional layout inspired by Seokyeong
University, not a surveyed reconstruction of the real campus.
"""
import math
from metropolis_region import ribbon, smooth_path, beam

GLYPHS = {
 'A':['01110','10001','10001','11111','10001','10001','10001'],
 'B':['11110','10001','10001','11110','10001','10001','11110'],
 'C':['01111','10000','10000','10000','10000','10000','01111'],
 'D':['11110','10001','10001','10001','10001','10001','11110'],
 'E':['11111','10000','10000','11110','10000','10000','11111'],
 'G':['01111','10000','10000','10111','10001','10001','01111'],
 'H':['10001','10001','10001','11111','10001','10001','10001'],
 'I':['11111','00100','00100','00100','00100','00100','11111'],
 'K':['10001','10010','10100','11000','10100','10010','10001'],
 'L':['10000','10000','10000','10000','10000','10000','11111'],
 'M':['10001','11011','10101','10101','10001','10001','10001'],
 'N':['10001','11001','11001','10101','10011','10011','10001'],
 'O':['01110','10001','10001','10001','10001','10001','01110'],
 'P':['11110','10001','10001','11110','10000','10000','10000'],
 'R':['11110','10001','10001','11110','10100','10010','10001'],
 'S':['01111','10000','10000','01110','00001','00001','11110'],
 'T':['11111','00100','00100','00100','00100','00100','00100'],
 'U':['10001','10001','10001','10001','10001','10001','01110'],
 'V':['10001','10001','10001','10001','10001','01010','00100'],
 'W':['10001','10001','10001','10101','10101','11011','10001'],
 'Y':['10001','10001','01010','00100','00100','00100','00100'],
 '0':['01110','10001','10011','10101','11001','10001','01110'],
 '1':['00100','01100','00100','00100','00100','00100','01110'],
 '2':['01110','10001','00001','00010','00100','01000','11111'],
 '3':['11110','00001','00001','01110','00001','00001','11110'],
 ' ':['00000']*7,
}


def lettering(m,text,x,y,z,cell,color):
    """Centered sign text in the X/Z plane, readable from the south (-Y)."""
    left=x-(len(text)*6-1)*cell/2
    for i,char in enumerate(text):
        for row,bits in enumerate(GLYPHS[char]):
            for col,bit in enumerate(bits):
                if bit=='1':m.box(left+(i*6+col+.5)*cell,y,z+(3-row)*cell,cell,cell*.35,cell,color)


def label(m,text,x,y,z,width,bg,fg):
    cell=min(width/(len(text)*6+2),.45)
    m.box(x,y+.10,z,width,.22,cell*9,bg,True)
    lettering(m,text,x,y-.035,z,cell,fg)


def arches(m,x,y,z,width=4,height=4):
    # Two freestanding golden arches with the common center reaching the ground.
    for side in [-1,1]:
        cx=x+side*width/4
        points=[(cx-width/4*math.cos(math.pi*i/24),y,z+height*math.sin(math.pi*i/24)) for i in range(25)]
        for a,b in zip(points,points[1:]):beam(m,a,b,.38,(.98,.71,.06))


def add_landmarks(region):
    c=region.c;Mesh=region.Mesh
    c.landmarks=[]

    # Home depot has a warehouse, numbered loading doors and a separate open pad.
    c.building('delivery_logistics_hub',-237,-50,54,32,12,'warehouse')
    m=Mesh();m.box(-222,-87,.07,84,38,.14,(.37,.39,.39),True)
    label(m,'SKYDROP LOGISTICS',-237,-66.25,10,43,(.10,.22,.32),(.89,.91,.86))
    for i,x in enumerate([-252,-237,-222]):
        m.box(x,-66.2,2.5,10,.2,5,(.18,.23,.27),True)
        label(m,str(i+1),x,-66.42,6,2,(.14,.24,.32),(.92,.76,.25))
        m.box(x,-68,.45,11,3,.9,(.50,.52,.50),True)
    for x in [-250,-239]:region.vehicle(m,x,-87,(.76,.79,.78),truck=True)
    for x,y in [(-224,-96),(-229,-96),(-224,-78)]:m.box(x,y,.7,2.5,2.5,1.4,(.48,.34,.20),True)
    # Pad with a ring and H, kept free of solid geometry at the center.
    ring=[(-210+5*math.cos(i*math.tau/64),-88+5*math.sin(i*math.tau/64),.155) for i in range(65)]
    ribbon(m,ring,.24,.02,(.93,.80,.28))
    for x in [-211,-209]:m.box(x,-88,.16,.3,3,.02,(.90,.90,.83))
    m.box(-210,-88,.16,2,.3,.02,(.90,.90,.83))
    for y in [-104,-72]:m.box(-201,y,3,.18,.18,6,(.21,.25,.27),True)
    label(m,'DRONE HUB',-201,-72,5,13,(.10,.22,.32),(.89,.91,.86))
    c.model('delivery_hub_yard',m)
    region.pad(-222,-87,0,84,38)
    route=[(-210,-88,0),(-175,-88,0),(-175,-112,0),(-150,-112,0)]
    region.road('depot_city_access',route,7,mark=False)
    region.add_spot('launch_depot','SkyDrop · 드론 물류센터','logistics',-210,-88,0,'배송 출발장, 하역문·배송 트럭·적재 화물')
    c.origin=dict(id='launch_depot',label='SkyDrop 드론 물류센터',x=-210,y=-88,z=0)
    c.departure_waypoints=[[-210,-88],[-175,-88],[-175,-112],[-150,-112],[-150,-56],[-150,0],[-90,0],[-30,0],[30,0],[90,0],[150,0]]
    c.landmarks.append(dict(id='logistics_hub',label='드론 물류센터',x=-237,y=-50,z=0))

    # A compact drive-through lot in the strip between downtown and the warehouses.
    c.building('mcdonalds_restaurant',172,86,20,18,6,'house')
    m=Mesh();m.box(172,76.7,5.1,21,.45,1.6,(.69,.07,.07),True)
    label(m,'MCDONALDS',172,76.36,5.1,18,(.69,.07,.07),(.98,.77,.08))
    m.box(172,86,6.6,21,19,.8,(.69,.07,.07),True)
    m.box(172,76.5,2.2,16,.14,3.0,(.16,.29,.33),True)
    arches(m,172,76.15,6.9,5,3.4)
    m.box(180,71,5,.35,.35,10,(.31,.32,.29),True)
    m.box(180,71,10.4,4.7,.7,4.8,(.69,.07,.07),True)
    arches(m,180,70.58,8.3,3.6,3.8)
    m.box(172,68,.07,23,19,.14,(.44,.44,.39),True)
    region.vehicle(m,177,61,(.70,.71,.69))
    m.box(166,64,.7,1.7,1.7,1.4,(.34,.28,.19),True)
    m.box(166,64,2.3,.15,.15,1.8,(.30,.28,.22),True)
    # Patio umbrella canopy, baked at its actual world position.
    m.poly([(164.3,62.3,2.8),(167.7,62.3,2.8),(167.7,65.7,2.8),(164.3,65.7,2.8),(166,64,3.5)],
           [(0,1,4),(1,2,4),(2,3,4),(3,0,4)],(.75,.10,.08))
    c.model('mcdonalds_signs_court',m)
    region.road('mcdonalds_access',[(150,56,0),(172,56,0),(172,68,0)],6,mark=False)
    region.add_spot('mcdonalds_delivery','맥도날드 · 매장 앞 픽업','restaurant',172,68,0,'간판·파라솔·주차 차량이 있는 매장 앞 배송 공간')
    c.landmarks.append(dict(id='mcdonalds',label='맥도날드',x=172,y=86,z=0))

    # Subway is a ground-floor tenant, preserving the existing urban building.
    shop=next(b for b in c.buildings if b['name']=='downtown_3_0_1')
    x,y=shop['x'],shop['y']-shop['d']/2
    m=Mesh()
    m.box(x,y-.24,2,14,.26,3.5,(.13,.26,.27),True)
    label(m,'SUBWAY',x,y-.5,4.5,14,(.04,.38,.20),(.96,.81,.12))
    m.box(x,y-1,3.55,15,2,.22,(.07,.40,.21),True)
    for dx in [-6,6]:m.box(x+dx,y-1,3.7,.5,2,.12,(.93,.79,.13))
    m.box(x+5,y-5,.7,1.3,1.3,1.4,(.35,.29,.19),True)
    c.model('subway_shopfront',m)
    region.add_spot('subway_delivery','써브웨이 · 도심 매장','restaurant',x,-104,0,'빌딩 1층 상가, 차양·보도·주변 도로의 좁은 배송 공간')
    c.landmarks.append(dict(id='subway',label='써브웨이',x=x,y=shop['y'],z=0))

    from metropolis_campus import add_campus
    add_campus(region)
