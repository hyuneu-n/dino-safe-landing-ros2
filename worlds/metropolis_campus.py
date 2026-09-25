"""Photo-guided Seokyeong campus: recognizable forms, fictional dimensions/layout."""
import math
from metropolis_region import bake, beam, smooth_path

STONE=(.67,.66,.60)
ROOF=(.23,.27,.29)
GLASS=(.12,.33,.43)
BRICK=(.48,.25,.20)
GOLD=(.94,.60,.10)


def cylinder(m,x,y,z,r,h,color,n=48):
    bottom=[(x+r*math.cos(i*math.tau/n),y+r*math.sin(i*math.tau/n),z) for i in range(n)]
    top=[(a,b,z+h) for a,b,_ in bottom]
    m.poly(bottom+top,[tuple(reversed(range(n))),tuple(range(n,2*n))]+
           [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)],color)


def arch(m,x,y,z,w,h,color):
    r=w/2
    vertices=[(x-r,y,z),(x+r,y,z)]
    vertices += [(x+r*math.cos(i*math.pi/16),y,z+h-r+r*math.sin(i*math.pi/16)) for i in range(17)]
    m.poly(vertices,[tuple(range(len(vertices)))],color)
    points=[(x-r,y-.04,z),(x-r,y-.04,z+h-r)]
    points += [(x+r*math.cos(math.pi-i*math.pi/16),y-.04,z+h-r+r*math.sin(math.pi-i*math.pi/16)) for i in range(17)]
    points += [(x+r,y-.04,z)]
    for a,b in zip(points,points[1:]):
        if sum((a[i]-b[i])**2 for i in range(3))>1e-8:beam(m,a,b,.22,STONE)
    m.box(x,y-.13,z+h*.40,.12,.12,h*.8,STONE)
    for dz in [h*.25,h*.50,h*.70]:m.box(x,y-.13,z+dz,w,.12,.12,STONE)


def hall(Mesh,w,d,h,color,arched=False,accent=False):
    m=Mesh();m.box(0,0,h/2,w,d,h,color,True)
    for side in [-1,1]:
        face=Mesh()
        for x in range(-int(w/2)+4,int(w/2)-1,6):
            if arched:
                arch(face,x,-d/2-.10,1.2,4.2,h-2.8,GLASS)
            else:
                for z in range(3,int(h)-1,3):
                    face.box(x,-d/2-.10,z,2.6,.12,1.8,GLASS)
                    face.box(x,-d/2-.19,z,2.7,.10,.10,STONE)
            face.box(x+2.8,-d/2-.16,h/2,.32,.26,h,STONE)
        for z in range(3,int(h),3):face.box(0,-d/2-.05,z-.8,w,.08,.12,STONE)
        bake(m,face,0,0,0,0 if side==-1 else math.pi)
    for y in range(-int(d/2)+3,int(d/2),4):
        for z in range(3,int(h)-1,3):
            for side in [-1,1]:m.box(side*(w/2+.03),y,z,.08,1.8,1.7,GLASS)
    # Mansard roof, dormers and gabled entrance echo the supplied photographs.
    lower=[(-w/2-.5,-d/2-.5,h),(w/2+.5,-d/2-.5,h),(w/2+.5,d/2+.5,h),(-w/2-.5,d/2+.5,h)]
    upper=[(-w/2+1.5,-d/2+3,h+5),(w/2-1.5,-d/2+3,h+5),(w/2-1.5,d/2-3,h+5),(-w/2+1.5,d/2-3,h+5)]
    m.poly(lower+upper,[(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7),(4,5,6,7)],ROOF)
    for x in range(-int(w/2)+5,int(w/2)-2,8):
        for sign in [-1,1]:
            yy=sign*(d/2-1)
            m.box(x,yy,h+1.8,2,.45,1.9,STONE)
            m.box(x,yy+sign*.25,h+1.8,1.2,.06,1.2,GLASS)
            m.poly([(x-1.3,yy-.4,h+2.7),(x+1.3,yy-.4,h+2.7),(x,yy-.4,h+3.8)],[(0,1,2)],STONE)
    if arched:
        for side in [-1,1]:
            yy=side*(d/2+.25)
            m.poly([(-7,yy,h),(7,yy,h),(0,yy,h+7)],[(0,1,2)],STONE)
            for i in range(32):
                a=i*math.tau/32;b=(i+1)*math.tau/32
                beam(m,(2*math.cos(a),yy+side*.1,h+2.8+2*math.sin(a)),
                     (2*math.cos(b),yy+side*.1,h+2.8+2*math.sin(b)),.18,GLASS)
    if accent:
        m.box(0,0,h+7,w-2,d-5,5,(.23,.26,.28),True)
        for x in range(-int(w/2)+4,int(w/2)-2,7):
            for side in [-1,1]:
                m.box(x,side*(d/2-2.4),h+7,3.8,.18,4.7,GOLD)
                m.box(x+.7,side*(d/2-2.25),h+7,1.8,.10,2,GLASS)
    return m


def add_campus(region):
    from metropolis_landmarks import label, GLYPHS
    c=region.c;Mesh=region.Mesh
    region.pad(350,338,12,225,116)
    def register(name,m,x,y,w,d,h,yaw=0):
        c.model(name,m,x,y,z=12,yaw=yaw,mesh_collision=True)
        c.buildings.append(dict(name=name,x=x,y=y,w=w,d=d,h=h,z=12,yaw=yaw,style='seokyeong_photo'))
    register('campus_south_arched_hall',hall(Mesh,96,18,22,STONE,arched=True),342,290,96,18,29)
    register('campus_east_yellow_hall',hall(Mesh,76,25,23,BRICK,arched=True,accent=True),414,344,76,25,33,math.pi/2)
    register('campus_west_brick_hall',hall(Mesh,37,24,24,BRICK),278,365,37,24,29)
    main=hall(Mesh,48,22,41,BRICK)
    main.box(-19,-11.25,20.5,7,.4,41,GLASS)
    main.box(0,0,46.3,44,15,.5,(.12,.40,.51))
    label(main,'SEOKYEONG',3,-11.4,36,32,(.31,.34,.35),(.93,.91,.83))
    register('campus_main_hall',main,341,382,48,22,47)
    tower=Mesh()
    cylinder(tower,0,0,0,14,52,GLASS)
    for z in range(0,53,3):cylinder(tower,0,0,z,14.2,.16,STONE)
    for i in range(48):
        t=i*math.tau/48;x,y=14.08*math.cos(t),14.08*math.sin(t)
        beam(tower,(x,y,.2),(x,y,52),.13,(.44,.57,.60))
        if i%5==0:
            # Alternating glass bays give the round tower a readable curved surface.
            t2=(i+1)*math.tau/48
            tower.poly([(x,y,4),(14.1*math.cos(t2),14.1*math.sin(t2),4),
                        (14.1*math.cos(t2),14.1*math.sin(t2),50),(x,y,50)],[(0,1,2,3)],(.16,.39,.49))
    cylinder(tower,0,0,52,14.8,.6,STONE)
    cylinder(tower,0,0,52.6,8,4,GLASS)
    cylinder(tower,0,0,56.6,8.5,.4,STONE)
    beam(tower,(0,0,57),(0,0,66),.25,STONE)
    register('campus_cylindrical_tower',tower,270,317,30,30,66)
    grounds=Mesh()
    grounds.box(346,338,12.03,104,63,.06,(.34,.36,.36),True)
    for i in range(20):
        yy=309+i*2.9
        grounds.box(346+(i%3-1)*1.1,yy,12.073,97-(i%4)*2,.62,.022,GOLD)
        grounds.box(342,yy+.9,12.073,83,.19,.022,(.70,.68,.60))
    # Ground lettering, legible in aerial footage without an external texture.
    text='SEOKYEONG UNIVERSITY';cell=.48;left=346-(len(text)*6-1)*cell/2
    for i,ch in enumerate(text):
        for row,bits in enumerate(GLYPHS[ch]):
            for col,bit in enumerate(bits):
                if bit=='1':grounds.box(left+(i*6+col)*cell,304+(6-row)*cell,12.10,cell*.9,cell*.9,.025,(.93,.91,.82))
    for x in [310,380]:
        grounds.box(x,359,12.09,13,7,.06,(.86,.49,.13))
        for yy in [355.5,362.5]:grounds.box(x,yy,12.14,13,.13,.04,STONE)
    # Lower fountain garden and stairs establish the school's steep campus character.
    region.pad(303,252,5,65,35)
    grounds.box(303,252,5.04,65,35,.08,(.61,.60,.54),True)
    cylinder(grounds,298,250,5.08,8,.9,STONE)
    cylinder(grounds,298,250,6.0,6.9,.08,(.13,.43,.51))
    cylinder(grounds,298,250,6.1,2,2.5,(.38,.44,.43))
    for i in range(28):
        z=5+(i+1)*.25;yy=266+i*.5
        grounds.box(323,yy,z-.125,9,.51,.25,STONE,True)
    for side in [-1,1]:
        beam(grounds,(323+side*4.4,266,6),(323+side*4.4,280,13),.13,(.30,.34,.34))
    for x,y,z in [(276,247,5),(324,247,5),(277,271,7),(302,277,10),(310,365,12),(389,307,12),(391,366,12)]:
        tree=Mesh();region.tree(tree,0,0,5)
        bake(grounds,tree,x,y,z)
        grounds.box(x,y,z+.35,3,3,.7,STONE,True)
    for x in [312,383]:grounds.box(x,335,12.55,3,.7,1.1,(.39,.27,.18),True)
    for x in [333,339,345]:
        grounds.poly([(x-.5,247,5),(x+.5,247,5),(x+.35,248,5),(x-.35,248,5),(x,247.5,8)],
                     [(0,1,4),(1,2,4),(2,3,4),(3,0,4)],STONE)
    c.model('seokyeong_campus_grounds',grounds,mesh_collision=True)
    # Keep the established approach open east of the front wing.
    region.road('campus_access',smooth_path([(350,225,5),(390,250,9),(398,280,12),(398,320,12),(340,330,12)]),7,mark=False)
    region.add_spot('campus_delivery','서경대학교 · 노란 광장','campus',340,330,12,'사진 기반 캠퍼스: 원통 유리 타워·아치 강의동·줄무늬 광장·계단 정원')
    c.landmarks.append(dict(id='seokyeong_campus',label='서경대학교 사진 기반 캠퍼스',x=350,y=335,z=12,fictional_layout=True,reference='Three user-provided campus photographs'))
