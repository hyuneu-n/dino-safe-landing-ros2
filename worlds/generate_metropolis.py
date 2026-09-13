#!/usr/bin/env python3
"""Connected downtown, freight district and residential town for Classic 11.

2026-09-14: authored street blocks replace unbounded random mesh scattering.
Facade boxes are baked into a single COLLADA mesh per building/streetscape;
collision boxes use those SAME dimensions. Generated assets live beside the world.
Only the scenario manifest contains destination knowledge; camera nodes are unchanged.
"""
import argparse
import json
import math
import random
from pathlib import Path
from xml.sax.saxutils import escape
import xml.etree.ElementTree as ET

from generate_world import DRONE_TEMPLATE, person

BRICK = [(0.39, .20, .15), (.52, .30, .22), (.58, .48, .36), (.32, .34, .35)]
GLASS = [(.12, .24, .30), (.19, .32, .38), (.25, .38, .43)]
STONE = (.63, .60, .53)
DARK = (.105, .125, .14)
ROAD = (.115, .13, .145)
PAVING = (.49, .49, .46)
WHITE = (.84, .82, .71)
GREEN = (.22, .34, .19)


class Mesh:
    """Colored, outward-facing boxes, batched by material into one mesh."""
    def __init__(self):
        self.groups = {}
        self.smooth_normals = False
        self.collisions = []

    def box(self, x, y, z, w, d, h, color, solid=False):
        assert min(w, d, h) > 0
        verts, tris = self.groups.setdefault(tuple(color), ([], []))
        off = len(verts)
        verts.extend([(x+a*w/2, y+b*d/2, z+c*h/2)
                      for a,b,c in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                    (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]])
        for face in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
            a,b,c,e = face
            tris.extend([off+a,off+b,off+c,off+a,off+c,off+e])
        if solid:
            self.collisions.append((x,y,z,w,d,h))

    def poly(self, vertices, faces, color):
        verts, tris = self.groups.setdefault(tuple(color), ([], []))
        offset = len(verts)
        verts.extend(vertices)
        for face in faces:
            for i in range(1, len(face)-1):
                tris.extend(offset+k for k in (face[0], face[i], face[i+1]))

    def canopy(self,x,y,z,r,h):
        vertices=[(x,y,z-h)]
        for latitude in [-math.pi/4,0,math.pi/4]:
            for k in range(8):
                angle=k*math.pi/4
                vertices.append((x+r*math.cos(latitude)*math.cos(angle),
                                 y+r*math.cos(latitude)*math.sin(angle),z+h*math.sin(latitude)))
        vertices.append((x,y,z+h))
        faces=[]
        for k in range(8):
            nxt=(k+1)%8
            faces.append((0,1+nxt,1+k))
            for ring in range(2):
                a=1+ring*8
                faces.append((a+k,a+nxt,a+8+nxt,a+8+k))
            faces.append((17+k,17+nxt,25))
        self.poly(vertices,faces,GREEN)
        self.collisions.append((x,y,z,r*1.5,r*1.5,h*1.6))

    def roof(self,w,d,z,h,color):
        self.poly([(-w/2,-d/2,z),(w/2,-d/2,z),(w/2,d/2,z),(-w/2,d/2,z),
                   (0,-d/2,z+h),(0,d/2,z+h)],
                  [(0,4,5,3),(1,2,5,4),(0,1,4),(3,5,2),(0,3,2,1)],color)
        self.collisions.append((0,0,z+h/2,w,d,h))

    def write(self, path):
        # Explicit normals avoid importer-dependent black facades.
        smooth={}
        if self.smooth_normals:
            for verts,tris in self.groups.values():
                for k in range(0,len(tris),3):
                    a,b,c=[verts[i] for i in tris[k:k+3]]
                    u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
                    n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
                    for p in (a,b,c):
                        target=smooth.setdefault(p,[0,0,0])
                        for j in range(3):target[j]+=n[j]
            for p,n in smooth.items():
                length=math.sqrt(sum(t*t for t in n))
                smooth[p]=[t/length for t in n]
        effects, materials, geos, nodes = [], [], [], []
        for idx,(color,(verts,tris)) in enumerate(self.groups.items()):
            key = f'm{idx}'
            rgba = ' '.join(map(str, (*color,1)))
            effects.append(f'<effect id="{key}fx"><profile_COMMON><technique sid="common"><phong>'
                           f'<ambient><color>{rgba}</color></ambient><diffuse><color>{rgba}</color></diffuse>'
                           '<specular><color>0.08 0.08 0.08 1</color></specular><shininess><float>12</float></shininess>'
                           '</phong></technique></profile_COMMON></effect>')
            materials.append(f'<material id="{key}"><instance_effect url="#{key}fx"/></material>')
            flat, normals = [], []
            for k in range(0,len(tris),3):
                a,b,c = [verts[i] for i in tris[k:k+3]]
                u,v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
                n = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
                length = math.sqrt(sum(t*t for t in n))
                for p in (a,b,c):
                    flat.extend(p)
                    normals.extend(smooth[p] if self.smooth_normals else (t/length for t in n))
            count=len(flat)//3
            sources=[]
            for suffix,data in [('pos',flat),('normal',normals)]:
                sid=f'{key}{suffix}'
                sources.append(f'<source id="{sid}"><float_array id="{sid}a" count="{len(data)}">'
                               +' '.join(f'{v:.4f}' for v in data)+f'</float_array><technique_common><accessor source="#{sid}a" count="{count}" stride="3">'
                               '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
                               '</accessor></technique_common></source>')
            geos.append(f'<geometry id="g{idx}"><mesh>'+''.join(sources)+
                        f'<vertices id="v{idx}"><input semantic="POSITION" source="#{key}pos"/></vertices>'
                        f'<triangles material="{key}" count="{count//3}"><input semantic="VERTEX" source="#v{idx}" offset="0"/>'
                        f'<input semantic="NORMAL" source="#{key}normal" offset="1"/><p>'+
                        ' '.join(f'{i} {i}' for i in range(count))+'</p></triangles></mesh></geometry>')
            nodes.append(f'<node id="n{idx}"><instance_geometry url="#g{idx}"><bind_material><technique_common>'
                         f'<instance_material symbol="{key}" target="#{key}"/></technique_common></bind_material></instance_geometry></node>')
        path.write_text('<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
                        '<asset><created>2026-09-14T00:00:00Z</created><modified>2026-09-14T00:00:00Z</modified><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>'
                        '<library_effects>'+''.join(effects)+'</library_effects><library_materials>'+''.join(materials)+
                        '</library_materials><library_geometries>'+''.join(geos)+'</library_geometries><library_visual_scenes>'
                        '<visual_scene id="Scene">'+''.join(nodes)+'</visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>')


class City:
    def __init__(self, out, seed):
        self.out = Path(out).resolve()
        self.assets=self.out.parent/(self.out.stem+'_assets')
        self.assets.mkdir(parents=True, exist_ok=True)
        self.rng=random.Random(seed)
        self.models=[]
        self.buildings=[]
        self.spots=[]
        self.audit_clearings={}
        self.roads3d=[]
        self.terrain_pads=[]
        self.seed=seed

    def model(self, name, mesh, x=0, y=0, yaw=0, z=0, mesh_collision=False):
        path=self.assets/(name+'.dae')
        mesh.write(path)
        cols=[]
        if mesh_collision:
            cols.append(f'<collision name="mesh_collision"><geometry><mesh><uri>{escape(path.as_uri())}</uri></mesh></geometry></collision>')
        for i,(cx,cy,cz,w,d,h) in enumerate([] if mesh_collision else mesh.collisions):
            cols.append(f'<collision name="c{i}"><pose>{cx} {cy} {cz} 0 0 0</pose><geometry><box><size>{w} {d} {h}</size></box></geometry></collision>')
        self.models.append(f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose><link name="l">'+''.join(cols)+
                           f'<visual name="v"><geometry><mesh><uri>{escape(path.as_uri())}</uri></mesh></geometry></visual></link></model>')

    def spot(self, ident, label, kind, x,y, note, z=0):
        self.spots.append(dict(id=ident,label=label,kind=kind,x=x,y=y,z=z,note=note,
                               flight_ready=z==0))

    def building(self,name,x,y,w,d,h,style='brick',z=0,yaw=0):
        base_z=z
        m=Mesh(); r=self.rng
        facade=r.choice(BRICK) if style!='glass' else r.choice(GLASS)
        if style=='stone':facade=(.64,.59,.48)
        m.box(0,0,h/2,w,d,h,facade,True)
        # Store exact collision footprint for layout audits and later dashboard map.
        self.buildings.append(dict(name=name,x=x,y=y,w=w,d=d,h=h,style=style,z=base_z,yaw=yaw))
        m.box(0,0,.4,w+.3,d+.3,.8,STONE)
        m.box(0,0,h+.18,w+.5,d+.5,.36,STONE)
        glass=r.choice(GLASS)
        for z in range(3,int(h)-1,3):
            for side in (-1,1):
                if style=='glass':
                    m.box(0,side*(d/2+.025),z,w-.7,.06,1.95,glass)
                    m.box(side*(w/2+.025),0,z,.06,d-.7,1.95,glass)
                else:
                    for ix in range(max(1,int(w/3))):
                        xx=-w/2+(ix+.5)*w/max(1,int(w/3))
                        m.box(xx,side*(d/2+.035),z,1.25,.08,1.65,glass)
                        m.box(xx,side*(d/2+.13),z-.88,1.55,.28,.13,STONE)
                    for iy in range(max(1,int(d/3))):
                        yy=-d/2+(iy+.5)*d/max(1,int(d/3))
                        m.box(side*(w/2+.035),yy,z,.08,1.25,1.65,glass)
        if style in ('glass','stone'):
            for xx in [-w/2+.25,0,w/2-.25]:
                m.box(xx,0,h/2,.24,d+.15,h,STONE)
            # Stepped penthouse and rooftop communications mast.
            m.box(0,0,h+3,w*.62,d*.62,6,facade,True)
            m.box(0,0,h+6.2,w*.66,d*.66,.4,STONE)
            if style=='stone':
                m.box(0,0,h+7.7,w*.38,d*.38,3,facade,True)
            if h>65: m.box(0,0,h+10,.35,.35,8,DARK,True)
        elif style != 'house':
            # Visible parapets and rooftop HVAC create geometric landing ambiguity.
            for yy in [-d/2,d/2]: m.box(0,yy,h+.65,w,.22,1,STONE,True)
            for xx in [-w/2,w/2]: m.box(xx,0,h+.65,.22,d,1,STONE,True)
            m.box(w*.2,0,h+.85,2,2,1.4,(.35,.38,.37),True)
        if style=='brick' and h>=15:
            # Exterior fire escape landings/ladders on the rear facade.
            for z in range(5,int(h),3):
                m.box(0,d/2+.65,z,3,1.4,.13,DARK,True)
                for xx in [-1.4,1.4]:m.box(xx,d/2+1.3,z+.5,.07,.07,1,DARK)
                m.box(0,d/2+1.3,z+1,3,.07,.07,DARK)
                for xx in [-.35,.35]:m.box(xx,d/2+1.15,z+1.5,.06,.06,3,DARK)
                for dz in [.5,1,1.5,2,2.5]:m.box(0,d/2+1.15,z+dz,.75,.06,.06,DARK)
        if style=='warehouse':
            for xx in [-w*.3,0,w*.3]:m.box(xx,0,h+.5,3,d*.6,.7,GLASS[1],True)
        # Shopfront and projecting awnings at street level.
        for xx in ([] if style in ('house','warehouse') else [-w*.27,w*.27]):
            m.box(xx,-d/2-.04,1.5,w*.35,.09,2.4,glass)
            m.box(xx,-d/2-.7,3.05,w*.4,1.5,.18,r.choice([(.19,.29,.25),(.53,.20,.14),(.16,.24,.33)]),True)
        self.model(name,m,x,y,yaw=yaw,z=base_z)


def tree(m,x,y,h=6):
    m.box(x,y,h*.3,.35,.35,h*.6,(.26,.19,.12),True)
    m.canopy(x,y,h*.72,h*.32,h*.36)


def bench(m,x,y):
    m.box(x,y,.48,2,.65,.16,(.40,.27,.16),True)
    m.box(x,y+.27,.85,2,.12,.65,(.40,.27,.16),True)
    for dx in [-.7,.7]:m.box(x+dx,y,.24,.12,.48,.48,DARK,True)


def vehicle(m,x,y,color=(.78,.56,.10),truck=False):
    w,d,h=(2.6,7,2.7) if truck else (1.85,4.3,1.25)
    m.box(x,y,.45+h/2,w,d,h,color,True)
    m.box(x,y-.2,h+.3,w*.8,d*.5,.65,GLASS[0],True)
    for dx in [-w/2,w/2]:
        for dy in [-d*.32,d*.32]:m.box(x+dx,y+dy,.48,.22,.72,.8,DARK)
    for dx in [-w*.3,w*.3]:m.box(x+dx,y-d/2-.02,.9,.35,.05,.22,WHITE)


def fence(m,x,y,w,d):
    # Deliberate 6m gate on south side. All fence pieces appear in depth/collision.
    for yy in [y-d/2,y+d/2]:
        for xx in [x-w/2+i*3 for i in range(int(w/3)+1)]:
            if yy<y and abs(xx-x)<3:continue
            m.box(xx,yy,1.1,.10,.10,2.2,DARK,True)
        for z in [.5,1.7]:
            if yy>y:m.box(x,yy,z,w,.07,.07,DARK,True)
            else:
                for sign in [-1,1]:m.box(x+sign*(w/4+1.5),yy,z,w/2-3,.07,.07,DARK,True)
    for xx in [x-w/2,x+w/2]:
        m.box(xx,y,1,.09,d,1.5,(.32,.35,.32),True)


def road(m,x,y,w,d,axis='x',mark=True):
    m.box(x,y,.025,w,d,.05,ROAD)
    if mark:
        length=w if axis=='x' else d
        for off in range(-int(length/2)+4,int(length/2)-2,9):
            if axis=='x':m.box(x+off,y,.058,4,.13,.016,(.78,.61,.25))
            else:m.box(x,y+off,.058,.13,4,.016,(.78,.61,.25))


def downtown(c):
    m=Mesh()
    xs=[-150,-90,-30,30,90,150]; ys=[-112,-56,0,56,112]
    for y in ys:road(m,0,y,336,16)
    for x in xs:road(m,x,0,16,252,'y')
    for x in xs:
        for y in ys:
            m.box(x,y,.075,16,16,.02,ROAD)
            for side in [-1,1]:
                for stripe in range(-5,6,2):
                    m.box(x+stripe,y+side*10,.085,.85,3.4,.018,WHITE)
                    m.box(x+side*10,y+stripe,.085,3.4,.85,.018,WHITE)
            # Traffic poles at alternate corners, few lights instead of hundreds of point lights.
            for side in [-1,1]:
                px,py=x+side*10,y+10
                m.box(px,py,2.5,.16,.16,5,DARK,True)
                m.box(px,py,4.45,.4,.35,1,DARK,True)
                m.box(px,py-.19,4.7,.19,.04,.19,(.65,.12,.07))
    for i in range(5):
        for j in range(4):
            x,y=(xs[i]+xs[i+1])/2,(ys[j]+ys[j+1])/2
            m.box(x,y,.13,44,40,.26,PAVING,True)
            if (i,j)==(3,2):
                m.box(x,y,.29,32,28,.06,(.65,.60,.49))
                for dx in [-15,15]:
                    for dy in [-13,13]:tree(m,x+dx,y+dy)
                for dx in [-10,10]:bench(m,x+dx,y+7)
                m.box(x,y+10,.9,5,2,1.5,STONE,True)
                c.spot('civic_plaza','Civic Plaza · 도심 광장','plaza',x,y-3,'석재 포장, 벤치·화단·보행자 사이 열린 공간')
                continue
            if (i,j)==(0,2):
                m.box(x,y,.28,34,30,.08,GREEN)
                road(m,x,y,4,30,'y',False)
                for dx,dy in [(-12,-10),(12,-10),(-12,10),(12,10)]:tree(m,x+dx,y+dy,7)
                bench(m,x+8,y);bench(m,x-8,y)
                c.spot('pocket_park','Hudson Green · 근린공원','park',x+7,y-6,'잔디와 산책로, 수목과 벤치')
                continue
            for k,dx in enumerate([-10,10]):
                h=c.rng.randint(5,10)*3
                style='brick'
                if 1<=i<=3 and j in (0,1,3) and k==0:
                    h=c.rng.randint(16,29)*3;style='glass'
                if (i,j,k)==(2,1,0):h,style=96,'stone'
                c.building(f'downtown_{i}_{j}_{k}',x+dx,y,16,27,h,style)
            for dx in [-18,18]:tree(m,x+dx,y+16,4.5)
            for dx in [-13,13]:
                m.box(x+dx,y-17,3,.13,.13,6,DARK,True)
                m.box(x+dx,y-17,6,.9,.45,.2,WHITE)
    for x,y in [(-146,-30),(-94,24),(-26,-82),(34,84),(86,-28),(146,40)]:
        vehicle(m,x,y)
    # Service alley/loading court along edge, away from the main route.
    c.spot('street_canyon','Lexington Ave · 빌딩 협곡','street',30,-28,'고층 외벽, 차도와 좁은 보도, 주차 차량')
    c.spot('depot','East Gate · 기본 배송지','helipad',150,0,'대로 교차로의 평탄한 아스팔트, 주변 신호등')
    c.model('downtown_streets',m)


def industrial(c):
    m=Mesh()
    road(m,264,0,212,16)
    for y in [-112,112]:road(m,264,y,212,12)
    for x in [190,280,370]:road(m,x,0,12,236,'y')
    for i,x in enumerate([235,325]):
        for j,y in enumerate([-56,56]):
            m.box(x,y,.06,74,92,.12,(.37,.36,.32),True)
            # Warehouse occupies back half; front half is a visibly usable logistics yard.
            wy=y+19
            c.building(f'warehouse_{i}_{j}',x,wy,54,27,10+3*i,'warehouse')
            for dx in [-16,0,16]:
                m.box(x+dx,wy-13.6,2.3,9,.15,4.5,(.25,.28,.27),True)
                m.box(x+dx,wy-15,.6,10,2,1.2,STONE,True)
            fence(m,x,y,74,92)
            for k in range(3):
                cx,cy=x-25+k*9,y-20
                color=[(.53,.23,.16),(.16,.32,.37),(.53,.48,.24)][k]
                m.box(cx,cy,1.4,6,12,2.8,color,True)
                for rib in range(-5,6,2):m.box(cx,cy+rib,1.4,6.1,.12,2.65,color)
                if k==0:m.box(cx,cy,4.2,6,12,2.8,color,True)
            vehicle(m,x+25,y-18,(.69,.68,.58),True)
            for dx,dy in [(7,-3),(15,-5),(23,1)]:
                m.box(x+dx,y+dy,.5,2,2,1,(.47,.34,.21),True)
            if i==0 and j==0:c.spot('freight_yard','Foundry Yard · 물류 야드','industrial',x+9,y-23,'컨테이너·트럭·팔레트, 좁게 남은 콘크리트 바닥')
            if i==1 and j==1:c.spot('warehouse_court','Warehouse Court · 하역장','industrial',x+7,y-14,'낮은 자재와 높은 창고, 가림과 높이 차이')
    # Brick boiler house / paired stacks mark the transition without repeating tanks everywhere.
    for x in [222,247]:
        m.box(x,-132,12,4,4,24,(.42,.24,.18),True)
        m.box(x,-132,24,4.5,4.5,.6,STONE)
    c.model('freight_streets_yards',m)


def town(c):
    m=Mesh()
    for y in [-112,0,112]:road(m,490,y,252,12)
    for x in [400,460,520,580]:road(m,x,0,10,236,'y')
    for y in [-56,56]:road(m,490,y,252,8)
    for i,x in enumerate([430,490,550]):
        for j,y in enumerate([-84,-28,28,84]):
            if (i,j)==(1,2):
                m.box(x,y,.08,44,42,.16,(.26,.38,.22),True)
                for dx in [-16,16]:
                    for dy in [-15,15]:tree(m,x+dx,y+dy,6)
                for dx in [-10,10]:bench(m,x+dx,y+10)
                c.spot('town_green','Maple Green · 마을 공원','green',x,y-5,'나무 그늘, 잔디, 벤치와 작은 보행 공간')
                continue
            for k,dx in enumerate([-12,12]):
                hx=x+dx
                m.box(hx,y,.09,21,42,.18,(.30,.39,.23),True)
                c.building(f'townhouse_{i}_{j}_{k}',hx,y+4,13,16,c.rng.choice([6,9]),'house')
                # Gabled silhouette distinguishes homes from flat-roof commercial blocks.
                roof=Mesh()
                bh=c.buildings[-1]['h']
                roof.roof(14,17,bh+.4,3,c.rng.choice([(.28,.30,.31),(.40,.22,.16),(.29,.32,.27)]))
                c.model(f'town_roof_{i}_{j}_{k}',roof,hx,y+4)
                m.box(hx,y-12,.20,2.2,12,.07,(.61,.57,.48))
                for xx in [hx-10,hx+10]:m.box(xx,y,.6,.18,40,1,(.55,.51,.42),True)
                tree(m,hx-7,y-13,4.2)
                if k==0:vehicle(m,hx+6,y-11,(.39,.48,.48))
    # Local shops and parking at the end of the boulevard.
    c.building('town_market',610,40,24,34,9,'brick')
    m.box(610,-15,.06,40,48,.12,(.33,.34,.32),True)
    for ix in range(5):
        xx=594+ix*7
        m.box(xx,-18,.135,.1,28,.02,WHITE)
        if ix!=2:vehicle(m,xx+3,-24,(.42,.48,.52))
    c.spot('market_parking','Maple Market · 상점 주차장','parking',611,-23,'주차선과 차량 사이 빈 자리, 주변 저층 상점')
    c.spot('residential_lane','Oak Lane · 주택 골목','street',520,28,'담장·주차 차량·정원 수목 사이 좁은 골목')
    # Allotments behind town form a rural edge, not another detached island.
    for x in [425,485,545]:
        m.box(x,153,.04,46,38,.08,(.34,.27,.17))
        for y in range(138,171,4):m.box(x,y,.18,44,.8,.28,(.33,.42,.20),True)
        fence(m,x,153,46,38)
    road(m,490,126,252,5,mark=False)
    for x in [400,460,520,580]:road(m,x,119,8,14,'y',False)
    c.spot('garden_edge','Allotment Gate · 텃밭 진입로','vacant',580,148,'흙과 초록 작물, 낮은 울타리, 비포장 진입 공간')
    road(m,580,146,5,40,'y',False)
    c.model('town_gardens_streets',m)


def waterfront(c):
    m=Mesh()
    # The regional river mesh supplies both banks and the water surface.
    m.box(235,-158,.05,950,25,.1,(.49,.47,.41),True)
    road(m,230,-136,900,10)
    for x in [-150,150,190,370,400,580]:
        road(m,x,-124,10,24,'y',False)
        m.box(x,-147,.08,4,14,.16,PAVING,True)
    for x in range(-190,641,24):
        tree(m,x,-153,6)
        bench(m,x+6,-161)
    for x in range(-205,661,6):m.box(x,-172,.8,.12,.12,1.6,DARK,True)
    m.box(230,-172,1.4,900,.08,.08,DARK,True)
    # Construction site: slab, blocks, excavated-looking dark patch and scaffold.
    m.box(675,20,.07,55,65,.14,(.44,.37,.26),True)
    fence(m,675,20,55,65)
    for dx in [-17,17]:
        for dy in [-17,0,17]:m.box(675+dx,20+dy,4,1,1,8,STONE,True)
    for z in [3,6]:m.box(675,30,z,35,18,.3,STONE,True)
    for dx,dy in [(-17,-24),(-10,-24),(15,-23)]:m.box(675+dx,20+dy,.4,4,2,.8,(.57,.43,.27),True)
    c.spot('construction','Riverside Works · 공사장','vacant',675,5,'낮은 적재물·기둥·상부 슬래브, 흙과 콘크리트 혼합')
    c.spot('promenade','Riverwalk · 수변 산책로','plaza',90,-160,'수면 경계, 난간·벤치·가로수, 길쭉한 포장면')
    road(m,665,0,80,10)
    c.model('riverwalk_construction',m)


def scenario_details(c):
    """Obstacles must fit the actual 12m scan footprint, not just the overview.

    Keep a 4m square available somewhere in the scan footprint. Four nominal GPS
    targets are occupied, requiring a visual offset. Clearings are retained only
    in memory for geometry tests, never sent to the detector/controller.
    """
    m=Mesh()
    for s in c.spots:
        x,y,kind=s['x'],s['y'],s['kind']
        if s['id']=='garden_edge':
            m.box(x-4,y+3,.24,3,4,.48,(.43,.31,.19),True)
            for dy in [2,3,4]:m.box(x-4,y+dy,.55,2.6,.5,.5,(.30,.43,.20),True)
            m.box(x+4,y-4,.6,5,.16,1.2,(.56,.49,.35),True)
            tree(m,x+5,y+4,3.5)
            s['obstacles']=['raised planting bed','crops','wooden fence','tree']
        elif s['id']=='construction':
            for dx in [3.5,6.5]:m.box(x+dx,y+4,1.5,.3,.3,3,(.46,.45,.40),True)
            m.box(x+5,y+4,3.1,4,2,.25,(.49,.46,.39),True)
            for dx in [-5,-3.5]:
                for dy in [-4,-2.5]:m.box(x+dx,y+dy,.3,1.2,1,.6,(.55,.32,.22),True)
            m.box(x-5,y+4,.65,2,2,1.3,(.49,.37,.22),True)
            s['obstacles']=['scaffold','low masonry','stacked timber']
        elif kind=='industrial':
            # Low pallet stack (depth-only challenge) and taller stacked cargo.
            for z in [.15,.4,.65]:
                for dy in [-.65,0,.65]:
                    m.box(x+4,y-3+dy,z,2.2,.23,.12,(.51,.36,.22),True)
            m.box(x-5,y+3,1.15,2.5,3.4,2.3,(.22,.34,.38),True)
            m.box(x-5,y+3,2.55,2.1,2.8,.5,(.50,.31,.18),True)
            for dx,dy in [(4,4),(5.5,3)]:
                m.box(x+dx,y+dy,.65,.8,.8,1.3,(.60,.31,.13),True)
            s['obstacles']=['low pallets','cargo crates','barrels']
        elif kind in ('park','green','plaza'):
            bench(m,x+5,y+2)
            m.box(x-5,y+3,.4,2.6,2.6,.8,STONE,True)
            tree(m,x-5,y+3,4)
            s['obstacles']=['bench','planter','tree','pedestrian']
            c.models.append(person('visitor_'+s['id'],x+4,y-4,color=(.65,.27,.16)))
        elif kind=='street':
            vehicle(m,x+4,y+2,(.66,.51,.15))
            for dy in [-4,-2]:
                m.box(x-4,y+dy,.38,.5,.5,.76,(.88,.35,.09),True)
                m.box(x-4,y+dy,.48,.53,.53,.12,WHITE)
            s['obstacles']=['parked car','traffic bollards','building walls']
        elif kind=='parking':
            m.box(x+5,y+4,.7,1,1,1.4,(.18,.33,.24),True)
            s['obstacles']=['parked cars','parking lines','waste bin']
        else:
            # The default delivery point remains easier but still contains obstacles.
            for dx in [-5,5]:
                m.box(x+dx,y+4,.35,.6,.6,.7,(.76,.34,.11),True)
            m.box(x-4,y+3,.4,2.2,2.2,.8,STONE,True)
            m.box(x-4,y+3,1,1.8,1.8,.5,GREEN,True)
            s['obstacles']=['traffic bollards','planter','road markings']
        blocked=s['id'] in ('civic_plaza','freight_yard','market_parking','construction')
        s['nominal_target_blocked']=blocked
        c.audit_clearings[s['id']]=(x,y-5 if blocked else y)
        if blocked:
            if s['id']=='civic_plaza':
                c.models.append(person('plaza_customer',x,y,color=(.7,.28,.13)))
            elif s['id']=='market_parking':
                vehicle(m,x,y,(.42,.49,.55))
            elif s['id']=='construction':
                m.box(x,y,.32,1.8,2,.64,(.55,.34,.24),True)
            else:
                m.box(x,y,.8,1.8,2.4,1.6,(.51,.37,.23),True)
            s['note']+=' · 지정 좌표가 점유됨: 영상에서 주변 대체 공간 탐색 필요'
    c.model('landing_scenario_props',m)


def generate(out,seed=0):
    c=City(out,seed)
    downtown(c);industrial(c);town(c);waterfront(c);scenario_details(c)
    from metropolis_region import extend_region
    extend_region(c, Mesh, tree, vehicle)
    # Controlled stationary pedestrians are visible obstacles; dynamic intruder stays compatible.
    for i,(x,y) in enumerate([(65,29),(55,37),(-116,18),(485,34),(613,-6),(77,-156),(246,-61)]):
        c.models.append(person(f'pedestrian_{i}',x,y,color=(.23+.06*(i%4),.26,.38)))
    c.models.append(person('person_intruder',150,9,static=False,color=(.8,.38,.13)))
    c.models.append(DRONE_TEMPLATE.replace("<far>500</far>","<far>2500</far>").format(sx=-150,sy=0,sz=7))
    xml='<?xml version="1.0"?><sdf version="1.6"><world name="metropolis">'
    xml+='<model name="ground_collision"><static>true</static><pose>230 0 -0.15 0 0 0</pose><link name="l"><collision name="c"><geometry><box><size>2400 2400 0.2</size></box></geometry></collision></link></model>'
    xml+='<light type="directional" name="afternoon_sun"><cast_shadows>true</cast_shadows><pose>0 0 200 0 0 0</pose><diffuse>0.95 0.88 0.76 1</diffuse><specular>0.15 0.15 0.15 1</specular><direction>-0.45 0.35 -0.82</direction></light>'
    xml+='<scene><ambient>0.48 0.51 0.55 1</ambient><background>0.65 0.76 0.83 1</background><shadows>true</shadows></scene>'
    xml+='<physics type="ode"><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>'
    xml+='<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros><update_rate>30</update_rate></plugin>'
    xml+='<gui><camera name="user_camera"><pose>-235 -220 170 0 0.58 0.45</pose></camera></gui>'
    # Terrain and riverbank meshes are generated by metropolis_region.
    xml+=''.join(c.models)+'</world></sdf>'
    ET.fromstring(xml)
    c.out.write_text(xml)
    manifest=dict(layout='metropolis',seed=seed,start_xy=[-150,0],target_xy=[150,0],
                  waypoints_xy=[[-150,0],[-90,0],[-30,0],[30,0],[90,0],[150,0]],
                  landing_spots=c.spots,buildings=c.buildings,roads3d=c.roads3d,
                  region_bounds=[-950,1550,-850,1050],
                  districts=[dict(id='downtown',bounds=[-180,180,-125,125]),dict(id='industrial',bounds=[180,376,-125,125]),
                             dict(id='town',bounds=[376,640,-125,175])])
    c.out.with_suffix('.waypoints.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print(f'[ok] {c.out}: {len(c.buildings)} buildings, {len(c.spots)} destinations, {len(c.models)} models')
    return c,manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--out',default=str(Path(__file__).parent/'generated/metropolis.world'))
    args=p.parse_args()
    generate(args.out,args.seed)
