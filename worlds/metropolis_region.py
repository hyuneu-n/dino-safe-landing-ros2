"""River basin, grade-separated expressway and terraced mountain neighborhoods.

All road decks and terrain use the same triangle mesh for rendering and collision.
Road paths store elevation and width; river, pads and road cuts share one terrain
function. The flat original city remains the core of this larger region.
"""
import math

ASPHALT = (.18,.20,.21)
CONCRETE = (.52,.53,.50)
STRIPE = (.83,.81,.71)
STEEL = (.27,.31,.32)


def distance(a,b):
    return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))


def bezier(a,b,c,d,n=32):
    return [tuple((1-t)**3*a[k]+3*(1-t)**2*t*b[k]+3*(1-t)*t*t*c[k]+t**3*d[k]
                  for k in range(3)) for t in [i/n for i in range(n+1)]]


def smooth_path(points, step=7):
    """Catmull-Rom spline through authored 3D control points."""
    out=[]
    for i in range(len(points)-1):
        p0,p1,p2,p3=points[max(0,i-1)],points[i],points[i+1],points[min(len(points)-1,i+2)]
        count=max(3,math.ceil(distance(p1,p2)/step))
        for j in range(count):
            t=j/count
            out.append(tuple(.5*(2*p1[k]+(-p0[k]+p2[k])*t+
                                (2*p0[k]-5*p1[k]+4*p2[k]-p3[k])*t*t+
                                (-p0[k]+3*p1[k]-3*p2[k]+p3[k])*t**3) for k in range(3)))
    return out+[tuple(points[-1])]


def shifted(points, offset):
    out=[]
    for i,p in enumerate(points):
        a,b=points[max(0,i-1)],points[min(len(points)-1,i+1)]
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        out.append((p[0]-dy/length*offset,p[1]+dx/length*offset,p[2]))
    return out


def ribbon(mesh, points, width, thickness, color, offset=0, lift=0):
    left,right=shifted(points,offset+width/2),shifted(points,offset-width/2)
    vertices=[]
    for a,b in zip(left,right):
        vertices.extend([(a[0],a[1],a[2]+lift),(b[0],b[1],b[2]+lift),
                         (a[0],a[1],a[2]+lift-thickness),(b[0],b[1],b[2]+lift-thickness)])
    faces=[]
    for i in range(len(points)-1):
        k=i*4;n=k+4
        faces.extend([(k,k+1,n+1,n),(k+2,n+2,n+3,k+3),
                      (k,n,n+2,k+2),(k+1,k+3,n+3,n+1)])
    faces.extend([(0,2,3,1),(len(vertices)-4,len(vertices)-3,len(vertices)-1,len(vertices)-2)])
    mesh.poly(vertices,faces,color)


def beam(mesh,a,b,width,color):
    axis=[b[i]-a[i] for i in range(3)]
    length=math.sqrt(sum(t*t for t in axis));axis=[t/length for t in axis]
    ref=(0,0,1) if abs(axis[2])<.9 else (0,1,0)
    u=[axis[1]*ref[2]-axis[2]*ref[1],axis[2]*ref[0]-axis[0]*ref[2],axis[0]*ref[1]-axis[1]*ref[0]]
    norm=math.sqrt(sum(t*t for t in u));u=[t/norm*width/2 for t in u]
    v=[axis[1]*u[2]-axis[2]*u[1],axis[2]*u[0]-axis[0]*u[2],axis[0]*u[1]-axis[1]*u[0]]
    vertices=[tuple(p[k]+su*u[k]+sv*v[k] for k in range(3)) for p in (a,b) for su,sv in [(-1,-1),(1,-1),(1,1),(-1,1)]]
    mesh.poly(vertices,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],color)


def bake(target,source,x,y,z=0,yaw=0):
    ct,st=math.cos(yaw),math.sin(yaw)
    for color,(verts,tris) in source.groups.items():
        tv,tt=target.groups.setdefault(color,([],[]));offset=len(tv)
        tv.extend((x+vx*ct-vy*st,y+vx*st+vy*ct,z+vz) for vx,vy,vz in verts)
        tt.extend(offset+i for i in tris)


def north_bank(x):
    if x < -250:return -175+35*math.sin((x+250)/220)
    if x > 700:return -175+48*math.sin((x-700)/250)
    return -175


def south_bank(x):
    return north_bank(x)-185-22*math.sin(x/180)


def base_height(x,y):
    if y<175:return 0.0
    t=min(1,max(0,(y-175)/210));fade=t*t*(3-2*t)
    peaks=[(300,720,170,235,200),(850,690,110,250,230),(-340,560,125,210,230),(1200,480,70,220,200)]
    return fade*sum(h*math.exp(-((x-cx)/sx)**2-((y-cy)/sy)**2) for cx,cy,h,sx,sy in peaks)


class Region:
    def __init__(self,city,Mesh,tree,vehicle):
        self.c=city;self.Mesh=Mesh;self.tree=tree;self.vehicle=vehicle
        self.ground_paths=[]
        self.pads=[]
        self.road_samples=[]
        self.road_bins={}

    def road(self,name,points,width,elevated=False,rails=False,mark=True):
        m=self.Mesh()
        ribbon(m,points,width,1.2 if elevated or max(p[2] for p in points)>1 else .22,ASPHALT)
        if mark:
            offsets=[0] if width<14 else [-width/4,0,width/4]
            for off in offsets:
                for i in range(0,len(points)-1,3):
                    ribbon(m,points[i:i+2],.15,.015,STRIPE,offset=off,lift=.022)
            for off in [-width/2+.45,width/2-.45]:
                ribbon(m,points,.16,.018,STRIPE,offset=off,lift=.025)
        if elevated or rails:
            for off in [-width/2+.13,width/2-.13]:
                if name in ('river_bridge','riverside_expressway'):
                    section=[]
                    for a,b in zip(points,points[1:]):
                        x,y=(a[0]+b[0])/2,(a[1]+b[1])/2
                        opening=(name=='river_bridge' and off<0 and
                                 (-432<y<-408 or -380<y<-330)) or (
                                 name=='riverside_expressway' and
                                 ((off>0 and 900<x<1000) or (off<0 and 640<x<735)))
                        if opening:
                            if len(section)>1:ribbon(m,section,.26,.85,CONCRETE,offset=off,lift=.85)
                            section=[]
                        else:
                            if not section:section=[a]
                            section.append(b)
                    if len(section)>1:ribbon(m,section,.26,.85,CONCRETE,offset=off,lift=.85)
                else:ribbon(m,points,.26,.85,CONCRETE,offset=off,lift=.85)
        if elevated:
            travelled=0
            for i,p in enumerate(points):
                if i:travelled+=distance(points[i-1],p)
                if travelled<45:continue
                travelled=0
                floor=-1.5 if south_bank(p[0])<p[1]<north_bank(p[0]) else 0
                if p[2]-floor>4:
                    m.box(p[0],p[1],(floor+p[2]-1.2)/2,2.7,2.7,p[2]-1.2-floor,CONCRETE)
                    tangent=shifted(points[max(0,i-1):min(len(points),i+2)],width*.36)
                    opposite=shifted(points[max(0,i-1):min(len(points),i+2)],-width*.36)
                    q=min(1,len(tangent)-1)
                    beam(m,(*tangent[q][:2],p[2]-1.7),(*opposite[q][:2],p[2]-1.7),1.1,CONCRETE)
        else:
            self.ground_paths.append((points,width))
            for p in points:
                sample=(p[0],p[1],p[2]-.12,width/2)
                self.road_samples.append(sample)
                self.road_bins.setdefault((math.floor(p[0]/50),math.floor(p[1]/50)),[]).append(sample)
        if elevated:
            # Earth-supported toes connect low ramp ends to surrounding terrain.
            for p in points:
                if p[2]>10:continue
                sample=(p[0],p[1],p[2]-.12,width/2)
                self.road_samples.append(sample)
                self.road_bins.setdefault((math.floor(p[0]/50),math.floor(p[1]/50)),[]).append(sample)
        self.c.model(name,m,mesh_collision=True)
        if elevated and width>=14:
            traffic=self.Mesh()
            for vehicle_idx,i in enumerate(range(12,len(points)-2,18)):
                p=points[i];a,b=points[i-1],points[i+1]
                heading=math.atan2(b[1]-a[1],b[0]-a[0])
                side=-1 if vehicle_idx%2 else 1
                offset=side*width*.24
                local=self.Mesh()
                self.vehicle(local,0,0,[(.72,.69,.59),(.21,.32,.39),(.57,.29,.19)][vehicle_idx%3])
                bake(traffic,local,p[0]-math.sin(heading)*offset,p[1]+math.cos(heading)*offset,
                     p[2],heading-math.pi/2+(math.pi if side==1 else 0))
            if traffic.groups:self.c.model(name+'_traffic',traffic,mesh_collision=True)
        self.c.roads3d.append(dict(id=name,width=width,elevated=elevated,points=points))

    def pad(self,x,y,z,w,d,yaw=0):
        self.pads.append((x,y,z,w,d,yaw))
        self.c.terrain_pads.append((x,y,z,w,d,yaw))

    def terrain_height(self,x,y):
        # Preserve the original city's flat ground exactly.
        if -210<=x<=705 and -174<=y<=175:return -.035
        value=base_height(x,y)
        # A nearest-road cut/fill makes a level cross-section, including steep slopes.
        nearest=None;best=1e9;weighted=0;weights=0
        bx,by=math.floor(x/50),math.floor(y/50)
        for ix in range(bx-2,bx+3):
            for iy in range(by-2,by+3):
                for rx,ry,rz,half in self.road_bins.get((ix,iy),[]):
                    ds=(x-rx)**2+(y-ry)**2
                    if ds>3600:continue
                    if ds<best:best=ds;nearest=(rz,half)
                    weight=math.exp(-ds/625)
                    weighted+=rz*weight;weights+=weight
        if nearest:
            rz,half=nearest;d=math.sqrt(best)
            influence=math.exp(-best/1600)
            value=value*(1-influence)+(weighted/weights)*influence
            flat=max(0,min(1,(half+17-d)/14))
            value=value*(1-flat)+rz*flat
        for px,py,pz,w,d,yaw in self.pads:
            dx,dy=x-px,y-py
            u=dx*math.cos(yaw)+dy*math.sin(yaw)
            v=-dx*math.sin(yaw)+dy*math.cos(yaw)
            distance_out=max(abs(u)-w/2,abs(v)-d/2,0)
            blend=max(0,1-distance_out/28)
            if blend:value=value*(1-blend)+pz*blend
        # Road cuts have final priority over neighboring house terraces. The cut
        # extends beyond half-width by one grid cell so triangulation cannot bury
        # the asphalt between terrain samples; the solid road slab fills the cut.
        if nearest:
            rz,half=nearest;d=math.sqrt(best)
            core=half+10
            blend=max(0,min(1,(core+12-d)/12))
            cut=rz-(.8 if rz>1 else .05)
            value=min(value,value*(1-blend)+cut*blend)
        for s in self.c.spots[12:]:
            if s['kind']=='elevated':continue
            outside=max(abs(x-s['x'])-10,abs(y-s['y'])-10,0)
            blend=max(0,1-outside/10)
            value=value*(1-blend)+s['z']*blend
        return value-.035

    def terrain(self):
        m=self.Mesh();m.smooth_normals=True
        # 8m terrain cells plus exact pads resolve road cuts without jagged terraces.
        xs=list(range(-950,1551,8))
        for bank in ['north','south']:
            rows=150 if bank=='north' else 60
            grid=[]
            for x in xs:
                edge=north_bank(x) if bank=='north' else south_bank(x)
                limit=1050 if bank=='north' else -850
                grid.append([(x,edge+(limit-edge)*j/rows,
                              self.terrain_height(x,edge+(limit-edge)*j/rows) if bank=='north' else -.035)
                             for j in range(rows+1)])
            for i in range(len(xs)-1):
                for j in range(rows):
                    verts=[grid[i][j],grid[i+1][j],grid[i+1][j+1],grid[i][j+1]]
                    if bank=='south':verts.reverse()
                    z=sum(p[2] for p in verts)/4
                    slope=max(p[2] for p in verts)-min(p[2] for p in verts)
                    color=(.32,.38,.26)
                    if z>45:color=(.29,.36,.24)
                    if z>100:color=(.33,.38,.29)
                    if slope>10:color=(.42,.43,.36)
                    m.poly(verts,[(0,1,2),(0,2,3)],color)
        # Explicit flat pads eliminate interpolation errors beneath buildings/clearings.
        for x,y,z,w,d,yaw in self.pads:
            corners=[]
            for dx,dy in [(-w/2,-d/2),(w/2,-d/2),(w/2,d/2),(-w/2,d/2)]:
                corners.append((x+dx*math.cos(yaw)-dy*math.sin(yaw),y+dx*math.sin(yaw)+dy*math.cos(yaw),z))
            m.poly(corners,[(0,1,2,3)],(.47,.45,.38))
        self.c.model('regional_terrain',m,mesh_collision=True)
        water=self.Mesh();bankmesh=self.Mesh()
        for a,b in zip(xs,xs[1:]):
            water.poly([(a,north_bank(a),-.16),(a,south_bank(a),-.16),
                        (b,south_bank(b),-.16),(b,north_bank(b),-.16)],[(0,1,2,3)],(.16,.31,.35))
            for fn in [north_bank,south_bank]:
                pts=[(a,fn(a),-.02),(b,fn(b),-.02)]
                ribbon(bankmesh,pts,2.5,.65,(.50,.49,.42))
        self.c.model('river_surface',water,mesh_collision=True)
        self.c.model('river_embankments',bankmesh,mesh_collision=True)

    def expressways(self):
        hy=lambda x:-225+17*math.sin(x/270)
        highway=[(x,hy(x),24) for x in range(-550,1251,8)]
        self.road('riverside_expressway',highway,24,True)
        west=bezier((-850,-90,0),(-800,-145,0),(-690,hy(-550),24),highway[0],50)
        self.road('west_ground_ramp',west,15,True)
        east=bezier(highway[-1],(1370,hy(1250),24),(1480,-150,0),(1480,-45,0),50)
        self.road('east_ground_ramp',east,15,True)
        bridge=[(720,y,34) for y in range(-560,-39,8)]
        self.road('river_bridge',bridge,18,True)
        # Bridge approaches connect to the old east boulevard and south-bank streets.
        self.road('bridge_north_approach',bezier(bridge[-1],(720,210,34),(1100,240,0),(1340,240,0),75),18,True)
        self.road('bridge_south_approach',bezier((120,-760,0),(400,-760,0),(720,-720,34),bridge[0],75),18,True)
        self.road('south_bridge_ground_link',smooth_path([(120,-760,0),(120,-590,0),(0,-420,0)]),12)
        # 270-degree loop: bridge -> expressway, maintaining exact endpoint heights.
        start=(720,-330,34)
        circle=[]
        for i in range(65):
            t=i/64;angle=math.pi+1.5*math.pi*t
            u=max(0,(t-.18)/.82)
            circle.append((780+60*math.cos(angle),-330+60*math.sin(angle),34-8*u*u*(3-2*u)))
        end=(650,hy(650),24)
        merge=bezier(circle[-1],(740,-270,26),(700,hy(650),24),end,24)
        self.road('bridge_loop_ramp',circle+merge[1:],8,True)
        # Broad curling off-ramp down to the eastern local street network.
        circle=[(950+60*math.cos(1.5*math.pi*i/64),-90+60*math.sin(1.5*math.pi*i/64),20-12*i/64) for i in range(65)]
        entry=bezier((900,hy(900),24),(990,hy(900),24),(1010,-145,20),circle[0],28)
        exit_path=bezier(circle[-1],(1040,-150,8),(1140,-150,0),(1140,-80,0),32)
        self.road('spiral_to_surface',entry+circle[1:]+exit_path[1:],8,True)
        self.road('east_local_link',smooth_path([(1140,-80,0),(1140,30,0),(990,65,0),(820,30,0),(720,0,0),(625,0,0)]),10)
        self.road('east_outer_avenue',smooth_path([(1140,30,0),(1300,80,0),(1470,60,0),(1480,-45,0)]),12)
        self.road('bridge_ground_link',smooth_path([(1340,240,0),(1400,160,0),(1470,60,0)]),12)
        # Cable-stayed bridge landmark: tower legs clear both road edges.
        m=self.Mesh()
        for y in [-305,-465]:
            for side in [-1,1]:
                x=720+side*12
                beam(m,(x,y,-1),(x,y,81),2.5,CONCRETE)
                for delta in [-64,-40,40,64]:
                    beam(m,(x,y,77),(720+side*8,y+delta,34.8),.17,(.73,.75,.72))
            beam(m,(708,y,72),(732,y,72),2.3,(.60,.29,.16))
        self.c.model('cable_bridge_pylons',m,mesh_collision=True)
        # Ground boulevard on the far bank, with a direct bridge approach junction.
        self.road('south_bank_boulevard',smooth_path([(-600,-440,0),(-300,-435,0),(0,-420,0),(300,-440,0),(500,-470,0),(720,-770,0),(990,-560,0),(1260,-480,0)]),14)
        self.road('west_local_link',smooth_path([(-150,0,0),(-230,35,0),(-360,90,0),(-480,120,0),(-650,80,0),(-780,-5,0),(-850,-90,0)]),12)
        # Side turnout on the bridge: a genuine elevated surface, not a floating pad.
        turnout=self.Mesh();turnout.box(735,-420,33.4,12,25,1.2,CONCRETE,True)
        for y in [-431,-409]:turnout.box(735,y,34.5,12,.3,1,CONCRETE,True)
        turnout.box(740.8,-420,34.5,.3,24,1,CONCRETE,True)
        turnout.box(739,-420,16,2,2,32,CONCRETE,True)
        self.c.model('bridge_service_turnout',turnout)
        self.add_spot('bridge_turnout','River Bridge · 고가 점검 공간','elevated',735,-420,34,'고가 상판, 난간, 수면과 수직 낙차')
        self.add_spot('under_bridge','Bridge Shelter · 교량 아래','underpass',720,-490,0,'교량 아래의 그늘, 기둥, 상부 구조물',underpass=True)

    def add_spot(self,ident,label,kind,x,y,z,note,underpass=False):
        self.c.spot(ident,label,kind,x,y,note,z)
        s=self.c.spots[-1]
        s.update(nominal_target_blocked=False,obstacles=['railing','low cargo','planter'],scan_altitude=z+12)
        self.c.audit_clearings[ident]=(x,y)
        if kind != 'elevated':self.pad(x,y,z,13,13)
        m=self.Mesh()
        m.box(x-4.5,y+3,z+.55,1.8,2,1.1,(.51,.35,.22),True)
        m.box(x+4.5,y+3,z+.4,2,2,.8,(.45,.46,.42),True)
        m.box(x+4.5,y+3,z+1.1,1.5,1.5,.6,(.25,.39,.23),True)
        if not underpass:
            for dx in [-5.8,5.8]:m.box(x+dx,y-5,z+.6,.15,3,1.2,STEEL,True)
        self.c.model('destination_'+ident,m)

    def hill_town(self):
        main=smooth_path([(610,112,0),(700,150,0),(760,210,5),(710,280,12),
                          (610,330,19),(650,390,26),(790,400,34),(910,350,40),
                          (1020,390,48),(1040,480,56),(920,520,64),(780,490,70),
                          (670,540,77),(640,650,85),(760,710,94)])
        # The first spline segment can undershoot zero by a few centimeters.
        main=[(x,y,max(0,z)) for x,y,z in main]
        self.road('mountain_switchbacks',main,10,rails=True)
        self.road('ridge_lane',smooth_path([(760,710,94),(850,745,99),(960,705,103),(1030,620,95)]),5,rails=True,mark=False)
        self.road('valley_diagonal',smooth_path([(150,112,0),(230,180,0),(350,225,5),(470,205,5),(610,112,0)]),12)
        self.add_spot('hill_court','Hillside Court · 산동네 마당','terrace',790,425,36,'높이 36m 테라스, 주변 경사면과 낮은 주택')
        self.add_spot('ridge_lookout','Ridge Lookout · 능선 쉼터','terrace',785,732,96,'높이 96m 능선, 수목·난간과 절벽 방향')
        self.road('hill_court_access',[(790,400,34),(790,414,36),(790,425,36)],5,mark=False)
        self.road('lookout_access',[(760,710,94),(775,720,96),(785,732,96)],5,mark=False)
        decor=self.Mesh();count=0
        # Homes face the local road rather than a global X/Y grid.
        for idx in range(15,len(main)-10,7):
            a,p,b=main[idx-1],main[idx],main[idx+1]
            heading=math.atan2(b[1]-a[1],b[0]-a[0])
            for side in [-1,1]:
                x=p[0]-math.sin(heading)*side*23
                y=p[1]+math.cos(heading)*side*23
                z=p[2]+1.0
                if any(math.hypot(x-s['x'],y-s['y'])<32 for s in self.c.spots):continue
                if any(math.hypot(x-h['x'],y-h['y'])<25 for h in self.c.buildings):continue
                # Avoid other bends of the same switchback.
                if any(math.hypot(x-q[0],y-q[1])<15 for q in main):continue
                yaw=heading+(0 if side==1 else math.pi)
                self.pad(x,y,z,21,23,yaw)
                house=f'hillside_home_{count}'
                h=self.c.rng.choice([6,9])
                self.c.building(house,x,y,11,14,h,'house',z=z,yaw=yaw)
                roof=self.Mesh();roof.roof(12,15,h+.4,2.7,self.c.rng.choice([(.40,.25,.18),(.28,.32,.32)]))
                self.c.model(house+'_roof',roof,x,y,yaw=yaw,z=z)
                # Short sloping driveway and solid retaining wall anchor each house.
                self.road(house+'_drive',[(p[0],p[1],p[2]),
                          (p[0]+(x-p[0])*13.5/23,p[1]+(y-p[1])*13.5/23,z+.04)],3,mark=False)
                wall=self.Mesh();wall.box(0,0,-1.5,21,23,3,CONCRETE,True)
                self.c.model(house+'_retaining',wall,x,y,yaw=yaw,z=z)
                count+=1
        # Forest remains outside roads, homes and destination clearings.
        rng=self.c.rng
        for _ in range(1400):
            x,y=rng.uniform(-750,1400),rng.uniform(200,940)
            if any(math.hypot(x-rx,y-ry)<half+8 for rx,ry,_,half in self.road_samples):continue
            if any(math.hypot(x-px,y-py)<max(w,d)/2+7 for px,py,_,w,d,_ in self.pads):continue
            z=self.terrain_height(x,y)
            local=self.Mesh();self.tree(local,0,0,rng.uniform(5,11))
            # Bake tree geometry into one world mesh by translating its vertices/collisions.
            for color,(verts,tris) in local.groups.items():
                target_verts,target_tris=decor.groups.setdefault(color,([],[]));off=len(target_verts)
                target_verts.extend((vx+x,vy+y,vz+z) for vx,vy,vz in verts)
                target_tris.extend(off+i for i in tris)
            decor.collisions.extend((cx+x,cy+y,cz+z,w,d,h) for cx,cy,cz,w,d,h in local.collisions)
        self.c.model('mountain_forest',decor)

    def west_neighborhood(self):
        loop=smooth_path([(-230,35,0),(-290,170,0),(-440,225,0),(-570,190,0),(-650,80,0)])
        self.road('old_town_curved_street',loop,8)
        alleys=[smooth_path([(-290,170,0),(-320,115,0),(-360,90,0)]),
                smooth_path([(-440,225,0),(-450,175,0),(-480,120,0)]),
                smooth_path([(-570,190,0),(-530,135,0),(-480,120,0)])]
        for i,path in enumerate(alleys):self.road('old_town_alley_'+str(i),path,4,mark=False)
        for i in range(6,len(loop)-5,6):
            p=loop[i];a,b=loop[i-1],loop[i+1];angle=math.atan2(b[1]-a[1],b[0]-a[0])
            x,y=p[0]-math.sin(angle)*17,p[1]+math.cos(angle)*17
            if any(math.hypot(x-h['x'],y-h['y'])<24 for h in self.c.buildings):continue
            self.pad(x,y,0,22,22,angle)
            self.c.building('old_town_home_'+str(i),x,y,12,13,9,'brick',yaw=angle)
        self.add_spot('old_town_square','Old Town · 굽은 골목 광장','plaza',-490,85,0,'굽은 간선도로와 골목, 비정렬 저층 주택')
        self.road('old_town_square_access',[(-480,120,0),(-485,105,0),(-490,85,0)],5,mark=False)
        self.add_spot('south_river_park','South Bank · 강 건너 공원','park',500,-415,0,'맞은편 도심·고가도로, 강변 산책 공간')
        self.road('south_park_access',[(500,-470,0),(500,-440,0),(500,-415,0),
                                              (500,south_bank(500)-13,0)],5,mark=False)
        # A modest riverside settlement makes the far bank inhabited as well.
        for i,x in enumerate(range(-300,401,65)):
            y=-475-14*math.sin(i*.7)
            self.pad(x,y,0,28,27)
            self.c.building('south_bank_home_'+str(i),x,y,16,18,9 if i%3 else 15,'brick')
            boulevard=next(r['points'] for r in self.c.roads3d if r['id']=='south_bank_boulevard')
            join=min(boulevard,key=lambda p:(p[0]-x)**2+(p[1]-y-12)**2)
            self.road('south_home_access_'+str(i),[(x,y+12,0),join],4,mark=False)
        m=self.Mesh()
        for x in range(-400,1201,45):
            y=south_bank(x)-24
            self.tree(m,x,y,7)
        path=[(x,south_bank(x)-13,0) for x in range(-600,1301,12)]
        self.road('south_riverwalk',path,6,mark=False)
        self.c.model('south_bank_trees',m)


def extend_region(city,Mesh,tree,vehicle):
    region=Region(city,Mesh,tree,vehicle)
    region.expressways()
    region.west_neighborhood()
    region.hill_town()
    region.terrain()
    city.terrain_sampler=region.terrain_height
