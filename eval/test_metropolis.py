"""World geometry invariants, including rotated hillside homes and sloping roads."""
import contextlib
import io
import math
import sys
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'worlds'))
from generate_metropolis import generate


def overlap(a,b):
    """Separating-axis test for oriented rectangles (x,y,w,d,yaw)."""
    def corners(rect):
        x,y,w,d,t=rect
        return [(x+u*math.cos(t)-v*math.sin(t),y+u*math.sin(t)+v*math.cos(t))
                for u,v in [(-w/2,-d/2),(w/2,-d/2),(w/2,d/2),(-w/2,d/2)]]
    ca,cb=corners(a),corners(b)
    for t in [a[4],a[4]+math.pi/2,b[4],b[4]+math.pi/2]:
        pa=[x*math.cos(t)+y*math.sin(t) for x,y in ca]
        pb=[x*math.cos(t)+y*math.sin(t) for x,y in cb]
        if max(pa)<=min(pb)+1e-5 or max(pb)<=min(pa)+1e-5:return False
    return True


class CityLayoutTest(unittest.TestCase):
    def test_layout_geometry_and_destinations(self):
        for seed in [0,7,23]:
            with self.subTest(seed=seed),tempfile.TemporaryDirectory() as tmp:
                with contextlib.redirect_stdout(io.StringIO()):
                    city,data=generate(Path(tmp)/'city.world',seed)
                self.assertEqual(len(data['landing_spots']),18)
                self.assertEqual(sum(s['nominal_target_blocked'] for s in data['landing_spots']),4)
                for s in data['landing_spots']:
                    if s['kind']=='elevated':continue
                    ax,ay=city.audit_clearings[s['id']]
                    for dx,dy in [(0,0),(-2,-2),(-2,2),(2,-2),(2,2)]:
                        self.assertLess(abs(city.terrain_sampler(ax+dx,ay+dy)-s.get('z',0)),.15,s['id'])
                buildings=data['buildings']
                for i,a in enumerate(buildings):
                    for b in buildings[i+1:]:
                        self.assertFalse(overlap(tuple(a.get(k,0) for k in ['x','y','w','d','yaw']),
                                                 tuple(b.get(k,0) for k in ['x','y','w','d','yaw'])),(a['name'],b['name']))
                for road in data['roads3d']:
                    points=road['points']
                    self.assertGreater(len(points),1)
                    for a,b in zip(points,points[1:]):
                        length=math.hypot(b[0]-a[0],b[1]-a[1])
                        self.assertGreater(length,.01,road['id'])
                        self.assertLessEqual(abs(b[2]-a[2])/length,.16,road['id'])
                    self.assertTrue(all(math.isfinite(v) for p in points for v in p))
                root=ET.parse(city.out)
                names=[m.get('name') for m in root.findall('.//world/model')]
                self.assertEqual(len(names),len(set(names)))
                for uri in root.findall('.//mesh/uri'):
                    self.assertTrue(Path(uri.text.removeprefix('file://')).is_file(),uri.text)
                # Terrain/road collision meshes must reference their exact visual mesh.
                for model in root.findall('.//world/model'):
                    mesh= model.findtext('./link/collision/geometry/mesh/uri')
                    if mesh:self.assertEqual(mesh,model.findtext('./link/visual/geometry/mesh/uri'))
                    pose=list(map(float,model.findtext('pose','0 0 0 0 0 0').split()))
                    for col in model.findall('./link/collision'):
                        size=col.findtext('./geometry/box/size')
                        if size is None:
                            radius=col.findtext('./geometry/cylinder/radius')
                            if radius is None:continue
                            w=d=2*float(radius);h=float(col.findtext('./geometry/cylinder/length'))
                        else:w,d,h=map(float,size.split())
                        cp=list(map(float,col.findtext('pose','0 0 0 0 0 0').split()))
                        x=pose[0]+cp[0]*math.cos(pose[5])-cp[1]*math.sin(pose[5])
                        y=pose[1]+cp[0]*math.sin(pose[5])+cp[1]*math.cos(pose[5])
                        z=pose[2]+cp[2]
                        for s in data['landing_spots']:
                            level=s.get('z',0)
                            # Underpasses have overhead structures; audit the landing envelope.
                            if z+h/2<level+.45 or z-h/2>level+3:continue
                            ax,ay=city.audit_clearings[s['id']]
                            self.assertFalse(overlap((x,y,w,d,pose[5]+cp[5]),(ax,ay,4,4,0)),
                                             (seed,s['id'],model.get('name'),x,y,z))


if __name__=='__main__':unittest.main()
