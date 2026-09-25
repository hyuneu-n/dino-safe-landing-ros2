#!/usr/bin/env python3
"""Capture repeatable Classic sensor views from an already running metropolis.
Use ROS_DOMAIN_ID / GAZEBO_MASTER_URI to target an isolated validation server.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from capture_frame import capture

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--out',required=True)
p.add_argument('--manifest',required=True)
p.add_argument('--landmarks-only',action='store_true',help='Recheck only the four new landmarks and campus access')
a=p.parse_args()
out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
views=[('downtown',-185,-150,135,.45),('avenue',-150,-68,22,1.5708),
       ('industrial',166,-115,94,.38),('town',371,-120,95,.37),
       ('connected',100,-270,230,.55),
       ('interchange',540,-520,190,.5),('bridge',640,-680,105,1.2),
       ('mountain_town',550,260,180,.5),('old_town',-700,-60,150,.7),
       ('river_region',-150,-550,270,.4),
       ('logistics',-285,-145,55,.7),('mcdonalds',145,35,28,.9),
       ('subway',70,-122,12,1.4),('campus',225,210,145,.65)]
if a.landmarks_only:views=views[-4:]
for name,x,y,z,yaw in views:
    rgb,_=capture(x,y,z,settle_sec=5,timeout_sec=40,yaw=yaw,cam='chase')
    if rgb is None:raise RuntimeError('Missing RGB: '+name)
    Image.fromarray(rgb).save(out/(name+'.png'))
    print('[ok] '+name,flush=True)
manifest=json.loads(Path(a.manifest).read_text())
report=[]
for s in manifest['landing_spots']:
    if a.landmarks_only and s['id'] not in {'launch_depot','mcdonalds_delivery','subway_delivery','campus_delivery'}:continue
    rgb,depth=capture(s['x'],s['y'],s.get('z',0)+12,settle_sec=4,timeout_sec=35,cam='down')
    if rgb is None or depth is None:raise RuntimeError('Missing RGB/depth: '+s['id'])
    Image.fromarray(rgb).save(out/(s['id']+'_down.png'))
    finite=depth[np.isfinite(depth)]
    if len(finite)<depth.size*.5:raise RuntimeError('Invalid depth: '+s['id'])
    variation=float(np.percentile(finite,99)-np.percentile(finite,1))
    if variation < .3:raise RuntimeError('Obstacles missing from scan view: '+s['id'])
    record=dict(id=s['id'],finite_fraction=float(len(finite)/depth.size),depth_p99_p01=variation,
                depth_min=float(finite.min()),depth_max=float(finite.max()))
    report.append(record)
    print('[ok] '+json.dumps(record),flush=True)
(out/'sensor_report.json').write_text(json.dumps(report,indent=2))

# Spot checks of the actual asphalt surface catch terrain covering a curved road.
if manifest.get('roads3d') and not a.landmarks_only:
    road=next(r for r in manifest['roads3d'] if r['id']=='mountain_switchbacks')
    for i,fraction in enumerate([.15,.35,.55,.75,.9]):
        x,y,z=road['points'][int((len(road['points'])-1)*fraction)]
        rgb,depth=capture(x,y,z+12,settle_sec=4,timeout_sec=35,cam='down')
        if depth is None:raise RuntimeError('Missing hillside road depth')
        center=float(np.median(depth[118:123,158:163]))
        if not 11.45<center<12.2:raise RuntimeError(f'Road buried or missing at {x,y,z}: depth={center}')
        Image.fromarray(rgb).save(out/f'hill_road_{i}.png')
        print(f'[ok] hill road {i}: center depth {center:.3f}',flush=True)

# The campus terrace must leave its uphill entry exposed to the depth camera.
road=next(r for r in manifest['roads3d'] if r['id']=='campus_access')
for i,fraction in enumerate([.2,.35,.5]):
    x,y,z=road['points'][int((len(road['points'])-1)*fraction)]
    rgb,depth=capture(x,y,z+12,settle_sec=4,timeout_sec=35,cam='down')
    if depth is None:raise RuntimeError('Missing campus road depth')
    center=float(np.median(depth[118:123,158:163]))
    if not 11.45<center<12.2:raise RuntimeError(f'Campus road buried at {x,y,z}: depth={center}')
    Image.fromarray(rgb).save(out/f'campus_road_{i}.png')
    print(f'[ok] campus road {i}: center depth {center:.3f}',flush=True)
