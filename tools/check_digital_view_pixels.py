#!/usr/bin/env python3
"""Check actual 1024x600 LVGL test renders: visible dots and equal I/Q axes."""
import sys,json
from pathlib import Path
root=Path(sys.argv[1]);results={}
for name,expected in [('iq',4),('bpsk',2)]:
    data=(root/(name+'.ppm')).read_bytes()
    header=b'P6\n1024 600\n255\n';assert data.startswith(header)
    rgb=data[len(header):];assert len(rgb)==1024*600*3
    # The lower plot's exact bright-green trace color, excluding the sidebar.
    points={(x,y) for y in range(246,473) for x in range(14,766)
            if rgb[(y*1024+x)*3:(y*1024+x)*3+3]==bytes((124,255,154))}
    clusters=[]
    while points:
        seed=points.pop();todo=[seed];cluster=[seed]
        while todo:
            x,y=todo.pop()
            for n in [(x-1,y),(x+1,y),(x,y-1),(x,y+1)]:
                if n in points:points.remove(n);todo.append(n);cluster.append(n)
        clusters.append(cluster)
    assert len(clusters)==expected,(name,len(clusters))
    assert all(len(c)>=5 for c in clusters),(name,[len(c) for c in clusters])
    centers=[(sum(x for x,y in c)/len(c),sum(y for x,y in c)/len(c)) for c in clusters]
    dx=max(x for x,y in centers)-min(x for x,y in centers)
    dy=max(y for x,y in centers)-min(y for x,y in centers)
    assert dx>100
    if name=='iq':assert abs(dx-dy)<=2,(dx,dy)
    else:assert dy<=1,dy
    results[name]={'visible_clusters':len(clusters),'centers':sorted(centers),'dx_px':dx,'dy_px':dy}
(root/'digital_pixel_validation.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results))
