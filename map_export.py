"""Portable local point-map archive and an image, not an occupancy grid."""
import json
from pathlib import Path
import numpy as np


def save_map(path, result, metadata):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    path=Path(path).with_suffix('.npz')
    image_path=path.with_suffix('.png')
    archive_tmp=path.with_suffix('.npz.tmp')
    image_tmp=path.with_suffix('.png.tmp')
    points=result['map'];trajectory=result['path']
    figure=Figure(figsize=(9,8),dpi=150);FigureCanvasAgg(figure)
    axes=figure.add_subplot(111)
    observed=np.asarray(result.get('observed_cells',[]),dtype=int).reshape(-1,2)
    resolution=float(result.get('observed_resolution_m',.1))
    if len(observed):
        low=observed.min(axis=0);high=observed.max(axis=0)
        image=np.zeros((high[1]-low[1]+1,high[0]-low[0]+1,4),dtype=np.uint8)
        image[observed[:,1]-low[1],observed[:,0]-low[0]]=[190,215,220,150]
        axes.imshow(image,origin='lower',interpolation='nearest',zorder=0,
                    extent=(low[0]*resolution,(high[0]+1)*resolution,low[1]*resolution,(high[1]+1)*resolution))
    axes.scatter(points[:,0],points[:,1],s=2,c='#173f5f',label='Measured surfaces')
    if len(trajectory):axes.plot(trajectory[:,0],trajectory[:,1],color='#d47300',label='Robot centre')
    axes.set_aspect('equal');axes.set_xlabel('X (m)');axes.set_ylabel('Y (m)')
    axes.set_title('Local LiDAR point map · '+metadata['source'])
    axes.grid(True,alpha=.3);axes.legend();figure.tight_layout()
    try:
        with archive_tmp.open('wb') as output:
            np.savez_compressed(output,map_xy_m=points,trajectory_xy_yaw=trajectory,
                                mount_xy_yaw_deg=metadata['mount_xy_yaw_deg'],source=metadata['source'],
                                observed_cells_xy=observed,observed_resolution_m=resolution,
                                metadata_json=json.dumps(metadata,ensure_ascii=False))
        figure.savefig(image_tmp,format='png')
        archive_tmp.replace(path)
        image_tmp.replace(image_path)
    finally:
        archive_tmp.unlink(missing_ok=True);image_tmp.unlink(missing_ok=True)
    return path,image_path
