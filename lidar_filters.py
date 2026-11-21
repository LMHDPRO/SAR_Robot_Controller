"""Self-return masks in the robot frame. Excluded beams remain unknown."""
import numpy as np
from coind6.scans import cartesian


def exclusion_mask(robot, origin, body_margin=.02, sector_center=180., sector_width=0.):
    """Return the same chassis and angular exclusions used by the mapper."""
    robot=np.asarray(robot,dtype=float).reshape(-1,2)
    local=robot-np.asarray(origin,dtype=float)
    inside=np.all(np.abs(robot)<=.115+body_margin,axis=1)
    bearings=np.degrees(np.arctan2(local[:,1],local[:,0]))
    delta=(bearings-sector_center+180)%360-180
    sector=(np.abs(delta)<=sector_width/2) if sector_width>0 else np.zeros(len(robot),dtype=bool)
    return inside|sector


def partition_scan(points, mount, body_margin=.02, sector_center=180., sector_width=0.):
    """Exclude a 23x23 cm footprint and optional angular sector at the LiDAR.

    Sector angles are calibrated robot directions: 0 forward, +90 left.
    Width zero disables the sector; 360 masks everything. Margin is metres.
    """
    x,y,angle,minimum,maximum=mount
    coords=np.asarray(cartesian(points,angle,True,minimum,maximum)).reshape(-1,3)
    local=coords[:,:2]
    robot=local+(x,y)
    excluded=exclusion_mask(robot,(x,y),body_margin,sector_center,sector_width)
    return robot[~excluded],robot[excluded]


def filter_scan(points, mount, body_margin=.02, sector_center=180., sector_width=0.):
    usable,excluded=partition_scan(points,mount,body_margin,sector_center,sector_width)
    return usable,len(excluded)
