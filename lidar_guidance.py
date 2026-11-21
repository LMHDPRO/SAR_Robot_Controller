"""Read-only scan and local-map guidance. Distances are not collision clearance."""
import math
import numpy as np


def sector_ranges(points):
    """Median projected distance of returns near each robot-frame axis, metres."""
    points=np.asarray(points,dtype=float).reshape(-1,2)
    points=points[np.all(np.isfinite(points),axis=1)]
    result={}
    for name,axis in (('frente',(1.,0.)),('izquierda',(0.,1.)),('atrás',(-1.,0.)),('derecha',(0.,-1.))):
        direction=np.asarray(axis)
        forward=points@direction
        lateral=np.abs(points[:,0]*direction[1]-points[:,1]*direction[0])
        selected=forward[(forward>0)&(lateral<=forward*math.tan(math.radians(25)))]
        result[name]=float(np.median(selected)) if len(selected)>=3 else None
    return result


def checkpoint_delta(current,reference):
    """Vector to a saved local pose expressed in the current robot frame."""
    current=np.asarray(current,dtype=float);reference=np.asarray(reference,dtype=float)
    if current.shape!=(3,) or reference.shape!=(3,) or not np.all(np.isfinite(current)) or not np.all(np.isfinite(reference)):
        raise ValueError('Pose inválida')
    dx,dy=reference[:2]-current[:2];c=math.cos(current[2]);s=math.sin(current[2])
    yaw=(reference[2]-current[2]+math.pi)%(2*math.pi)-math.pi
    return c*dx+s*dy,-s*dx+c*dy,math.degrees(yaw)


def motor_spin(telemetry,geometry):
    left,right=telemetry.get('pl'),telemetry.get('pr')
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in (left,right)):return False
    if geometry.get('swap_lr',0):left,right=right,left
    left*=geometry.get('motor_l',1);right*=geometry.get('motor_r',1)
    return left*right<0


def spin_during_scan(history,start,end):
    samples=[entry for entry in history if start-.15<=entry[0]<=end+.15]
    return bool(len(samples)>=2 and samples[0][0]<=start+.15 and samples[-1][0]>=end-.15
                and all(entry[2] for entry in samples))
