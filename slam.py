"""Bounded local 2-D scan-to-map SLAM. Metres, radians, X forward, Y left.

No loop closure, global optimization or motion control. Rejected scans never
modify the map. Odometry is an initial guess, not a replacement for matching.
"""
import math
import numpy as np


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


def relative_prediction(pose, old, new):
    delta = transform(np.array([new[:2] - old[:2]]), (0, 0, -old[2]))[0]
    xy = transform(np.array([delta]), pose)[0]
    return np.array([*xy, pose[2] + new[2] - old[2]])


def voxel(points, size=.05, limit=40000):
    if not len(points):
        return np.empty((0, 2))
    _, indices = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(indices)[-limit:]]


def nearest(points, target):
    # Chunking bounds memory without adding a SciPy dependency.
    ids, distances = [], []
    for chunk in np.array_split(points, max(1, math.ceil(len(points) / 64))):
        d = np.sum((chunk[:, None] - target[None, :]) ** 2, axis=2)
        ix = np.argmin(d, axis=1)
        ids.extend(ix)
        distances.extend(d[np.arange(len(chunk)), ix])
    return np.asarray(ids), np.sqrt(distances)


class LocalSLAM:
    def __init__(self):
        self.pose = np.zeros(3)
        self.map = np.empty((0, 2))
        self.path = []
        self.odom = None
        self.last_time = None

    def update(self, points, timestamp, odom=None, rotation_only=False):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        points = voxel(points[np.all(np.isfinite(points), axis=1)], .04, 10000)
        points = points[::max(1, math.ceil(len(points)/600))]
        if len(points) < 40:
            return self.result(False, 'Too few valid points', points)
        recovering=self.last_time is not None and timestamp-self.last_time>1.0
        if recovering:
            gap=timestamp-self.last_time
            if odom is None or self.odom is None:
                return self.result(False, 'Localización perdida: falta odometría para recuperar el mapa', points)
            delta=np.asarray(odom)-self.odom
            if np.linalg.norm(delta[:2])>.5:
                return self.result(False, 'Localización perdida: desplazamiento demasiado grande para recuperar el mapa', points)
        prediction = self.pose.copy()
        if odom is not None and self.odom is not None:
            prediction = relative_prediction(self.pose, self.odom, np.asarray(odom))
        else:rotation_only=False
        odom_prediction=prediction.copy()
        anchored_xy=prediction[:2].copy()
        if len(self.map):
            target = self.map[np.linalg.norm(self.map - prediction[:2], axis=1) < 13]
            if len(target) < 40:
                return self.result(False, 'No map overlap', points)
            target = target[::max(1, math.ceil(len(target)/3000))]
            for _ in range(18):
                world = transform(points, prediction)
                ids, distances = nearest(world, target)
                keep = distances < .30
                if keep.sum() < 30:
                    return self.result(False, 'Insufficient correspondences', points)
                threshold = np.quantile(distances[keep], .85)
                keep &= distances <= threshold
                a, b = world[keep], target[ids[keep]]
                ac, bc = a.mean(axis=0), b.mean(axis=0)
                u, singular, vt = np.linalg.svd((a-ac).T @ (b-bc))
                if singular[-1] < singular[0] * .005:
                    return self.result(False, 'Degenerate geometry (wall/line)', points)
                if rotation_only:
                    # Fit the rotation around the predicted axle centre directly;
                    # unconstrained ICP translation is unreliable in a partial scan.
                    aa=a-anchored_xy;bb=b-anchored_xy
                    angle=math.atan2(np.sum(aa[:,0]*bb[:,1]-aa[:,1]*bb[:,0]),np.sum(aa*bb))
                    shift=np.zeros(2)
                else:
                    r = vt.T @ u.T
                    if np.linalg.det(r) < 0:
                        vt[-1] *= -1
                        r = vt.T @ u.T
                    shift = bc - r @ ac
                    angle = math.atan2(r[1, 0], r[0, 0])
                    prediction[:2] = r @ prediction[:2] + shift
                prediction[2] += angle
                if np.linalg.norm(shift) < .0005 and abs(angle) < .0005:
                    break
            _, distances = nearest(transform(points, prediction), target)
            keep = distances < .15
            ratio = keep.mean()
            error = np.sqrt(np.mean(distances[keep]**2)) if keep.any() else float('inf')
            if rotation_only and keep.sum()>=30:
                world=transform(points,prediction);ids,_=nearest(world,target)
                bias=np.linalg.norm(np.mean(target[ids[keep]]-world[keep],axis=0))
                if bias>.04:
                    return self.result(False,'Giro: escaneo no coincide con el centro estimado',points)
            turn = (prediction[2]-self.pose[2]+math.pi) % (2*math.pi)-math.pi
            correction_turn=(prediction[2]-odom_prediction[2]+math.pi)%(2*math.pi)-math.pi
            if recovering and (np.linalg.norm(prediction[:2]-odom_prediction[:2])>.12 or abs(correction_turn)>math.radians(12)):
                return self.result(False, 'Recuperación rechazada: escaneo y odometría no coinciden', points)
            if ratio < .65 or error > .08 or np.linalg.norm(prediction[:2]-self.pose[:2]) > .5 or (not recovering and abs(turn) > math.radians(30)):
                return self.result(False, f'Rejected: overlap {ratio:.0%}, error {error:.3f} m', points)
            note = f'{"Mapa recuperado" if recovering else "Localización"} · coincidencia {ratio:.0%} · error {error:.3f} m'
        else:
            note = 'First scan: map origin'
        self.pose = prediction
        self.map = voxel(np.vstack((self.map, transform(points, self.pose))))
        self.path = (self.path + [self.pose.copy()])[-10000:]
        self.odom = None if odom is None else np.asarray(odom).copy()
        self.last_time = timestamp
        return self.result(True, note, points)

    def result(self, accepted, note, points):
        return dict(accepted=accepted, note=note, pose=self.pose.copy(),
                    points=transform(points, self.pose), map=self.map.copy(),
                    path=np.asarray(self.path).reshape(-1, 3))
