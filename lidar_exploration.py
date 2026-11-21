"""Display-only observed-ray cells; not a navigation or collision map."""
import numpy as np


class ExplorationGrid:
    def __init__(self, resolution=.1, extent=20.):
        self.resolution=resolution;self.extent=extent
        self.observed=set();self.hits=set()

    def update(self,origin,endpoints):
        origin=np.asarray(origin,dtype=float);points=np.asarray(endpoints,dtype=float).reshape(-1,2)
        points=points[np.all(np.isfinite(points),axis=1)]
        points=points[np.all(np.abs(points)<=self.extent,axis=1)]
        if not len(points) or not np.all(np.isfinite(origin)) or np.any(np.abs(origin)>self.extent):return
        # A half-cell step avoids gaps. Never infer rays for absent readings.
        hit_cells=np.floor(points/self.resolution).astype(int)
        self.hits.update(map(tuple,hit_cells));self.observed.difference_update(self.hits)
        delta=points-origin
        counts=np.maximum(1,np.ceil(np.linalg.norm(delta,axis=1)/(self.resolution*.5)).astype(int))
        maximum=int(counts.max())
        active=np.ones(len(points),dtype=bool)
        for step in range(maximum):
            valid=np.flatnonzero(active & (step<counts))
            if not len(valid):break
            samples=origin+delta[valid]*(step/counts[valid])[:,None]
            cells=np.floor(samples/self.resolution).astype(int)
            for index,cell in zip(valid,map(tuple,cells)):
                if cell in self.hits:active[index]=False
                else:self.observed.add(cell)

    def image(self):
        if not self.observed:return None,None
        cells=np.asarray(list(self.observed),dtype=int);low=cells.min(axis=0);high=cells.max(axis=0)
        image=np.zeros((high[1]-low[1]+1,high[0]-low[0]+1,4),dtype=np.uint8)
        image[cells[:,1]-low[1],cells[:,0]-low[0]]=[55,95,107,120]
        return image,(low[0]*self.resolution,low[1]*self.resolution,image.shape[1]*self.resolution,image.shape[0]*self.resolution)
