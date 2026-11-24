"""Occupancy grid: células fechadas, origem SE(2), desconhecido bloqueado."""
from pathlib import Path
import hashlib
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, label

ROOT = Path(__file__).resolve().parents[1]

class Grid:
    def __init__(self, cells, resolution, origin=(0, 0, 0), source='synthetic'):
        self.cells = np.asarray(cells, dtype=np.int8)  # -1 unknown, 0 free, 1 occupied; row 0 = bottom
        self.base_cells = self.cells.copy()
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        c, s = np.cos(self.origin[2]), np.sin(self.origin[2])
        self.rotation = np.array([[c, -s], [s, c]])
        self.source = source
        self.refresh()

    def refresh(self):
        self.blocked = self.cells != 0
        # Pad treats outside as blocked too. Subtract cell diagonal for conservative clearance.
        padded = np.pad(~self.blocked, 1)
        self.clearance = np.maximum(0, distance_transform_edt(padded)[1:-1, 1:-1]*self.resolution-self.resolution*np.sqrt(2))
        self.components, self.n_components = label(self.clearance > .52)

    @classmethod
    def load(cls, path):
        path = Path(path)
        meta = yaml.safe_load(path.read_text())
        if meta.get('mode', 'trinary') != 'trinary':
            raise ValueError('Somente mapa trinary é suportado explicitamente')
        pixels = np.asarray(Image.open(path.parent / meta['image']).convert('L'))[::-1]
        p = pixels/255. if meta.get('negate', 0) else 1-pixels/255.
        cells = np.full(p.shape, -1, np.int8)
        cells[p < meta['free_thresh']] = 0
        cells[p > meta['occupied_thresh']] = 1
        return cls(cells, meta['resolution'], meta['origin'], str(path))

    def world(self, local):
        return np.asarray(local) @ self.rotation.T + self.origin[:2]

    def local(self, world):
        return (np.asarray(world)-self.origin[:2]) @ self.rotation

    def indices(self, world):
        return np.floor(self.local(world)/self.resolution).astype(int)

    def hits(self, world):
        ij = self.indices(world)
        x, y = ij[..., 0], ij[..., 1]
        valid = (x >= 0)&(y >= 0)&(x < self.cells.shape[1])&(y < self.cells.shape[0])
        return ~valid | self.blocked[np.clip(y, 0, self.cells.shape[0]-1), np.clip(x, 0, self.cells.shape[1]-1)]

    def distance(self, point):
        x, y = self.indices(point)
        if x < 0 or y < 0 or x >= self.cells.shape[1] or y >= self.cells.shape[0]: return 0.
        return float(self.clearance[y, x])

    def sample_pair(self, rng, minimum=.8, maximum=7.):
        sizes = np.bincount(self.components.ravel()); sizes[0] = 0
        component = int(np.argmax(sizes))
        candidates = np.argwhere(self.components == component)
        if not sizes[component]: raise ValueError('Mapa sem área segura para o footprint')
        for _ in range(2000):
            a, b = candidates[rng.integers(len(candidates), size=2)]
            start, goal = self.world((np.array([a[::-1], b[::-1]])+.5)*self.resolution)
            if minimum < np.linalg.norm(goal-start) < maximum: return start, goal
        raise RuntimeError('Não foi possível amostrar destino conectado')

    def add_box(self, center, size):
        # Synthetic axis-aligned in grid frame; never changes imported asset.
        cx, cy = self.local(center)
        w, h = size
        x0, y0 = np.floor((np.array([cx-w/2, cy-h/2]))/self.resolution).astype(int)
        x1, y1 = np.ceil((np.array([cx+w/2, cy+h/2]))/self.resolution).astype(int)
        self.cells[max(0,y0):min(self.cells.shape[0],y1), max(0,x0):min(self.cells.shape[1],x1)] = 1

    @classmethod
    def synthetic(cls, rng, variant=False):
        width, height = rng.uniform(6, 10), rng.uniform(5, 8)
        cells = np.zeros((int(height/.05), int(width/.05)), np.int8)
        cells[[0,-1],:] = 1; cells[:,[0,-1]] = 1
        grid = cls(cells, .05)
        if variant:
            # Unseen synthetic layouts; no claim these are physical arena walls.
            for _ in range(3):
                grid.add_box(rng.uniform([1,1], [width-1,height-1]), rng.uniform([.3,.3],[1.2,1.2]))
            grid.refresh()
        return grid


def inventory(folder):
    rows = []
    for path in sorted(Path(folder).rglob('*.yaml')):
        try:
            meta = yaml.safe_load(path.read_text())
            if not isinstance(meta, dict) or 'image' not in meta: continue
            image = path.parent / meta['image']
            row = dict(yaml=str(path), image=str(image), image_exists=image.exists(), metadata=meta)
            if image.exists():
                grid = Grid.load(path)
                row.update(shape=list(grid.cells.shape), occupied=int((grid.cells==1).sum()),
                           free=int((grid.cells==0).sum()), unknown=int((grid.cells==-1).sum()),
                           sha256=hashlib.sha256(image.read_bytes()).hexdigest())
            rows.append(row)
        except (ValueError, OSError, KeyError) as exc:
            rows.append(dict(yaml=str(path), error=str(exc)))
    return rows
