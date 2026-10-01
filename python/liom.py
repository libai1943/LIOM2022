"""Chapter 5, Algorithm 5-1: corridor reconstruction and quadratic softening.

The supplied data are a protected HA* initializer's output, not an optimized answer.
The numerical sums follow the author's supplied 200-node AMPL implementation.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import os
import numpy as np
import casadi as ca

FIELDS = ('x', 'y', 'theta', 'v', 'a', 'phy', 'w', 'xf', 'yf', 'xr', 'yr')


@dataclass
class Problem:
    p: np.ndarray
    reference: np.ndarray
    grid: np.ndarray
    obstacles: np.ndarray

    @classmethod
    def load(cls, directory):
        directory = Path(directory)
        arrays = [np.loadtxt(directory/name, delimiter=',', ndmin=2) for name in
                  ('parameters.csv', 'reference.csv', 'dilated_map.csv', 'obstacles.csv')]
        p, ref, grid, obs = arrays
        p = p.ravel()
        if p.size != 28 or ref.shape != (int(p[0]), 11) or obs.shape[1] != 3:
            raise ValueError('Unexpected input dimensions; see data/README.md.')
        if not all(np.isfinite(a).all() for a in arrays):
            raise ValueError('Inputs must contain finite numbers.')
        if not np.isin(grid, [0, 1]).all():
            raise ValueError('The dilated map must contain only 0 and 1.')
        return cls(p, ref, grid.astype(bool), obs)

    @property
    def n(self): return int(self.p[0])

    @property
    def radius(self): return math.hypot(sum(self.p[5:8])/4, self.p[8]/2)

    @property
    def offsets(self):
        length = sum(self.p[5:8])
        return 0.75*length-self.p[7], 0.25*length-self.p[7]

    def initial_vector(self):
        return np.r_[self.reference.ravel(order='F'), self.p[27]]


class Corridors:
    """The original alternating up/left/down/right AABB expansion."""
    def __init__(self, problem):
        self.problem = problem
        self.p = problem.p
        self.dx = (self.p[2]-self.p[1])/problem.grid.shape[0]
        self.dy = (self.p[4]-self.p[3])/problem.grid.shape[1]

    def points_free(self, x, y):
        ix = np.floor((np.asarray(x)-self.p[1])/self.dx).astype(int)
        iy = np.floor((np.asarray(y)-self.p[3])/self.dy).astype(int)
        if np.any(ix < 0) or np.any(ix >= self.problem.grid.shape[0]) or np.any(iy < 0) or np.any(iy >= self.problem.grid.shape[1]):
            return False
        return not np.any(self.problem.grid[ix, iy])

    def seed(self, x, y):
        if self.points_free(x, y): return x, y
        for i in range(1, 100000):
            angle_type = i % 8
            radius = 1+(i-angle_type)/8
            angle = angle_type*math.pi/4
            xx = x+math.cos(angle)*radius*self.dx/2
            yy = y+math.sin(angle)*radius*self.dx/2
            if self.points_free(xx, yy): return xx, yy
        raise RuntimeError('No free corridor seed was found.')

    def strip_free(self, rect):
        xmin, xmax, ymin, ymax = rect
        radius = self.problem.radius
        if xmin < self.p[1]+radius or xmax > self.p[2]-radius or ymin < self.p[3]+radius or ymax > self.p[4]-radius:
            return False
        ds = self.dx/2
        xs = np.linspace(xmin, xmax, math.ceil((xmax-xmin)/ds)+1)
        ys = np.linspace(ymin, ymax, math.ceil((ymax-ymin)/ds)+1)
        xx, yy = np.meshgrid(xs, ys, indexing='ij')
        return self.points_free(xx, yy)

    def box(self, x, y):
        x, y = self.seed(x, y)
        lengths = np.zeros(4)
        done = np.zeros(4, dtype=bool)
        while not done.all():
            for direction in range(4):
                if done[direction]: continue
                trial = lengths.copy()
                trial[direction] += self.p[13]
                if trial[direction] > self.p[14]:
                    done[direction] = True
                    continue
                u, l, d, r = lengths
                strips = [(x-l,x+r,y+u,y+trial[0]), (x-trial[1],x-l,y-d,y+u),
                          (x-l,x+r,y-trial[2],y-d), (x+r,x+trial[3],y-d,y+u)]
                if self.strip_free(strips[direction]): lengths = trial
                else: done[direction] = True
        u, l, d, r = lengths
        return [x-l, x+r, y-d, y+u]

    def build(self, vector):
        states = vector[:-1].reshape((self.problem.n, 11), order='F')
        x, y, theta = states[:, 0], states[:, 1], states[:, 2]
        boxes = []
        for offset in self.problem.offsets:
            boxes.append(np.array([self.box(xx, yy) for xx, yy in
                         zip(x+offset*np.cos(theta), y+offset*np.sin(theta))]))
        return boxes  # front, rear


def bounds(problem, front, rear):
    p, n = problem.p, problem.n
    low, high = np.full((n,11), -np.inf), np.full((n,11), np.inf)
    for j, limit in [(3,p[9]),(4,p[10]),(5,p[11]),(6,p[12])]:
        low[:,j], high[:,j] = -limit, limit
    for j in range(3):
        low[0,j] = high[0,j] = p[21+j]
        low[-1,j] = high[-1,j] = p[24+j]
    for j in [3,5]:
        low[[0,-1],j] = high[[0,-1],j] = 0
    for j in [4,6]: low[-1,j] = high[-1,j] = 0
    for j, boxes in [(7,front),(9,rear)]:
        low[:,j],high[:,j] = boxes[:,0],boxes[:,1]
        low[:,j+1],high[:,j+1] = boxes[:,2],boxes[:,3]
    for row, base in [(0,21),(-1,24)]:
        for j, offset in [(7,problem.offsets[0]),(9,problem.offsets[1])]:
            xy = p[base:base+2]+offset*np.array([math.cos(p[base+2]),math.sin(p[base+2])])
            if np.any(xy < low[row,j:j+2]-1e-9) or np.any(xy > high[row,j:j+2]+1e-9):
                raise ValueError('An endpoint disc lies outside its corridor.')
            low[row,j:j+2] = high[row,j:j+2] = xy
    return np.r_[low.ravel(order='F'),0.1], np.r_[high.ravel(order='F'),n*0.5]


def build_solver(problem, linear_solver='ma27', hsl_library=None):
    p, n = problem.p, problem.n
    z = ca.SX.sym('z',n*11+1)
    s = ca.reshape(z[:-1],n,11)
    x,y,theta,v,a,phi,w,xf,yf,xr,yr = [s[:,j] for j in range(11)]
    tf, weight = z[-1], ca.SX.sym('weight')
    dt = tf/(n-1)
    f2p,r2p = problem.offsets
    residual = ca.vertcat(x[1:]-x[:-1]-dt*v[:-1]*ca.cos(theta[:-1]),
        y[1:]-y[:-1]-dt*v[:-1]*ca.sin(theta[:-1]),v[1:]-v[:-1]-dt*a[:-1],
        theta[1:]-theta[:-1]-dt*v[:-1]*ca.tan(phi[:-1])/p[5],
        phi[1:]-phi[:-1]-dt*w[:-1],
        xf[1:]-x[1:]-f2p*ca.cos(theta[1:]),yf[1:]-y[1:]-f2p*ca.sin(theta[1:]),
        xr[1:]-x[1:]-r2p*ca.cos(theta[1:]),yr[1:]-y[1:]-r2p*ca.sin(theta[1:]))
    infeasibility = ca.sumsqr(residual)
    cost = tf+p[19]*ca.sumsqr(a)+p[20]*ca.sumsqr(w)
    options = {'print_time':False, 'ipopt.print_level':0,'ipopt.tol':1e-7,
               'ipopt.max_iter':9000,'ipopt.max_cpu_time':120.,
               'ipopt.mu_strategy':'adaptive','ipopt.linear_solver':linear_solver}
    if hsl_library:
        library = Path(hsl_library).resolve(strict=True)
        # An ASCII library name plus a Unicode Windows PATH also supports Chinese folders.
        options['ipopt.hsllib'] = library.name if os.name == 'nt' else str(library)
        if os.name == 'nt': os.environ['PATH'] = str(library.parent)+os.pathsep+os.environ['PATH']
    solver = ca.nlpsol('liom','ipopt',{'x':z,'p':weight,'f':cost+weight*infeasibility},options)
    evaluate = ca.Function('metrics',[z],[cost,infeasibility,residual])
    return solver, evaluate


def validate(problem, z, front, rear, infeasibility, difference):
    lb, ub = bounds(problem, front, rear)
    violation = max(0.,float(np.max(lb-z)),float(np.max(z-ub)))
    if infeasibility >= problem.p[15] or difference >= problem.p[16] or violation > 1e-5:
        raise RuntimeError('Discrete convergence/boundary/corridor validation failed.')
    states = z[:-1].reshape((problem.n,11),order='F')
    centres = np.vstack([states[:,:2]+offset*np.c_[np.cos(states[:,2]),np.sin(states[:,2])]
                         for offset in problem.offsets])
    clearance = np.inf
    # Convex source obstacles: signed-edge test plus point-to-segment distance.
    for obstacle_id in np.unique(problem.obstacles[:,0]):
        vertices = problem.obstacles[problem.obstacles[:,0]==obstacle_id,1:]
        signs, distances = [], []
        for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
            edge = b-a
            if edge@edge == 0: continue
            d = centres-a
            signs.append(edge[0]*d[:,1]-edge[1]*d[:,0])
            t = np.clip(d@edge/(edge@edge),0,1)
            distances.append(np.linalg.norm(d-t[:,None]*edge,axis=1))
        cross = np.array(signs)
        if np.any(np.all(cross>=-1e-12,axis=0)|np.all(cross<=1e-12,axis=0)):
            raise RuntimeError('A disc centre intersects an obstacle at a configuration point.')
        clearance = min(clearance,np.min(distances)-problem.radius)
    if clearance < -1e-5: raise RuntimeError('Discrete covering-disc clearance is negative.')
    return {'max_bound_violation':violation,'min_disc_clearance':float(clearance)}


def solve(problem, linear_solver='ma27', hsl_library=None, max_iterations=20):
    solver, evaluate = build_solver(problem,linear_solver,hsl_library)
    corridors, previous, weight = Corridors(problem), problem.initial_vector(), problem.p[17]
    history, statistics = [], []
    for iteration in range(1,max_iterations+1):
        front,rear = corridors.build(previous)
        lb,ub = bounds(problem,front,rear)
        answer = solver(x0=previous,lbx=lb,ubx=ub,p=weight)
        status = solver.stats()
        if not status['success']:
            raise RuntimeError(f"IPOPT failed: {status['return_status']}. Check the requested linear solver/HSL installation.")
        current = np.asarray(answer['x']).ravel()
        cost, infeasibility, _ = evaluate(current)
        cost, infeasibility = float(cost),float(infeasibility)
        difference = float(np.linalg.norm(current-previous))
        statistics.append([iteration,weight,cost,infeasibility,difference])
        history.append(current.copy())
        print(f'LIOM {iteration:2d} | weight {weight:.3g} | J {cost:.7f} | infeasibility {infeasibility:.3g} | difference {difference:.4g}',flush=True)
        if infeasibility < problem.p[15] and difference < problem.p[16]:
            report = validate(problem,current,front,rear,infeasibility,difference)
            return current,np.array(statistics),history,report
        previous, weight = current,weight*problem.p[18]
    raise RuntimeError('Both stopping criteria were not met. No converged result is claimed.')


def save_results(problem, solution, statistics, history, directory):
    directory = Path(directory); directory.mkdir(parents=True,exist_ok=True)
    states = solution[:-1].reshape((problem.n,11),order='F')
    np.savetxt(directory/'trajectory.csv',states,delimiter=',',header=','.join(FIELDS),comments='')
    np.savetxt(directory/'iterations.csv',statistics,delimiter=',',header='iteration,weight,cost,infeasibility,difference',comments='')
    np.savetxt(directory/'summary.csv',[[solution[-1],statistics[-1,2],statistics[-1,3],statistics[-1,4],len(history)]],
               delimiter=',',header='terminal_time,cost,infeasibility,difference,iterations',comments='')
    curves = [np.c_[np.full(problem.n,i+1),z[:-1].reshape((problem.n,11),order='F')[:,:3]] for i,z in enumerate(history)]
    np.savetxt(directory/'intermediate.csv',np.vstack(curves),delimiter=',',header='iteration,x,y,theta',comments='')

