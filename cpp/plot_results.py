"""Static visualization of numeric results; no optimization or video generation."""
from pathlib import Path
import numpy as np


def draw(data_directory, output_directory, show=True):
    import matplotlib
    if not show: matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    data, out = Path(data_directory),Path(output_directory)
    p = np.loadtxt(data/'parameters.csv',delimiter=',')
    reference = np.loadtxt(data/'reference.csv',delimiter=',')
    obstacles = np.loadtxt(data/'obstacles.csv',delimiter=',')
    r = np.loadtxt(out/'trajectory.csv',delimiter=',',skiprows=1)
    history = np.loadtxt(out/'intermediate.csv',delimiter=',',skiprows=1)
    tf,cost,infeasibility,difference,count = np.loadtxt(out/'summary.csv',delimiter=',',skiprows=1)
    for name in ['LIOM trajectory','LIOM profiles']:
        if plt.fignum_exists(name): plt.close(name)
    fig,ax = plt.subplots(num='LIOM trajectory',figsize=(8,8),layout='constrained')
    for ident in np.unique(obstacles[:,0]):
        ax.add_patch(Polygon(obstacles[obstacles[:,0]==ident,1:],facecolor='#71777d',edgecolor='#4d545b'))
    s = np.r_[0,np.cumsum(np.linalg.norm(np.diff(r[:,:2],axis=0),axis=1))]
    _,unique = np.unique(s,return_index=True)
    samples = np.unique(np.rint(np.interp(np.linspace(0,s[-1],65),s[unique],unique)).astype(int))
    body = np.array([[p[5]+p[6],p[8]/2],[p[5]+p[6],-p[8]/2],[-p[7],-p[8]/2],[-p[7],p[8]/2],[p[5]+p[6],p[8]/2]])
    for i in samples:
        c,sn = np.cos(r[i,2]),np.sin(r[i,2])
        vertices = body@np.array([[c,sn],[-sn,c]])+r[i,:2]
        footprint, = ax.plot(*vertices.T,color='#66b5e8',lw=.65,label='Optimal footprints')
    mid = None
    for iteration in np.unique(history[:,0])[:-1]:
        a = history[history[:,0]==iteration]
        mid, = ax.plot(a[:,1],a[:,2],color='#aaaaaa',lw=.8,label='Intermediate iterates')
    initial, = ax.plot(reference[:,0],reference[:,1],'--',color='#262b33',lw=1.4,label='HA* reference')
    optimal, = ax.plot(r[:,0],r[:,1],color='#d62e21',lw=2,label='LIOM optimum')
    for i,label in [(0,'Start'),(-1,'Goal')]:
        ax.annotate('',xy=r[i,:2]+1.8*np.array([np.cos(r[i,2]),np.sin(r[i,2])]),xytext=r[i,:2],arrowprops={'arrowstyle':'->','color':'#1a3852'})
        ax.text(r[i,0]+.6,r[i,1]+.8,label,fontsize=10,weight='bold')
    ax.set(xlim=p[1:3],ylim=p[3:5],xlabel='x / m',ylabel='y / m',aspect='equal',title=f'LIOM parking | J = {cost:.4f} | {int(count)} iterations')
    ax.grid(alpha=.16)
    handles = [optimal,initial,footprint]+([mid] if mid is not None else [])
    ax.legend(handles=handles,loc='upper left',fontsize=9)
    fig.savefig(out/'trajectory.png',dpi=180)
    fig2,axes = plt.subplots(2,3,num='LIOM profiles',figsize=(12,7),layout='constrained')
    t = np.linspace(0,tf,len(r))
    for ax,j,label,title,limit in zip(axes.flat,[3,5,2,4,6],
        ['v / (m/s)','phi / rad','theta / rad','a / (m/s²)','omega / (rad/s)'],
        ['Speed','Steering angle','Heading','Acceleration','Steering rate'],[p[9],p[11],None,p[10],p[12]]):
        if j in [4,6]: ax.step(t,r[:,j],where='post',color='#d62e21',lw=1.3)
        else: ax.plot(t,r[:,j],color='#1a63a3',lw=1.5)
        if limit:
            ax.axhline(limit,color='.6',ls=':',lw=.7); ax.axhline(-limit,color='.6',ls=':',lw=.7)
            ax.set_ylim(-1.12*limit,1.12*limit)
        ax.set(title=title,xlabel='t / s',ylabel=label,xlim=(0,tf)); ax.grid(alpha=.2)
    axes[1,2].plot(t,r[:,:2],lw=1.5);axes[1,2].legend(['x','y'])
    axes[1,2].set(title='Rear-axle centre',xlabel='t / s',ylabel='Position / m',xlim=(0,tf));axes[1,2].grid(alpha=.2)
    fig2.suptitle(f'Chapter 5 | Optimal states and controls | T = {tf:.4f} s')
    fig2.savefig(out/'profiles.png',dpi=180)
    if show: plt.show()
    return fig,fig2


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--data',type=Path,default=Path(__file__).parent/'data')
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results')
    parser.add_argument('--no-show',action='store_true')
    args=parser.parse_args();draw(args.data,args.output,not args.no_show)
