"""Run the compiled C++ solver, then draw its two static result figures.

All corridor construction and optimization run in the C++ executable.
This launcher uses Python only for process invocation and visualization.
"""
from pathlib import Path
import argparse
import os
import subprocess


def main():
    root = Path(__file__).resolve().parent
    executable = root/'build'/('liom.exe' if os.name == 'nt' else 'liom')
    if not executable.exists() and (root/'build'/'Release'/executable.name).exists():
        executable = root/'build'/'Release'/executable.name
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable',type=Path,default=executable)
    parser.add_argument('--data',type=Path,default=root/'data')
    parser.add_argument('--output',type=Path,default=root/'results')
    parser.add_argument('--linear-solver',choices=['ma27','mumps'],default='ma27')
    parser.add_argument('--hsl-library',default=os.getenv('LIOM_HSL_LIBRARY'))
    parser.add_argument('--no-show',action='store_true')
    args = parser.parse_args()
    if not args.executable.is_file():
        parser.error('Build the C++ executable first; see cpp/README.md.')
    command = [str(args.executable.resolve()),'--data',str(args.data.resolve()),
               '--output',str(args.output.resolve()),'--linear-solver',args.linear_solver]
    if args.hsl_library: command += ['--hsl-library',args.hsl_library]
    env = os.environ.copy()
    if os.name == 'nt' and env.get('CASADIPATH'):
        env['PATH'] = env['CASADIPATH']+os.pathsep+env.get('PATH','')
    env.setdefault('OMP_NUM_THREADS','1')
    env.setdefault('OPENBLAS_NUM_THREADS','1')
    subprocess.run(command,check=True,env=env)
    from plot_results import draw
    draw(args.data,args.output,show=not args.no_show)


if __name__ == '__main__': main()
