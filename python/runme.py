"""Run Chapter 5 LIOM using CasADi/IPOPT and generate exactly two static figures."""
import argparse
import os
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from liom import Problem, solve, save_results
from plot_results import draw


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',type=Path,default=root/'data')
    parser.add_argument('--output',type=Path,default=root/'results')
    parser.add_argument('--linear-solver',choices=['ma27','mumps'],default='ma27')
    parser.add_argument('--hsl-library',default=os.environ.get('LIOM_HSL_LIBRARY'))
    parser.add_argument('--no-show',action='store_true')
    args = parser.parse_args()
    problem = Problem.load(args.data)
    answer,stats,history,report = solve(problem,args.linear_solver,args.hsl_library)
    save_results(problem,answer,stats,history,args.output)
    draw(args.data,args.output,not args.no_show)
    print(f'Converged: J = {stats[-1,2]:.7f}, T = {answer[-1]:.7f} s; {report}')


if __name__ == '__main__': main()
