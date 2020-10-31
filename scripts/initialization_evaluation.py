# coding:utf-8
#!/usr/bin/python

import os
import argparse

import numpy as np
import matplotlib.pyplot as plt
from colorama import Fore

from evaluation import Evaluation
import plot as pl

plot_format = '.png'



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='''Analyze trajectory estimate in a folder.''')
    parser.add_argument(
        'dir_results', type=str,
        help="Folder containing the groundtruth and the estimate.")
    parser.add_argument(
        '--dir_plot', type=str,
        help="Folder to output plots",
        default='')
    parser.add_argument('--plot_format',
        help='Save plots as png instead of pdf',
        action='store_true')

    args = parser.parse_args()

    assert os.path.exists(args.dir_results)

    dir_plot = args.dir_plot
    if not args.dir_plot:
        dir_plot = os.path.join(args.dir_results, 'plot')
    if not os.path.exists(dir_plot):
        os.makedirs(dir_plot)

    if args.plot_format:
        plot_format = '.pdf'

    print(Fore.RED + "****** Initialization of Visual-Inertial Odometry Evaluation ******")
    print(Fore.WHITE +
          "Begining to evaluate the initialization results in {0}.".format(args.dir_results))  
    print(Fore.WHITE + 
          "The plot will saved in {0}.".format(dir_plot))

    evaluation = Evaluation(args.dir_results)

    if evaluation.success:
        evaluation.compute_initialization_evaluation_metrics()

    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Frame id [number]', ylabel='Sub Scale.',
        xlim=[0, len(evaluation.sub_scales)])

    pl.plot_sub_scale(
        ax, range(len(evaluation.sub_scales)),
        np.reshape(evaluation.sub_scales, (-1, 1)),
        dir_plot, colors=['b'], labels=['scale'])
    ax.legend()
    fig.tight_layout()
    fig.savefig(dir_plot + '/sub_scale' + plot_format,
                bbox_inches='tight')
    print(Fore.GREEN + "****** Finished Initialization Evaluation ******")

