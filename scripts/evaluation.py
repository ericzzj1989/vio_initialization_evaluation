# coding:utf-8
#!/usr/bin/python

import os
import numpy as np

import associate_timestamps as at
import alignment as align

from colorama import init, Fore

class Evaluation:
    def __init__(self, dir_results, 
                 file_gt='groundtruth.txt',
                 file_init_est='init_estimate.txt'):
        self.success = False
        self.dir_data = dir_results
        self.t_est, self.p_est, self.q_est, self.t_gt, self.p_gt, self.q_gt = \
            self.read_data(file_gt, file_init_est) 
        if self.p_est.size == 0:
            print(Fore.RED + "There is no data in the estimate file.")
            return
        
        self.success = True

        self.init_scale_converge_range = 0.1
        self.init_start_id = 3
        self.init_end_id = len(self.p_est)
        self.max_init_frame_num = int(0.5 * self.init_end_id)
        self.sub_scales = []

    def read_data(self, file_gt, file_init_est):
        if not os.path.exists(os.path.join(self.dir_data, file_gt)) or \
                not os.path.exists(os.path.join(self.dir_data, file_init_est)):
            print(Fore.RED + "Either groundtruth or estimate does not exist")
            return False

        print("Reading groundtruth {0} and initialization estimation {1} data...".format(file_gt, file_init_est))

        dir_gt = os.path.join(self.dir_data, file_gt)
        data_gt = np.loadtxt(dir_gt)
        dir_init_est = os.path.join(self.dir_data, file_init_est)
        data_est = np.loadtxt(dir_init_est)

        matches = np.array([])
        matches = at.read_files_and_associate(dir_init_est, dir_gt, 0.0, 0.02)
        dict_matches = dict(matches)
        
        t_est = []
        p_est = []
        q_est = []
        t_gt = []
        p_gt = []   
        q_gt = []

        for id_est, est in enumerate(data_est):
            if id_est in dict_matches:
                gt = data_gt[dict_matches[id_est]]
                t_est.append(est[0])
                p_est.append(est[1:4])
                q_est.append(est[4:8])
                t_gt.append(gt[0])
                p_gt.append(gt[1:4])
                q_gt.append(gt[4:8])

        t_est = np.array(t_est)
        p_est = np.array(p_est)
        q_est = np.array(q_est)
        t_gt = np.array(t_gt)
        p_gt = np.array(p_est)
        q_gt = np.array(q_gt)

        return t_est, p_est, q_est, t_gt, p_gt, q_gt
    
    def compute_initialization_evaluation_metrics(self):
        print(Fore.WHITE + 'Calculating VIO initilization evaluation metrics...')

        for id_init in range(0, self.init_end_id):
            if id_init >= self.init_start_id:
                scale, R, t = align.umeyama_transforamtion(self.p_gt[0:id_init, :], self.p_est[0:id_init, :])
                self.sub_scales.append(scale)
            else:
                self.sub_scales.append(-0.1)

        convergence_id = 0
        sub_scale_final = self.sub_scales[-1]
        for convergence_id in range(0, self.init_end_id):
            if self.sub_scales[convergence_id] < 0:
                continue
            is_convergence = True
            for sub_scale_id in range(convergence_id, self.init_end_id):
                sub_scale = self.sub_scales[sub_scale_id]
                if (np.absolute(sub_scale - sub_scale_final) / sub_scale_final > self.init_scale_converge_range):
                    is_convergence = False
                    break

            if is_convergence:
                break
                
        if convergence_id > self.max_init_frame_num:
            print(Fore.RED + "****** Initialization of Visual-Inertial Odometry Evaluation Results ******")
            print(Fore.RED + "Initialization cannot converge and fail.")
            print(Fore.RED + "Please review the scale variation plot for further debug")
            return False
        else:
            convergence_p_est_aligned = np.zeros(shape=(convergence_id, 3))
            convergence_p_gt = np.zeros(shape=(convergence_id, 3))
            scale, R, t = align.umeyama_transforamtion(self.p_gt[0:convergence_id, :], self.p_est[0:convergence_id, :])
            for i in range(0, convergence_id):
                convergence_p_est_aligned[i, :] = scale * R.dot(self.p_est[i, :]) + t
                convergence_p_gt[i, :] = self.p_gt[i, :]

            print(Fore.RED + 'Calculating convergence time...')
            t_conv = self.t_est[convergence_id] - self.t_est[0]
            print(Fore.GREEN + 'done.')

            gt_init = np.diff(convergence_p_gt, 0)
            est_init = np.diff(convergence_p_est_aligned, 0)
            dist_gt_init = np.sqrt(np.sum(np.multiply(gt_init, gt_init), 1))
            dist_est_init = np.sqrt(np.sum(np.multiply(est_init, est_init), 1))
            e_scale_perc = np.abs((np.divide(dist_est_init, dist_gt_init) - 1.0) * 100)
            print(Fore.RED + 'Calculating ASE...')
            ase_init = np.sqrt(np.dot(e_scale_perc, e_scale_perc) / len(e_scale_perc))
            print(Fore.GREEN + 'done.')
            
            print(Fore.RED + 'Calculating initialization score (IS)...')
            init_score =  t_conv * ase_init
            print(Fore.GREEN + 'done.')

            print(Fore.GREEN + "****** Initialization of Visual-Inertial Odometry Evaluation Results ******")
            print(Fore.GREEN + " Scale converges at frame index (out of total " + str(len(self.t_est)) + " frames)" + "#:" + repr(
                convergence_id) + "\n Convergence time(sec):" + repr(float("{0:.6f}".format(t_conv))) +
                  "\n First converged sub-scale:" + repr(
                float("{0:.6f}".format(self.sub_scales[convergence_id]))) + "\n Final sub-scale:" + repr(
                float("{0:.6f}".format(self.sub_scales[-1])))
                  + "\n Initialization score (IS): " + repr(float("{0:.6f}".format(init_score))))
    
