# coding:utf-8
#!/usr/bin/python

import numpy as np
import transformations as tf

def umeyama_transforamtion(positions_1, positions_2):
    mu_1 = positions_1.mean(0)
    mu_2 = positions_2.mean(0)
    pts1_decentroid = positions_1 - mu_1
    pts2_decentroid = positions_2 - mu_2
    n = np.shape(positions_1)[0]

    W = 1.0 / n*np.dot(pts1_decentroid.transpose(), pts2_decentroid)
    sigma_1 = 1.0 / n * np.multiply(pts2_decentroid, pts2_decentroid).sum()
    U, D, V = np.linalg.linalg.svd(W)
    D = np.diag(D)
    V = np.transpose(V)

    S = np.eye(3)
    if(np.linalg.det(U) * np.linalg.det(V) < 0):
        S[2, 2] = -1

    R = np.dot(U, np.dot(S, np.transpose(V)))

    s = 1.0 / sigma_1 * np.trace(np.dot(D, S))

    t = mu_1 - s * np.dot(R, mu_2)

    return s, R, t