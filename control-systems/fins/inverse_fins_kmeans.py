from sklearn.cluster import KMeans
from sklearn.model_selection import train_test_split
import numpy as np
import sys
import argparse

argparser = argparse.ArgumentParser()
argparser.add_argument('-n', default=20, type=int)
argparser.add_argument('-m', defualt='moments.csv')
argparser.add_argument('-a', default='angles.csv')

args = argparser.parse_args()
kmeans = KMeans(n_clusters=args.n, random_state=0)

moments = np.loadtxt(args.m, delimiter=',')
angles = np.loadtxt(args.a, delimiter=',')

kmeans.fit(moments)

X_train = kmeans.cluster_centers_
y_train = np.zeros(shape=(kmeans.n_clusters, 3), dtype=int)

for j, x in enumerate(X_train):

    argmin = 0
    minimum = sys.maxsize

    for i in range(moments.shape[0]):
        temp = np.linalg.norm(x - moments[i])
        if temp < minimum:
            minimum = temp
            argmin = i
    y_train[j, :] = angles[argmin, :]

np.savetxt('kmeans_centers.csv', X_train, fmt='%.3e', delimiter=',')
np.savetxt('kmeans_angles.csv', y_train, fmt='%i', delimiter=',')
