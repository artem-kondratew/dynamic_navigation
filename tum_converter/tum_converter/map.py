import sys

import matplotlib.pyplot as plt
import numpy as np


def main():
    if len(sys.argv) != 3:
        print('usage: python3 map.py <ground_truth_trajectory.txt> <trajectory.txt>')
        exit(1)

    truth = open(sys.argv[1], 'r').readlines()
    trajectory = open(sys.argv[2], 'r').readlines()

    gt_pts_x = []
    gt_pts_y = []
    tr_pts_x = []
    tr_pts_y = []

    cnt = 0

    max = None
    min = None

    for pt in truth:
        data = pt.strip().split(' ')
        if data[0] == '#' or cnt > 1000:
            continue
        gt_pts_x.append(data[1])
        gt_pts_y.append(data[2])
        print(data[1], data[2])
        cnt += 1

    # for pt in trajectory:
    #     data = pt.strip().split(' ')
    #     if data[0] == '#':
    #         continue
    #     if max == None:
    #         max = data[1]
    #     if data[1] > max:
    #         max = data[1]
    #     if min == None:
    #         min = data[2]
    #     if data[2] > min:
    #         min = data[2]
    #     print(data[1], data[2])
    #     tr_pts_x.append(data[1])
    #     tr_pts_y.append(data[2])
    #     cnt += 1
    # print(min, max)
    # make data

    # gt_x = np.array(gt_pts_x)
    # gt_y = np.array(gt_pts_y)
    # tr_x = np.array(tr_pts_x)
    # tr_y = np.array(tr_pts_y)

    gt_x = np.array([0, 2, 6, 1])
    gt_y = np.array([4, 5, 8, 3])

    # plot
    fig, ax = plt.subplots()

    ax.scatter(gt_x, gt_y, linewidth=1.0)

    plt.show()


if __name__ == '__main__':
    main()
