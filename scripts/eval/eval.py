# -*- coding:utf-8 -*-
# usage: gen_file_lists.py /path/to/data/

import os
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='gen gt')
    parser.add_argument('root_dir', type=str, default="/home/shenyl/Documents/sweeper/data/", help='root_dir')
    parser.add_argument('det_file_name', type=str, default="object_det_opencv_sgbm.txt", help='root_dir')

    args = parser.parse_args()
    root_dir = args.root_dir
    gt_path = root_dir + "object_det_result/"
    gt_file = gt_path + "gt.txt"
    det_file = gt_path + args.det_file_name
    error_z_file = gt_path + "error_z.txt"
    error_x_file = gt_path + "error_x.txt"
    error_z0_file = gt_path + "error_z0.txt"
    error_x0_file = gt_path + "error_x0.txt"
    error_z1_file = gt_path + "error_z1.txt"
    error_x1_file = gt_path + "error_x1.txt"
    error_z2_file = gt_path + "error_z2.txt"
    error_x2_file = gt_path + "error_x2.txt"
    error_z3_file = gt_path + "error_z3.txt"
    error_x3_file = gt_path + "error_x3.txt"

    e_x_0 = 0
    e_z_0 = 0
    e_x_1 = 0
    e_z_1 = 0
    e_x_2 = 0
    e_z_2 = 0
    e_x_3 = 0
    e_z_3 = 0
    count_0 = 0
    count_1 = 0
    count_2 = 0
    count_3 = 0

    f_gt = open(gt_file)
    f_det = open(det_file)
    f_error_z = open(error_z_file, 'w')
    f_error_x = open(error_x_file, 'w')
    f_error_z0 = open(error_z0_file, 'w')
    f_error_x0 = open(error_x0_file, 'w')
    f_error_z1 = open(error_z1_file, 'w')
    f_error_x1 = open(error_x1_file, 'w')
    f_error_z2 = open(error_z2_file, 'w')
    f_error_x2 = open(error_x2_file, 'w')
    f_error_z3 = open(error_z3_file, 'w')
    f_error_x3 = open(error_x3_file, 'w')

    for line_gt in f_gt.readlines():
        gt = line_gt.split(" ")

        c_gt, x_gt, z_gt = gt[0], float(gt[1]), float(gt[3])
        # print("gt")
        # print(c_gt)
        # print(str(x_gt))
        # print(str(z_gt))

        line_det = f_det.readline()

        det = line_det.split(" ")
        c_det, x_det, z_det = det[0], float(det[1]), float(det[3])
        # print("det")
        # print(c_det)
        # print(str(x_det))
        # print(str(z_det))
        if c_det != c_gt:
            # print("fail")
            print(100000)
            f_error_z.write(str(c_gt)+' '+str(100000)+'\n')

            continue

        if c_det == '0':
            # print("0")
            # if z_gt>1.3:
            #     continue
            e_z_0 = e_z_0 + abs((z_det - z_gt))
            e_x_0 = e_x_0 + abs((x_det - x_gt))
            count_0 = count_0 + 1.0
            f_error_z0.write(str(z_gt)+'\t'+str(z_det-z_gt)+'\n')
            f_error_x0.write(str(x_gt) + '\t' + str(x_gt - x_gt) + '\n')
        if c_det == '1':
            # print("1")
            # if z_gt>1.3:
            #     continue
            e_z_1 = e_z_1 + abs((z_det - z_gt))
            e_x_1 = e_x_1 + abs((x_det - x_gt))
            count_1 = count_1 + 1.0
            f_error_z1.write(str(z_gt) + '\t' + str(z_det - z_gt) + '\n')
            f_error_x1.write(str(x_gt) + '\t' + str(x_gt - x_gt) + '\n')
        if c_det == '2':
            # print("2")
            # if z_gt>1.3:
            #     continue
            e_z_2 = e_z_2 + abs((z_det - z_gt))
            e_x_2 = e_x_2 + abs((x_det - x_gt))
            count_2 = count_2 + 1.0
            f_error_z2.write(str(z_gt) + '\t' + str(z_det - z_gt) + '\n')
            f_error_x2.write(str(x_gt) + '\t' + str(x_gt - x_gt) + '\n')
        if c_det == '3':
            # print("3")
            # if z_gt>1.3:
            #     continue
            e_z_3 = e_z_3 + abs((z_det - z_gt))
            e_x_3 = e_x_3 + abs((x_det - x_gt))
            count_3 = count_3 + 1.0
            f_error_z3.write(str(z_gt) + '\t' + str(z_det - z_gt) + '\n')
            f_error_x3.write(str(x_gt) + '\t' + str(x_gt - x_gt) + '\n')
        f_error_z.write(str(c_gt)+' '+str(z_gt)+" "+str(z_det)+" "+str(z_det-z_gt)+" "+str((z_det-z_gt)/z_gt*100)+'\n')
        f_error_x.write(str(c_gt) + ' ' + str(x_gt) + " " + str(x_det) + " " + str(x_det - x_gt) + '\n')

    f_error_z.close()
    f_error_x.close()
    f_error_z0.close()
    f_error_x0.close()
    f_error_z1.close()
    f_error_x1.close()
    f_error_z2.close()
    f_error_x2.close()
    f_error_z3.close()
    f_error_x3.close()
    e_z_0, e_x_0 = e_z_0 / count_0, e_x_0 / count_0
    e_z_1, e_x_1 = e_z_1 / count_1, e_x_1 / count_1
    e_z_2, e_x_2 = e_z_2 / count_2, e_x_2 / count_2
    e_z_3, e_x_3 = e_z_3 / count_3, e_x_3 / count_3
    print("e_z_0: " + str(e_z_0))
    print("e_z_1: " + str(e_z_1))
    print("e_z_2: " + str(e_z_2))
    print("e_z_3: " + str(e_z_3))
    print("e_x_0: " + str(e_x_0))
    print("e_x_1: " + str(e_x_1))
    print("e_x_2: " + str(e_x_2))
    print("e_x_3: " + str(e_x_3))
    print("count0: " + str(count_0) + " "+str(count_0/60))
    print("count1: " + str(count_1) + " "+str(count_1/60))
    print("count2: " + str(count_2) + " "+str(count_2/60))
    print("count3: " + str(count_3) + " "+str(count_3/60))
