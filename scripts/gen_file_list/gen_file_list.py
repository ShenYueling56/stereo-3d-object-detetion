# -*- coding:utf-8 -*-
# usage: gen_file_lists.py /path/to/data/

import os
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='gen file list')
    parser.add_argument('root_dir', type=str, default="/home/shenyl/Documents/sweeper/data/", help='root_dir')
    args = parser.parse_args()

    root_dir = args.root_dir
    left_dir = root_dir + "left/"
    right_dir = root_dir + "right/"
    result_left_file = root_dir + "file_left.txt"
    result_right_file = root_dir + "file_right.txt"

    file_num = len(os.listdir(left_dir))
    with open(result_left_file, 'w') as f:
        for i in range(file_num):
            f.write(left_dir + "%06d.jpg" % (i))
            f.write('\n')
    with open(result_right_file, 'w') as f:
        for i in range(file_num):
            f.write(right_dir + "%06d.jpg" % (i))
            f.write('\n')



