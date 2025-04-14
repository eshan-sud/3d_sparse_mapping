#!/usr/bin/env python3

# Script to generate associate.txt required for execution of ORB-SLAM3 RGB-D mode of TUM dataset

import argparse
import os

def read_file_list(filename):
    with open(filename) as f:
        lines = f.read().replace(",", " ").replace("\t", " ").split("\n")
    lines = [[v.strip() for v in line.split(" ") if v.strip() != ""] for line in lines if len(line) > 0 and line[0] != "#"]
    return dict((float(l[0]), l[1:]) for l in lines if len(l) > 1)

def associate(first_list, second_list, offset, max_difference):
    first_keys = set(first_list.keys())
    second_keys = set(second_list.keys())
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    matches.sort()
    return matches

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them.
    It outputs timestamp matches between the two.
    ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--output', help='output file path (default: stdout)', default=None)
    parser.add_argument('--first_only', help='only output associated lines from first file', action='store_true')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)', type=float, default=0.0)
    parser.add_argument('--max_difference', help='max allowed time difference for matching entries (default: 0.02)', type=float, default=0.02)
    args = parser.parse_args()

    first_list = read_file_list(args.first_file)
    second_list = read_file_list(args.second_file)

    matches = associate(first_list, second_list, args.offset, args.max_difference)

    output_lines = []
    if args.first_only:
        for a, b in matches:
            output_lines.append("%f %s" % (a, " ".join(first_list[a])))
    else:
        for a, b in matches:
            output_lines.append("%f %s %f %s" % (a, " ".join(first_list[a]), b - args.offset, " ".join(second_list[b])))

    if args.output:
        with open(args.output, 'w') as f:
            f.write("\n".join(output_lines) + "\n")
    else:
        for line in output_lines:
            print(line)
