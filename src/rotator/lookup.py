import csv
import os

LOOKUP_FNAME = 'lookup.csv'

from .math_utils import solve_positions, C_OFFSET

def create_lookup():
    with open(LOOKUP_FNAME, 'w') as csvfile:
        writer = csv.writer(csvfile)
        for azimuth in range(0, 360*2):
            azimuth = round(azimuth / 2.0, 1)
            for elevation in range(0, 81):
                elevation = round(50 + elevation / 2.0, 1)
                pose = solve_positions(azimuth, elevation)
                writer.writerow([azimuth, elevation, *pose])

def lookup_pose(pose_n, pose_e, pose_s, pose_w):
    if not os.path.exists(LOOKUP_FNAME):
        create_lookup()

    data = []
    with open(LOOKUP_FNAME, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for azimuth, elevation, pose_n_, pose_e_, pose_s_, pose_w_ in reader:
            azimuth = float(azimuth)
            elevation = float(elevation)
            pose_n_ = float(pose_n_) + 120
            pose_e_ = float(pose_e_) + 120
            pose_s_ = float(pose_s_) + 120
            pose_w_ = float(pose_w_) + 120
            data.append({
                'az': azimuth,
                'el': elevation,
                'pose': [pose_n_, pose_e_, pose_s_, pose_w_]
            })

    azimuth = 0
    elevation = 90
    min_delta = 100
    for record in data:
        delta = sum([
            abs(x - y)
            for x, y in zip([pose_n, pose_e, pose_s, pose_w], record['pose'])
        ])
        if delta < min_delta:
            min_delta = delta
            azimuth = record['az']
            elevation = record['el']

    return azimuth, elevation