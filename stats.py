import math
import csv
import numpy as np

filename = 'data/sample-laser-radar-measurement-data-1.txt'

pxs = []
pys = []
vxs = []
vys = []

with open(filename, 'rt') as csvfile:
    reader = csv.reader(csvfile, delimiter='\t')
    for row in reader:
        if row[0] == 'R':
            rho = float(row[1])
            phi = float(row[2])
            rho_dot = float(row[3])

            px = rho * math.cos(phi)
            py = rho * math.sin(phi)
            vx = rho_dot * math.cos(phi)
            vy = rho_dot * math.sin(phi)

            px_gt = float(row[5])
            py_gt = float(row[6])
            vx_gt = float(row[7])
            vy_gt = float(row[8])

            pxs.append(px - px_gt)
            pys.append(py - py_gt)
            vxs.append(vx - vx_gt)
            vys.append(vy - vy_gt)

        if row[0] == 'L':
            px = float(row[1])
            py = float(row[2])

            px_gt = float(row[4])
            py_gt = float(row[5])

            pxs.append(px - px_gt)
            pys.append(py - py_gt)

def variance(data):
    data = np.array(data)
    return np.var(data)

print('px: ', variance(pxs))
print('py: ', variance(pys))    
print('vx: ', variance(vxs))
print('vy: ', variance(vys))

