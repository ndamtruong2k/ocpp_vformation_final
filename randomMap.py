import numpy as np
import matplotlib.pyplot as plt
from sweep_line_has_bad import getConvexPolygon

n_vertices = 6
polygon_radius = 50
rad_var = 1
ang_var = 1
dx = 2
transl_spd = 12
rot_spd = np.pi/2

M, Mshifted= getConvexPolygon(n_vertices,polygon_radius,rad_var,ang_var)
coord = M.tolist()

# coord = [[58.98295314305732, -40.46389776524755], [-19.5748849118947, -78.531936254165], [-62.674712335622026, 24.06481669719506], [-31.09947113556031, 61.08658069805723], [52.20911077098446, 26.412130624396212]]

print(coord)

coord.append(coord[0])
xs, ys = zip(*coord) #create lists of x and y values

plt.figure()
plt.fill(xs,ys)
plt.xlim(-100, 100)
plt.ylim(-100, 100)
plt.show() # if you need...