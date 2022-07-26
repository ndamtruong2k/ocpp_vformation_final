from allMap import Map
import matplotlib.pyplot as plt

def area(x, y):
    return abs(sum(x[i] * (y[i + 1] - y[i - 1]) for i in range(-1, len(x) - 1))) / 2.0

coord = Map().map16

coord.append(coord[0])
xs, ys = zip(*coord) #create lists of x and y values

plt.figure()
plt.title("Area_map:{0}".format(area(xs,ys))+ "[m]")
plt.fill(xs,ys)
plt.xlim(-100, 100)
plt.ylim(-100, 100)
plt.show() # if you need...