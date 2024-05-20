# modificirao sam kod koji smo spomenuli još u subotu s ovog linka https://matplotlib.org/mpl_examples/mplot3d/scatter3d_demo.py
# sad radi s nasumičnim točkama i temperaturama pa samo još to trebamo promjenit
# prikazuju se temperature s kružićima i bojom

from TemperaturePoint import TemperaturePoint
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np

class Plot3D:
    def __init__(self):
        pass
    def update(self, temperaturePoint):
        pass

# nasumične vrijednosti za temp i točke
def randrange(n, vmin, vmax):
    return (vmax - vmin) * np.random.rand(n) + vmin

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# n broj nasumičnih točaka te njihovih temp, onda min i max vrijednosti za svaku opciju (xs,ys,zs,temp,tlak)
n = 100
xs = randrange(n, 23, 32)
ys = randrange(n, 0, 100)
zs = randrange(n, -50, 50)
temperatures = randrange(n, 18, 25)

# kružni markeri za temp
scatter = ax.scatter(xs, ys, zs, c=temperatures, s=100, cmap='plasma', alpha=0.6, marker='o')

colorbar = fig.colorbar(scatter, ax=ax)
colorbar.set_label('Temperatura (°C)')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
