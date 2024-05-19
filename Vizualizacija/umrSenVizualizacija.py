from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np

colorbar = None
colorbar2 = None

def randrange(n, vmin, vmax):
    return (vmax - vmin) * np.random.rand(n) + vmin

def switch_view(event):
    global view_temperature
    view_temperature = not view_temperature
    update_plot()

def update_plot():
    global scatter, colorbar, colorbar2
    if view_temperature:
        scatter.set_array(temperatures)
        scatter.set_cmap('plasma')
        scatter.set_visible(True)
        scatter2.set_visible(False)
        scatter.set_clim(vmin=min(temperatures), vmax=max(temperatures))
        if colorbar2 is not None: 
            colorbar2.remove()
        if colorbar is None:
            colorbar = fig.colorbar(scatter, ax=ax)
            colorbar.set_label('Temperature (°C)')
    else:
        scatter2.set_array(pressure)
        scatter2.set_cmap('viridis')
        scatter.set_visible(False)
        scatter2.set_visible(True)
        scatter2.set_clim(vmin=min(pressure), vmax=max(pressure))
        if colorbar is not None:
            colorbar.remove()
        colorbar = None  # Reset colorbar to None when switching views
        colorbar2 = fig.colorbar(scatter2, ax=ax)
        colorbar2.set_label('Pressure (hPa)')
    plt.draw()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

n = 100
xs = randrange(n, 23, 32)
ys = randrange(n, 0, 100)
zs = randrange(n, -50, 50)
temperatures = randrange(n, 18, 25)
pressure = randrange(n, 1008.25, 1018.25)

view_temperature = True
scatter = ax.scatter(xs, ys, zs, c=temperatures, s=100, cmap='plasma', alpha=0.6, marker='o')
scatter2 = ax.scatter(xs, ys, zs, c=pressure, s=100, cmap='viridis', alpha=0.6, marker='^')
scatter2.set_visible(False)

colorbar = fig.colorbar(scatter, ax=ax)
colorbar.set_label('Temperature (°C)')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

ax_switch = plt.axes([0.7, 0.01, 0.2, 0.05])
button = Button(ax_switch, 'Switch View')
button.on_clicked(switch_view)

plt.show()
