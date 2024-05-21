from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np

class Plot3D:
    def __init__(self):
        # Inicijalizacija 3D projekcije
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Lista točaka
        self.points = []
        
        # Graf i labela
        self.graph = None
        self.colorbar = None
        self.norm = None
        plt.ion()
        
        # Inicijalne vrijednosti na osima - kasnije se mjenjaju po min/max vrijednostima
        self.ax.set_xlim([0, 20])
        self.ax.set_ylim([0, 10])
        self.ax.set_zlim([0, 10])
        
        # Nazivi osi
        self.ax.set_xlabel('X Axis')
        self.ax.set_ylabel('Y Axis')
        self.ax.set_zlabel('Z Axis')
        
    def update(self, temperaturePoint):
        # Dodaj novu točku
        self.points.append(temperaturePoint)
        
        # Razdvajanje
        x = [point.x for point in self.points]
        y = [point.y for point in self.points]
        z = [point.z for point in self.points]
        temp = [point.temp for point in self.points]
        
        # Uklonit prijašnje podatke ako postoje
        if self.graph:
            self.graph.remove()
        
        # Dodaj nove podatke
        self.graph = self.ax.scatter(x, y, z, s=100, c=temp, cmap='plasma', marker='o')
        
        # Prikaz labele boja
        if not self.colorbar:
            self.norm = plt.Normalize(min(temp), max(temp))
            self.colorbar = self.fig.colorbar(cm.ScalarMappable(norm=self.norm, cmap='plasma'), ax=self.ax, pad=0.1)
            self.colorbar.set_label('Temperature (°C)')
        else:
            self.norm.vmin = min(temp)
            self.norm.vmax = max(temp)
            self.colorbar.update_normal(cm.ScalarMappable(norm=self.norm, cmap='plasma'))
        
        # Azuriranje granica osi
        self.ax.set_xlim([0, max(x) + 1])
        self.ax.set_ylim([0, max(y) + 1])
        self.ax.set_zlim([0, max(z) + 1])
        
        # Iscrtavanje
        plt.draw()
