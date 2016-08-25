import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

lines = [line.rstrip('\n') for line in open('grid.occ')]

first_line = lines[0].split()
second_line = lines[1].split()

print('Grid size is: ')
print(first_line[7])

grid_size = int(first_line[7])

# make values from -5 to 5, for this example

fig = plt.figure(figsize=(32, 32))

k = 16
zvals = np.random.rand(grid_size, grid_size)
for i in range (0, grid_size):
    for j in range (0, grid_size):
        zvals[j, i] = second_line[k * grid_size * grid_size + j * grid_size + i]

cmap = mpl.colors.LinearSegmentedColormap.from_list('my_colormap',
                                                       [(0.0, 'blue'), (1.0, 'black')],
                                                        256)
plt.subplot(1, 1, 1)
plt.xticks(np.arange(0, 31.5, 1.0))
plt.yticks(np.arange(0, 31.5, 1.0))
plt.grid(True, color='white')
plt.imshow(zvals, interpolation='nearest', cmap=cmap)
plt.axis([-0.5, 31.5, -0.5, 31.5])
plt.colorbar()

plt.show()
#img = plt.imshow(zvals,
 #                       interpolation='nearest',
  #                      cmap = cmap,
   #                     origin='lower')

# make a color bar
#plt.colorbar(img, cmap=cmap)

#plt.show()
