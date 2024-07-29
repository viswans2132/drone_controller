import numpy as np
import matplotlib.pyplot as plt

X = np.arange(-0.5,0.5,0.01)
Y = np.sqrt(4*(0.25 - X**2))

plt.plot(X,Y)
plt.axis('equal')
plt.show()