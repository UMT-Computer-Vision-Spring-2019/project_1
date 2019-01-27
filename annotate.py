import matplotlib.pyplot as plt
import numpy as np

im = plt.imread('IMG_0433.JPG')

plt.figure(figsize=(12, 8))

plt.imshow(im)
coords = plt.ginput(n=1, show_clicks=True)
print(coords)

