import matplotlib.pyplot as plt
import numpy as np

ims = ['campus_stereo_1.jpg', 'campus_stereo_2.jpg']

for f in ims:
    im = plt.imread(f)
    plt.figure(figsize=(12, 8))
    plt.imshow(im)
    coords = plt.ginput(n=1, show_clicks=True)
    print(coords)

