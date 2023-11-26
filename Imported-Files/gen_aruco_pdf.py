# This Python script will draw a set of 6 markers from the aruco.DICT_6X6_250 library
# to a PDF file (markers.pdf)

# Import the necessary packages
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl


filename = "markers.pdf"
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

fig = plt.figure()
nx = 3
ny = 2
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,i, 700)
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")

plt.savefig(filename)
plt.show()