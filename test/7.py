from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord
import time
# import the project package

# Initiate a MyPalletizer object
mc = MyPalletizer("COM6", 115200)

# # Get the current coordinates and pose of the head
coords = mc.get_coords()
print(coords)

#Plan the route at random, let the head reach the coordinates of [57.0, -107.4, 316.3] in an non-linear manner at the speed is 80mm/s
mc.send_coords([187.8, 42.1, 183.3, -159.6], 80, 0)
# wait for 2 seconds
time.sleep(2)

# Plan the route at random, let the head reach the coordinates of [207.9, 47, 49.3,-159.69] in an non-linear manner at the speed is 80mm/s
mc.send_coords([207.9, 47, 49.3,-159.69], 80, 0)
# wait for 2 seconds
time.sleep(2)

#To change only the x-coordinate of the head, set the x-coordinate of the head to 20. Let it plan the route at random and move the head to the changed position at a speed of 70mm/s
mc.send_coord(Coord.X.value, 20, 50)