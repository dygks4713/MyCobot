from pymycobot.mycobot import MyCobot
import time


mc = MyCobot('COM6',115200)
while True:
    mc.send_angles([0,0,0,0,0,0],20)
    time.sleep(2)
    mc.send_angle(1,20,20)
    time.sleep(2)
    mc.send_angle(2,20,20)
    time.sleep(2)