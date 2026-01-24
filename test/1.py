from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM4',115200)
mc.send_angles([0,0,0,0,0,0],20)
