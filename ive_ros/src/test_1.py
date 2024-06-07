# #!/usr/bin/env python3
# # -*- coding:utf-8 -*-
# import sys
# import os

# import time

# from pymycobot.mycobot import MyCobot
# from pymycobot.genre import Coord
# from pynput import keyboard
# import rospy
# import cv2 as cv
# import time
# import numpy as np
# mc = MyCobot("/dev/ttyACM1", 115200)
# mc.send_angles([0,0,0,0,0,0],20)
# input_key = None

# def on_press(key):
#     global input_key
#     try:
#         if key.char == '1':
#             input_key = '1'
#         elif key.char == '2':
#             input_key = '2'
#         elif key.char == '3':
#             input_key = '3'
#         elif key.char == '4':
#             input_key = '4'
#     except AttributeError:
#         pass

# def on_release(key):
#     global input_key
#     if key == keyboard.Key.esc:
#         return False
    
# def key_input(key, mc):
#     if key == '1':
#         print("1이 입력되었습니다.")
#         mc.send_coord(1, 230, 30)
#     elif key =='2':
#         print("2가 입력되었습니다.")
#         mc.send_coord(1, 200, 30)
#     elif key == '3':
#         print("3가 입력되었습니다.")
#         mc.send_coords([210, 0, 300, -180, 0.0, -90], 30)
#     elif key == '4':
#         print("4가 입력되었습니다.")
#         mc.send_angles([0,0,0,0,0,0], 30)
#     key == None

# def main():
#     # global input_key
#     print("들어옴")
#     listener = keyboard.Listener(on_press=on_press, on_release=on_release)
#     listener.start()
#     while True:
#         if input_key == '1' or input_key =='2' or input_key =='3' or input_key == '4':
#             key_input(input_key, mc)

#         #     mc.send_coord(1, 10, 10)
#         #     input_key = None
#         # elif input_key=='2':
#         #     mc.send_coord(1, -10, 10)
#         #     input_key = None
#         # elif input_key=='3':
#         #     mc.send_coords([210, 0, 300, -180, 0.0, -90], 30)
#         #     input_key = None
#         #     print("3했음")
        
#         # listener.stop()

# if __name__ == "__main__":
#     try:
#         main()
#     except :
#         pass
#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import os
import time
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from pynput import keyboard
import rospy
import cv2 as cv
import numpy as np

mc = MyCobot("/dev/ttyACM0", 115200)
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
input_key = None

def on_press(key):
    global input_key
    try:
        if key.char == '1':
            input_key = '1'
        elif key.char == '2':
            input_key = '2'
        elif key.char == '3':
            input_key = '3'
        elif key.char == '4':
            input_key = '4'
    except AttributeError:
        pass

def on_release(key):
    global input_key
    if key == keyboard.Key.esc:
        return False

def key_input(key, mc):
    global input_key
    if key == '1':
        print("1이 입력되었습니다.")
        # mc.send_coord(1, 230, 30)
        mc.jog_coord(1, 1, 20)
    elif key == '2':
        print("2가 입력되었습니다.")
        mc.send_coord(1, 200, 30)
    elif key == '3':
        print("3이 입력되었습니다.")
        mc.send_coords([210, 0, 300, -180, 0.0, -90], 30)
    elif key == '4':
        print("4가 입력되었습니다.")
        mc.send_angles([0, 0, 0, 0, 0, 0], 30)
    input_key = None

def main():
    global input_key
    print("프로그램이 시작되었습니다.")
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    try:
        while True:
            if input_key in ['1', '2', '3', '4']:
                key_input(input_key, mc)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("프로그램이 종료되었습니다.")
    finally:
        listener.stop()

if __name__ == "__main__":
    main()
