#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import sys
from respeaker_ros.respeaker3800 import PARAMETERS_3800, Respeaker3800

def main():
    dev = Respeaker3800()
    if not dev:
        print('No device found. Please connect a device.')
        return
    
    for key, val in PARAMETERS_3800.items():
        #print(key)
        count, rw_, type_, desc = val[2:6]
        value = dev.read(key)
        print(f"KEY: {key} value:{value}")

# def write():
#     dev = Respeaker3800()
#     if not dev:
#         print('No device found. Please connect a device.')
#         return
#     ## defaults
#     #AUDIO_MGR_OP_ALL value:(8, 0, 1, 0, 1, 2, 7, 3, 1, 1, 1, 3)
#     dev.write("AUDIO_MGR_OP_ALL", (8, 0, 0, 0, 7, 2, 7, 3, 7, 1, 0, 0))


if __name__ == '__main__':
    main()
