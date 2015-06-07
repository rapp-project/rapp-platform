#!/bin/bash

#CURRENT_DIR=$(pwd)
cd set_denoise_profile && python test.py
cd speech_detection_sphinx4 && python test.py
cd qr_detection && python test.py
cd face_detection && python test.py


