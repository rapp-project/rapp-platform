#!/bin/bash

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

# Authors: Konstantinos Panayiotou, Manos Tsardoulias
# contact: klpanagi@gmail.com, etsardou@iti.gr


#  ========== **Define python test files here** ==========
FACE_TEST=face_detection_test_lenna_png.py
QR_TEST=qr_detection_test_1.py
SUBCLASSES_TEST=ontology_subclasses_of_test_Oven.py
SUPERCLASSES_TEST=ontology_superclasses_of_test_1.py
IS_SUBSUPER_TEST=ontology_is_subsuperclass_of_SpatialThing.py
DENOISE_TEST=denoise_profile_test_1.py
SPEECH_DETECT_TEST=speech_detection_sphinx_test_nao_wav_1_ch_nai_oxi.py

TESTS=($FACE_TEST $QR_TEST $SUBCLASSES_TEST $SUPERCLASSES_TEST)
TESTS+=($IS_SUBSUPER_TEST $DENOISE_TEST $SPEECH_DETECT_TEST)
# ========================================================

#  These variables hold the inner and outer loop values
NUM_CONC_CALLS=1
LOOP=1
# ======================================================

for ((i=1;i<LOOP+1;i++)); do
  for TEST in "${TESTS[@]}"
  do
    for ((j=1;j<=NUM_CONC_CALLS;j++)); do
      echo -e "`python run_python_tests.py -i $TEST`"
    done
  done
done


