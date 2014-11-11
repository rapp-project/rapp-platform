#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest
import roslib

class FaceDetectionFuncTests(unittest.TestCase):

    def naiveTest(self):
        self.assertEqual(1,1)
        print "ok"

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'face_detection_func_testing', FaceDetectionFuncTests)
