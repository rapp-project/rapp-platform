#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest

class FaceDetectionFunc(unittest.TestCase):

    def naive_test(self):
        self.assertEqual(1,2)
        print "ok"

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'face_detection_func', FaceDetectionFunc)
