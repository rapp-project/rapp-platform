#!/usr/bin/env python
PKG='ros_nodes'

import sys
import unittest
import rospy
from rapp_platform_ros_communications.srv import (
  SpeechToTextSrv,
  SpeechToTextSrvRequest
  )

class SpeechToTextFunc(unittest.TestCase):

    def test_speechToTextFunctional(self):
        rospy.wait_for_service('ric/speech_to_text_service')
        stt_service = rospy.ServiceProxy('ric/speech_to_text_service', SpeechToTextSrv)
        req = SpeechToTextSrvRequest()
        req.filename.data = \
            "/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform/ric/test_auxiliary_files/test.flac"
        response = stt_service(req)
        words_basic = len(response.words)

        # Check number of words 
        self.assertEqual( words_basic, 6)

        # Check the actual words
        self.assertEqual( response.words[0].data, "I") 
        self.assertEqual( response.words[1].data, "want") 
        self.assertEqual( response.words[2].data, "to") 
        self.assertEqual( response.words[3].data, "use") 
        self.assertEqual( response.words[4].data, "the") 
        self.assertEqual( response.words[5].data, "Skype") 

        # Check number of alternatives
        self.assertEqual( len(response.alternatives), 4) 


       
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'SpeechToTextFunc', SpeechToTextFunc)














