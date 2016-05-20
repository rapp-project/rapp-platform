#!/usr/bin/env python

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.


PKG='ros_nodes'

import sys
import unittest
import rospy
import roslib
import rospkg
from geometry_msgs.msg import PointStamped

from rapp_platform_ros_communications.srv import (
  FaceRecognitionRosSrv,
  FaceRecognitionRosSrvRequest
  )


class FaceRecogFunc(unittest.TestCase):
    """Handles the face recognition functional tests
    """

    ## Tests learn face model and face recognition with an image. Should return faceID equal to 11
    def test_faceLearn_realistic(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_recognition_recognize_faces_topic")
        rospy.wait_for_service(face_service)
        fr_service = rospy.ServiceProxy(face_service, FaceRecognitionRosSrv)
        req = FaceRecognitionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/face_samples/face_recog_1.png'
        req.learn = True
        req.fn_csv = rospack.get_path('rapp_testing_tools') + \
                '/test_data/face_samples/face_recognition_model/faces-1.csv'
        req.model_name = 'eigenfaces_recog_new.yml'
        req.user = 'Jan'
        req.recognize = True
        ## Faces
        faces_up_left = PointStamped()
        # data for the given image (face_recog_1.png) # 263,119;369,225; 
        faces_up_left.point.x = 263;
        faces_up_left.point.y = 119;
        list_faces_up_left = []
        list_faces_up_left.append(faces_up_left)
        faces_down_right = PointStamped()
        faces_down_right.point.x = 369;
        faces_down_right.point.y = 225;
        list_faces_down_right = []
        list_faces_down_right.append(faces_down_right)
        req.faces_up_left = list_faces_up_left
        req.faces_down_right = list_faces_down_right

        
        response = fr_service(req)
        face_id = -1
        if (len(response.recognizedIDs) > 0):
            face_id = response.recognizedIDs[len(response.recognizedIDs)-1]
        self.assertEqual( face_id, 11 )
    ## Tests face recognition with an image. Should return faceID equal to 11
    def test_faceRecog_realistic(self):
        rospack = rospkg.RosPack()
        face_service = rospy.get_param("rapp_face_recognition_recognize_faces_topic")
        rospy.wait_for_service(face_service)
        fr_service = rospy.ServiceProxy(face_service, FaceRecognitionRosSrv)
        req = FaceRecognitionRosSrvRequest()
        req.imageFilename = rospack.get_path('rapp_testing_tools') + \
                '/test_data/face_samples/face_recog_1.png'
        req.learn = False
        req.fn_csv = ''
        req.recognize = True
        req.user = 'Jan'
        req.model_name = 'eigenfaces_recog_new.yml'
        faces_up_left = PointStamped()
        # data for the given image (face_recog_1.png) # 263,119;369,225; 
        faces_up_left.point.x = 263;
        faces_up_left.point.y = 119;
        list_faces_up_left = []
        list_faces_up_left.append(faces_up_left)
        faces_down_right = PointStamped()
        faces_down_right.point.x = 369;
        faces_down_right.point.y = 225;
        list_faces_down_right = []
        list_faces_down_right.append(faces_down_right)
        req.faces_up_left = list_faces_up_left
        req.faces_down_right = list_faces_down_right
        
        response = fr_service(req)
        face_id = response.recognizedIDs[len(response.recognizedIDs)-1]
        self.assertEqual( face_id, 11 )

    
   

## The main function. Initializes the functional tests
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'FaceRecogFunc', FaceRecogFunc)














