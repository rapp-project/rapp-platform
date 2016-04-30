#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr


import xml.etree.ElementTree as ET
import rospkg
import rospy
import os
import sys
from app_error_exception import AppError

from rapp_platform_ros_communications.srv import (
  cognitiveTestCreatorSrv,
  cognitiveTestCreatorSrvResponse,
  cognitiveTestCreatorSrvRequest,
  createCognitiveExerciseTestSrv,
  createCognitiveExerciseTestSrvResponse,
  createCognitiveExerciseTestSrvRequest,
  cognitiveTestsOfTypeSrv,
  cognitiveTestsOfTypeSrvRequest,
  cognitiveTestsOfTypeSrvResponse,
  returnTestsOfTypeSubtypeDifficultySrv,
  returnTestsOfTypeSubtypeDifficultySrvResponse,
  returnTestsOfTypeSubtypeDifficultySrvRequest
  )

from rapp_platform_ros_communications.msg import (
  StringArrayMsg
  )

## @class CognitiveTestCreator
# @brief Provides the necessary functions for the cognitive test creator service
#
# It implements the cognitive test creator service
class CognitiveTestCreator:

  ## @brief The callback function of the cognitive test creator service, it reads the cognitive test from the text file, converts it to xml and registers it on the ontology
  # @param req [rapp_platform_ros_communications::cognitiveTestCreatorSrvRequest::Request&] The ROS service request
  # @param res [rapp_platform_ros_communications::cognitiveTestCreatorSrvResponse::Response&] The ROS service response

  def testCreator(self,req):
    res=cognitiveTestCreatorSrvResponse()
    fname=req.inputFile
    d=dict()
    questions=dict()
    answers=dict()
    correctAnswers=dict()
    listQuestions=[]
    listAnswers=[]
    listCorrectAnswers=[]
    supportedLanguages=[]
    flag=False
    questionsStart=False
    count=0    
    
    try:
      with open(fname) as f:
        content = f.readlines()
      for s in content:
        if (":" in s):
          s=s.strip()
          tmp=s.split(":")
          if(tmp[0] == "Difficulty" or tmp[0]=="variationID"):
            if (not self.is_int(tmp[1])):
              res.trace.append("error, difficulty or variation ID is not an integer")
              flag=False
          if(tmp[0] == "language"):
            if(not len(questions)==len(answers)==len(correctAnswers)):
              flag=False
              res.trace.append("error, test is broken, questions-answers-correctAnswers not equal in size")
              break
            supportedLanguages.append(tmp[1])
            listQuestions.append(questions.copy())
            questions.clear()
            listAnswers.append(answers.copy())
            answers.clear()
            listCorrectAnswers.append(correctAnswers.copy())
            correctAnswers.clear()
            questionsStart=True
            flag=True
            count=0
          elif(questionsStart):
            if(tmp[0] == "question"):
              count=count+1;
              tmp[0]=tmp[0]+str(count)
              #print tmp[1]
              questions[tmp[0]]=tmp[1]
            elif(tmp[0]=="answers"):
              tmp[0]=tmp[0]+str(count)
              answers[tmp[0]]=tmp[1]
            elif(tmp[0]=="correctAnswer"):
              tmp[0]=tmp[0]+str(count)
              if(tmp[1] not in answers["answers"+str(count)]):
                flag=False
                res.trace.append("Correct Answer not in answers in Question "+str(count) +" in test "+fname )
                break
              correctAnswers[tmp[0]]=tmp[1]
            else:
              flag=False
              res.trace.append("field other than questions,answers,correctAnswer found")
              break
          else:
            d[tmp[0]]=[tmp[1]]

      if(not len(questions)==len(answers)==len(correctAnswers)):
        flag=False
        res.trace.append("error, test is broken, questions-answers-correctAnswers not equal in size")
      else:
        listQuestions.append(questions.copy())
        questions.clear()
        listAnswers.append(answers.copy())
        answers.clear()
        listCorrectAnswers.append(correctAnswers.copy())
        correctAnswers.clear()
      xmlFileName=""
      if(flag):
        #xmlFileName=d["testType"][0]+"_"+d["testSubType"][0]+"_"+"diff"+d["difficulty"][0]
        root = ET.Element("cognitiveTest")
        ET.SubElement(root, "testType").text = d["testType"][0] 
        ET.SubElement(root, "testSubType").text = d["testSubType"][0]
        Languages=ET.SubElement(root,"Languages")
        
        for l in range(1,len(listQuestions)):
          questions=listQuestions[l] 
          answers=listAnswers[l]    
          correctAnswers=listCorrectAnswers[l] 
          currentLanguage=ET.SubElement(Languages, supportedLanguages[l-1])
          for i in range(1,count+1):
            nm="Q"+str(i)
            Q=ET.SubElement(currentLanguage, "Question", name=nm)
            nm="question"+str(i)
            ET.SubElement(Q, "body").text = questions[nm].decode('UTF-8')
            nm="answers"+str(i)
            answs=answers[nm].split(",")
            for j in answs:
              A=ET.SubElement(Q, "answer")
              ET.SubElement(A, "body").text = j.decode('UTF-8')
            nm="correctAnswer"+str(i)
            corrAnswer=correctAnswers[nm]
            ET.SubElement(Q, "correctAnswer").text = corrAnswer.decode('UTF-8')

        tree = ET.ElementTree(root)
        rospack = rospkg.RosPack()

        localPackagePath=rospack.get_path('rapp_cognitive_exercise')
        test_id=self.determineCognitiveTestId(d["testType"][0], d["testSubType"][0], d["difficulty"][0])
        xmlFileName=d["testType"][0]+"_"+d["testSubType"][0]+"_"+"diff"+d["difficulty"][0]+"_id_"+str(test_id)
        inNodeName="/cognitiveTests/"+xmlFileName+".xml"
        localPackagePath=localPackagePath+inNodeName
        tree.write(localPackagePath,encoding="UTF-8",xml_declaration=True)

        serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_cognitve_tests')          
        createTestReq=createCognitiveExerciseTestSrvRequest()
        createTestReq.test_type=d["testType"][0]
        createTestReq.test_difficulty=int(d["difficulty"][0])
        createTestReq.test_subtype=d["testSubType"][0]
        createTestReq.test_path=inNodeName
        createTestReq.supported_languages=supportedLanguages
        
        createTestReq.test_id=supportedLanguages=test_id

        
        knowrob_service = rospy.ServiceProxy(serv_topic, createCognitiveExerciseTestSrv)
        createCognitiveTestResponse = knowrob_service(createTestReq)
        if(createCognitiveTestResponse.success!=True):
          res.trace.extend(createCognitiveTestResponse.trace)
          res.error=createCognitiveTestResponse.error
          res.success=False
          os.remove(localPackagePath)
          return res
          
        ontologyName=createCognitiveTestResponse.test_name
        tmp=ontologyName.split("#")
        ontologyName=tmp[1]

        tree = ET.parse(localPackagePath)
        root = tree.getroot()
        ET.SubElement(root, "name").text = ontologyName
        self.indent(root)
        os.remove(localPackagePath)
        tree.write(localPackagePath,encoding="UTF-8",xml_declaration=True)
        res.success=True
        
      else:
        res.error="test "+fname +" is broken"
        res.success=False

    except IndexError:
      #print "test "+fname +" is broken"
      res.error="IndexError.. test "+fname +" is broken"
      res.success=False
    except IOError:
      #print "IO Error, cannot open test file or write xml file"
      res.error="IO Error, cannot open test file or write xml file"
      res.success=False
    except AppError as e:
      AppError.passErrorToRosSrv(e,res) 

    return res

  def determineCognitiveTestId(self, testType, testSubType, difficulty):
    serv_topic = rospy.get_param('rapp_cognitive_exercise_return_tests_of_type_subtype_difficulty_topic')
    returnTestsOfTypeSubtypeDifficultySrvReq=returnTestsOfTypeSubtypeDifficultySrvRequest()
    returnTestsOfTypeSubtypeDifficultySrvReq.testType=testType
    returnTestsOfTypeSubtypeDifficultySrvReq.testSubType=testSubType
    returnTestsOfTypeSubtypeDifficultySrvReq.difficulty=difficulty
    knowrob_service = rospy.ServiceProxy(serv_topic, returnTestsOfTypeSubtypeDifficultySrv)
    returnTestsOfTypeSubtypeDifficultySrvRes = knowrob_service(returnTestsOfTypeSubtypeDifficultySrvReq)  
    return returnTestsOfTypeSubtypeDifficultySrvRes.totalNumberOfTestsReturned




  ## @brief Checks if string can be converted to an integer
  # @param s [string] The input string
  #
  # @return bool [bool] True if string can be converted to an integer, otherwise false
  def is_int(self,s):
      try:
          int(s)
          return True
      except ValueError:
          return False
          
  ## @brief Formats the xml file so as to be human readable
  # @param elem [tree] The xml in element tree mode
  #
  # @return elem [tree] The xml in element tree mode
  def indent(self, elem, level=0):
      i = "\n" + level*"  "
      if len(elem):
          if not elem.text or not elem.text.strip():
              elem.text = i + "  "
          if not elem.tail or not elem.tail.strip():
              elem.tail = i
          for elem in elem:
              self.indent(elem, level+1)
          if not elem.tail or not elem.tail.strip():
              elem.tail = i
      else:
          if level and (not elem.tail or not elem.tail.strip()):
              elem.tail = i
