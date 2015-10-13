#!/usr/bin/env python
# -*- encode: utf-8 -*-


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

# Author: Athanassios Kintsakis
# contact: akintsakis@issel.ee.auth.gr


import xml.etree.ElementTree as ET
import rospkg
import rospy
import os
import sys


from rapp_platform_ros_communications.srv import (
  cognitiveTestCreatorSrv,
  cognitiveTestCreatorSrvResponse,
  cognitiveTestCreatorSrvRequest,
  createCognitiveExerciseTestSrv,
  createCognitiveExerciseTestSrvResponse,
  createCognitiveExerciseTestSrvRequest,
  cognitiveTestsOfTypeSrv,
  cognitiveTestsOfTypeSrvRequest,
  cognitiveTestsOfTypeSrvResponse
  )
  
from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  )

class CognitiveTestCreator: 
  
  def testCreator(self,req):   
    res=cognitiveTestCreatorSrvResponse()
    fname=req.inputFile
    
      
    d=dict()
    questions=dict()
    answers=dict()
    correctAnswers=dict()
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
          print tmp[1]
          if(tmp[0] == "Difficulty" or tmp[0]=="variationID"):
            if (not self.is_int(tmp[1])):               
              res.trace.append("error, difficulty or variation ID is not an integer")
              flag=False
              
          if(tmp[0] == "question"):
            questionsStart=True
            flag=True
            
          if(questionsStart):                 
            if(tmp[0] == "question" and questionsStart):
              count=count+1;
            if(tmp[0] == "question" and questionsStart):
              tmp[0]=tmp[0]+str(count)
              questions[tmp[0]]=tmp[1]
            elif(tmp[0]=="answers" and questionsStart):
              tmp[0]=tmp[0]+str(count)
              answers[tmp[0]]=tmp[1]
            elif(tmp[0]=="correctAnswer" and questionsStart):
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
          
      xmlFileName=""
      if(flag):        
        #res.trace.append("test seems good")
        #xmlFileName=d["testType"][0]+"_"+d["testSubType"][0]+"_"+"diff"+d["difficulty"][0]+"_"+"var"+d["variationID"][0]+".xml"
        xmlFileName=d["testType"][0]+"_"+d["testSubType"][0]+"_"+"diff"+d["difficulty"][0]+"_"+"var"+".xml"


        
        #print xmlFileName
        root = ET.Element("cognitiveTest")
        #ET.SubElement(root, "name").text = ontologyName
        ET.SubElement(root, "testType").text = d["testType"][0]
        #ET.SubElement(root, "variationID").text = d["variationID"][0]
        ET.SubElement(root, "difficulty").text = d["difficulty"][0]
        ET.SubElement(root, "testSubType").text = d["testSubType"][0]
        #ET.SubElement(root, "Questions")
        Questions=ET.SubElement(root,"Questions")
        #print "count "+str(count)
        for i in range(1,count+1):
          nm="Q"+str(i)
          #ET.SubElement(Questions, "Question", name=nm)
          Q=ET.SubElement(Questions, "Question", name=nm)
          nm="question"+str(i)
          #print nm
          #print questions[nm]
          ET.SubElement(Q, "body").text = questions[nm].decode('UTF-8')#[0]
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
        localPackagePath=localPackagePath+"/cognitiveTests/"+xmlFileName
        tree.write(localPackagePath,encoding="UTF-8",xml_declaration=True)
        
        #get the cognitive test_id
        serv_topic = rospy.get_param('rapp_knowrob_wrapper_cognitive_tests_of_type')
        if(not serv_topic):
          rospy.logerror("rapp_knowrob_wrapper_cognitive_tests_of_type not found")
          res.trace.append("rapp_knowrob_wrapper_cognitive_tests_of_type not found")
          res.error="rapp_knowrob_wrapper_cognitive_tests_of_type not found"
          res.success=False
          return res
        cognitiveTestsOfTypeSrvReq=cognitiveTestsOfTypeSrvRequest()
        cognitiveTestsOfTypeSrvReq.test_type=d["testType"][0]
        knowrob_service = rospy.ServiceProxy(serv_topic, cognitiveTestsOfTypeSrv)       
        cognitiveTestsOfTypeResponse = knowrob_service(cognitiveTestsOfTypeSrvReq)
        
        calculatedVariationID=1
        if(cognitiveTestsOfTypeResponse.success==True):     
          maxidlist=[]
          for test_i in range(len(cognitiveTestsOfTypeResponse.subtype)):
            tmp=cognitiveTestsOfTypeResponse.subtype[test_i].split("#")
            tmpSubtype=tmp[1]            
            if (tmpSubtype==d["testSubType"][0] and cognitiveTestsOfTypeResponse.difficulty[test_i]==d["difficulty"][0]):
              maxidlist.append(cognitiveTestsOfTypeResponse.variation[test_i])
          print maxidlist
          if (len(maxidlist)>0):
            maxidlist=map(int, maxidlist)
            calculatedVariationID=max(maxidlist)+1
          #print calculatedVariationID           
        
        #
        
        serv_topic = rospy.get_param('rapp_knowrob_wrapper_create_cognitve_tests')
        if(not serv_topic):
          rospy.logerror("rapp_knowrob_wrapper_create_cognitve_tests not found")
          res.trace.append("rapp_knowrob_wrapper_create_cognitve_tests not found")
          res.error="rapp_knowrob_wrapper_create_cognitve_tests not found"
          res.success=False
          os.remove(localPackagePath)
          return res
        createTestReq=createCognitiveExerciseTestSrvRequest()
        createTestReq.test_type=d["testType"][0]
        createTestReq.test_variation=calculatedVariationID#int(d["variationID"][0])
        createTestReq.test_difficulty=int(d["difficulty"][0])
        createTestReq.test_subtype=d["testSubType"][0]
        createTestReq.test_path="/cognitiveTests/"+xmlFileName
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
        localPackagePath=localPackagePath+"_var"+str(calculatedVariationID)+"_"+ontologyName
        tree.write(localPackagePath,encoding="UTF-8",xml_declaration=True)
        
        res.success=True
      if(not flag):
        #print "test "+fname +" is broken" 
        res.error="test "+fname +" is broken" 
        res.success=False     
        
    except IndexError:
      #print "test "+fname +" is broken"
      res.error="test "+fname +" is broken"
      res.success=False
    except IOError:
      #print "IO Error, cannot open test file or write xml file"
      res.error="IO Error, cannot open test file or write xml file"
      res.success=False
      
    return res
  
  def is_int(self,s):
      try:
          int(s)
          return True
      except ValueError:
          return False
          
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
