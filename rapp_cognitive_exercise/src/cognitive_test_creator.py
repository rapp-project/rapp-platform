#!/usr/bin/env python
import xml.etree.cElementTree as ET
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
  createCognitiveExerciseTestSrvRequest
  )
  
from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  )

class CognitiveTestCreator: 
  
  def testCreator(self,req):   
    res=cognitiveTestCreatorSrvResponse()
    fname=req.inputFile
    with open(fname) as f:
      content = f.readlines()
      
    d=dict()
    questions=dict()
    answers=dict()
    correctAnswers=dict()
    flag=False
    questionsStart=False
    count=0
    try:
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
        xmlFileName=d["testType"][0]+"_"+d["testSubType"][0]+"_"+"diff"+d["difficulty"][0]+"_"+"var"+d["variationID"][0]+".xml"
        


        
        #print xmlFileName
        root = ET.Element("cognitiveTest")
        #ET.SubElement(root, "name").text = ontologyName
        ET.SubElement(root, "testType").text = d["testType"][0]
        ET.SubElement(root, "variationID").text = d["variationID"][0]
        ET.SubElement(root, "difficulty").text = d["difficulty"][0]
        ET.SubElement(root, "testSubType").text = d["testSubType"][0]
        ET.SubElement(root, "Questions")
        Questions=ET.SubElement(root,"Questions")
        #print "count "+str(count)
        for i in range(1,count+1):
          nm="Q"+str(i)
          #ET.SubElement(Questions, "Question", name=nm)
          Q=ET.SubElement(Questions, "Question", name=nm)
          nm="question"+str(i)
          #print nm
          #print questions[nm]
          ET.SubElement(Q, "body").text = questions[nm][0]
          nm="answers"+str(i)
          answs=answers[nm].split(",")
          for j in answs:
            A=ET.SubElement(Q, "answer")
            ET.SubElement(A, "body").text = j
          nm="correctAnswer"+str(i)
          corrAnswer=correctAnswers[nm]  
          ET.SubElement(Q, "correctAnswer").text = corrAnswer  
                  
        tree = ET.ElementTree(root) 
        rospack = rospkg.RosPack()
        localPackagePath=rospack.get_path('rapp_cognitive_exercise')
        localPackagePath=localPackagePath+"/cognitiveTests/"+xmlFileName
        tree.write(localPackagePath)
        
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
        createTestReq.test_variation=int(d["variationID"][0])
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
        tree.write(localPackagePath)
        
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
      es.success=False
      
    return res
         
  def is_int(self,s):
      try:
          int(s)
          return True
      except ValueError:
          return False
