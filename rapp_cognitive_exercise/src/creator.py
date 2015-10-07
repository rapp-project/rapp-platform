#!/usr/bin/env python
import xml.etree.cElementTree as ET


class CognitiveExercise: 
  
  def foo(self):   
    
    fname="/home/thanos/Desktop/test"
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
              print "error, difficulty or variation ID is not an integer"
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
              correctAnswers[tmp[0]]=tmp[1]            
            else:
              flag=False
              print "field other than questions,answers,correctAnswer found"
              break
          else:                     
            d[tmp[0]]=[tmp[1]]
          
      if(not len(questions)==len(answers)==len(correctAnswers)):     
        flag=False   
        print "error, test is broken, questions-answers-correctAnswers not equal in size"
          
      
      if(flag):
        print "test is good"       
        ontologyName="ontName"
        xmlFileName=ontologyName+"_"+d["testType"][0]+"_"+d["testSubType"][0]+"_"+"diff"+d["difficulty"][0]+"_"+"var"+d["variationID"][0]
        print xmlFileName
        root = ET.Element("cognitiveTest")
        ET.SubElement(root, "name").text = ontologyName
        ET.SubElement(root, "testType").text = d["testType"][0]
        ET.SubElement(root, "variationID").text = d["variationID"][0]
        ET.SubElement(root, "difficulty").text = d["difficulty"][0]
        ET.SubElement(root, "testSubType").text = d["testSubType"][0]
        ET.SubElement(root, "Questions")
        Questions=ET.SubElement(root,"Questions")
        print "count "+str(count)
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
          if(corrAnswer not in answs):
            flag=False
            print "Correct Answer not in answers in Question "+str(i) +" in test "+fname
          print corrAnswer
          ET.SubElement(Q, "correctAnswer").text = corrAnswer  
                  
        tree = ET.ElementTree(root) 
        tree.write("/home/thanos/Desktop/filename.xml")
      if(not flag):
        print "test "+fname +" is broken"      
        
    except IndexError:
      print "test "+fname +" is broken"
    except IOError:
      print "IO Error, cannot open test file or write xml file"
     

      
      
    
    
    
  def is_int(self,s):
      try:
          int(s)
          return True
      except ValueError:
          return False

if __name__ == "__main__": 
  CognitiveExerciseNode = CognitiveExercise() 
  CognitiveExerciseNode.foo()
