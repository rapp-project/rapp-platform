#!/usr/bin/env python

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


import rospy
import MySQLdb as mdb
import sys

from rapp_platform_ros_communications.srv import (
  DbWrapperSrv,
  DbWrapperSrvResponse
  )
  
from rapp_platform_ros_communications.msg import ( 
  StringArrayMsg 
  ) 
    
from std_msgs.msg import ( 
  String 
  ) 

class MySQLdbWrapper: 
  
  def __init__(self):   
    self.serv=rospy.Service('ric/db/mysql_wrapper_service/fetchPersonalData', DbWrapperSrv, self.fetchPersonalDataHandler)
    self.serv=rospy.Service('ric/db/mysql_wrapper_service/writePersonalData', DbWrapperSrv, self.writePersonalDataHandler)
    self.serv=rospy.Service('ric/db/mysql_wrapper_service/deletePersonalData', DbWrapperSrv, self.deletePersonalDataHandler)
    self.serv=rospy.Service('ric/db/mysql_wrapper_service/updatePersonalData', DbWrapperSrv, self.updatePersonalDataHandler)
  
  def writeData(self,req,tblName):
    res = DbWrapperSrvResponse()
    db_username,db_password=self.getLogin()
    con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
    cur = con.cursor()
    returncols=self.constructCommaColumns(req.return_cols)
    if (len(returncols)>1):
      returncols="("+returncols+")"
    print returncols
    values=""
    for i in range(len(req.req_data)):
      if (i==0):
        values=values+"("+self.constructCommaColumns(req.req_data[i].s)+")"
      else:
        values=values+",("+self.constructCommaColumns(req.req_data[i].s)+")"
    
    print values      
    query="Insert into "+tblName+" "+ returncols+" values "+values
    print query      
    cur.execute("LOCK TABLES "+tblName+" WRITE")
    cur.execute(query)
    cur.execute("UNLOCK TABLES")
    return res
      
  def deleteData(self,req,tblName):
    res = DbWrapperSrvResponse()
    db_username,db_password=self.getLogin()
    con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
    cur = con.cursor()
    where=self.constructAndQuery(req.req_data)
    query="Delete from "+tblName+where
    cur.execute("LOCK TABLES "+tblName+" WRITE")
    cur.execute(query)
    cur.execute("UNLOCK TABLES")
    return res
    
  def updateData(self,req,tblName):
    res = DbWrapperSrvResponse()
    db_username,db_password=self.getLogin()
    con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
    cur = con.cursor()
    returncols=self.constructCommaColumns(req.return_cols)
    where=self.constructAndQuery(req.req_data)
    query="Update "+tblName+" SET "+returncols+where
    print query
    cur.execute("LOCK TABLES "+tblName+" WRITE")
    cur.execute(query)
    cur.execute("UNLOCK TABLES")
    return res
    
  def fetchData(self,req,tblName):
    res = DbWrapperSrvResponse()
    db_username,db_password=self.getLogin()
    con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
    cur = con.cursor()
    returncols=self.constructCommaColumns(req.return_cols)
    #print returncols            
    where=self.constructAndQuery(req.req_data)          
    #print where
    query="SELECT "+returncols+" FROM "+tblName+where
    #print query
    cur.execute(query)  
    result_set = cur.fetchall() 
    for i in range(len(result_set)):
      line=StringArrayMsg()       
      for j in range(len(result_set[i])):
        temp_s=String(result_set[i][j])          
        line.s.append(String(data=str(result_set[i][j])))#=line.s+[String(data=temp_s)]
      res.res_data.append(line)
 
    con.close()
    if (returncols=="*"):
      res.res_cols=self.getTableColumnNames(tblName)
    else:
      res.res_cols=req.return_cols
    return res
    
  def constructCommaColumns(self,cols):        
    if (len(cols)<1):
      #print "return cols empty"
      return ""
      
    elif (cols[0].data=="*"):      
      return "*"      
    else:
      returncols=""
      for i in range(len(cols)):
          if i==0:
            returncols=returncols+cols[i].data
          else:
            returncols=returncols+","+cols[i].data
      return returncols 
    
  def constructAndQuery(self,cols):    
    returnquery=""
    if(len(cols)==0):
      return ""
    else:  
      for i in range(len(cols)):
        if i==0:
          returnquery=returnquery+cols[i].s[0].data+"=\""+cols[i].s[1].data+"\""
        else:          
          returnquery=returnquery+" AND "+cols[i].s[0].data+"=\""+cols[i].s[1].data+"\""
      returnquery=" WHERE "+returnquery
      return returnquery 
    
  def getTableColumnNames(self,tblName):
    db_username,db_password=self.getLogin()   
    try:
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
      cur = con.cursor()
      cur.execute("Show columns from "+tblName)
      result_set = cur.fetchall()
      Columns=[]
      for row in result_set: 
        Columns=Columns+[String(data=str(row[0]))]
      return Columns 
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])
    

  def getLogin(self):
    fh = open("/etc/db_credentials", "r")
    db_username=fh.readline()
    db_username=db_username.split( )[0]
    db_password=fh.readline()
    db_password=db_password.split()[0]
    return db_username,db_password
     
           
  def checkConnection(self):        
    try:
      db_username,db_password=self.getLogin() 
      con = mdb.connect('localhost', db_username, db_password, 'RappStore')
      cur = con.cursor()
      cur.execute("SELECT VERSION()")
      ver = cur.fetchone()
      print "Database version : %s " % ver
      con.close()
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])
      
  def fetchPersonalDataHandler(self,req):   
    print "fetch called"            
    try:      
      res = DbWrapperSrvResponse()
      res=self.fetchData(req,"tbluser")
      res.success.data=True
      res.report.data="Success"
    except mdb.Error, e:
      res.report.data= "Database Error %d: %s" % (e.args[0],e.args[1])
      res.success.data=False
      print "Error %d: %s" % (e.args[0],e.args[1]) 
    except IndexError:
      res.report.data= "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
      res.success.data=False
    except IOError:
      print "Error: can\'t find login file or read data" 
      res.success.data=False
      res.report.data="Error: can\'t find login file or read data" 
    return res      

  def writePersonalDataHandler(self,req):
    print "write called"             
    try:
      res = DbWrapperSrvResponse()
      res=self.writeData(req,"tbluser")
      res.success.data=True
      res.report.data="Success"
    except mdb.Error, e:
      res.report.data= "Database Error %d: %s" % (e.args[0],e.args[1])
      res.success.data=False
      print "Error %d: %s" % (e.args[0],e.args[1]) 
    except IndexError:
      print "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
      res.report.data= "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
      res.success.data=False
    except IOError:
      print "Error: can\'t find login file or read data" 
      res.success.data=False
      res.report.data="Error: can\'t find login file or read data"
    return res  
    
  def deletePersonalDataHandler(self,req):
    print "delete called"
    try:
      res = DbWrapperSrvResponse()
      res=self.deleteData(req,"tbluser")
      res.success.data=True
      res.report.data="Success"
    except mdb.Error, e:
      res.report.data= "Database Error %d: %s" % (e.args[0],e.args[1])
      res.success.data=False
      print "Error %d: %s" % (e.args[0],e.args[1])
    except IndexError:
      print "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
      res.report.data= "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
      res.success.data=False
    except IOError:
      print "Error: can\'t find login file or read data" 
      res.success.data=False
      res.report.data="Error: can\'t find login file or read data"
    return res  
    
  def updatePersonalDataHandler(self,req):
    print "update called"
    try:
      res = DbWrapperSrvResponse()
      res=self.updateData(req,"tbluser")
      res.success.data=True
      res.report.data="Success"      
    except mdb.Error, e:
      res.report.data= "Database Error %d: %s" % (e.args[0],e.args[1])
      res.success.data=False
      print "Error %d: %s" % (e.args[0],e.args[1])     
    except IndexError:
      print "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
      res.report.data= "Wrong Query Input Format, check for empty required columns list or wrong/incomplete Query data format"
      res.success.data=False
    except IOError:
      print "Error: can\'t find login file or read data" 
      res.success.data=False
      res.report.data="Error: can\'t find login file or read data"
    return res
    
if __name__ == "__main__": 
  rospy.init_node('MySQLWrapper')
  MySQLWrapperNode = MySQLdbWrapper() 
  rospy.spin()
