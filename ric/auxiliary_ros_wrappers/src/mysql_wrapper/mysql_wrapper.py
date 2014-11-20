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
    self.serv=rospy.Service('ric/db/mysql_wrapper_service', DbWrapperSrv, self.fetchPersonalDataHandler)
    
  def constructCommaColumns(self,cols):
    if(len(cols)==0):
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
    for i in range(len(cols)):
      if i==0:
        returnquery=returnquery+cols[i].s[0].data+"=\""+cols[i].s[1].data+"\""
      else:          
        returnquery=returnquery+" AND "+cols[i].s[0].data+"=\""+cols[i].s[1].data+"\""
    return returnquery 
    
  def getColumnNames(self):
    db_username,db_password=self.getLogin()   
    try:
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
      cur = con.cursor()
      cur.execute("Show columns from tblUser")
      #s=str(cur.fetchone()[0])
      result_set = cur.fetchall()
      Columns=[]
      for row in result_set:
        #print row[0]
        Columns=Columns+[String(data=str(row[0]))]
      #print Columns[0:]
      return Columns 
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])
    

  def getLogin(self):
    try:
      fh = open("/etc/db_credentials", "r")
      db_username=fh.readline()
      db_username=db_username.split( )[0]
      db_password=fh.readline()
      db_password=db_password.split()[0]
      return db_username,db_password
    except IOError:
      print "Error: can\'t find file or read data"      
           
  def checkConnection(self):    
    db_username,db_password=self.getLogin() 
    try:
      con = mdb.connect('localhost', db_username, db_password, 'RappStore')
      cur = con.cursor()
      cur.execute("SELECT VERSION()")
      ver = cur.fetchone()
      print "Database version : %s " % ver
      con.close()

    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])
      
  def fetchPersonalDataHandler(self,req): 
    res = DbWrapperSrvResponse()
    db_username,db_password=self.getLogin()        
    try:  
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
      cur = con.cursor()
      returncols=self.constructCommaColumns(req.return_cols)
      print returncols
      self.getColumnNames()
      where=self.constructAndQuery(req.req_data)
      print where
      query="SELECT "+returncols+" FROM tblUser WHERE "+where
      print query
      cur.execute(query)  
      result_set = cur.fetchall()     

      for i in range(len(result_set)):
        line=StringArrayMsg()       
        for j in range(len(result_set[i])):
          temp_s=String(result_set[i][j])          
          line.s.append(String(data=str(result_set[i][j])))#=line.s+[String(data=temp_s)]
        res.res_data.append(line)
   
      con.close()
      res.report.data="Operation Successful"
      if (returncols=="*"):
        res.res_cols=self.getColumnNames()
      else:
        res.res_cols=req.return_cols
      print "Operation Successful"
      return res
    except mdb.Error, e:
      res.report.data= "Error %d: %s" % (e.args[0],e.args[1]) 
      print "Error %d: %s" % (e.args[0],e.args[1]) 

    return res      


if __name__ == "__main__": 
  rospy.init_node('MySQLWrapper')
  MySQLWrapperNode = MySQLdbWrapper() 
  rospy.spin()
