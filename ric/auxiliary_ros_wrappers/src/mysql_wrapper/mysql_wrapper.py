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
    print "start"
    self.serv=rospy.Service('ric/mysql_wrapper_service', DbWrapperSrv, self.DbHandler)
    
    
  def getColumnNames(self):
    db_username,db_password=self.getLogin()
    print "dunato"
    try:
      con = mdb.connect('localhost', db_username, db_password, 'ric_db');            
      cur = con.cursor()
      cur.execute("Show columns from users")
      #s=str(cur.fetchone()[0])
      result_set = cur.fetchall()
      Columns=[]
      for row in result_set:
        print row[0]
        Columns=Columns+[row[0]]
      print Columns[0:]
      
      
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

  #def writePersonalData(self):     
      #TO BE DEVELOPED
      
      #con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');               
      #cur = con.cursor()
      #cur.execute("INSERT INTO users(Name,Job) VALUES('Takis','ydraylikos')")
      #cur.execute("INSERT INTO Users(Name,Job) VALUES('Sakis','hlektrologos')")
      #cur.execute("SELECT * FROM users")  

  def fetchPersonalData(self,req,res):
    db_username,db_password=self.getLogin()        
    try:  
      con = mdb.connect('localhost', db_username, db_password, 'ric_db');            
      cur = con.cursor()
      ttt1="test_name"
      t2="users"	
      
      returncols=req.return_cols[0].data+","+req.return_cols[1].data+","+req.return_cols[2].data
      print returncols
      
      
      query=req.req_data[0].s[0].data+"=\""+req.req_data[0].s[1].data+"\""+" AND "+req.req_data[1].s[0].data+"=\""+req.req_data[1].s[1].data+"\""
      print query
      st1="SELECT "+returncols+" FROM tblUser WHERE "+query
      print st1
      cur.execute(st1)
      #s=str(cur.fetchone()[0])
      result_set = cur.fetchall()     
      res.res_data=[]
      for i in range(len(result_set)):
        line=StringArrayMsg()
        line=[]        
        for j in range(len(result_set[i])):
          temp_s=result_set[i][j]
          print temp_s
          line=line+[String(data=temp_s)]
          #print line.s[0].data
        res.res_data=res.res_data+ [StringArrayMsg(s=line)]
        #res.res_data=res.res_data+[line]
          
          
      #line=StringArrayMsg() 
      #line=[String(data="1"), String(data="2"), String(data="3")]
      #res.res_data=res.res_data+[StringArrayMsg(s=line)]
      
      
      
      
      #s=str(cur.fetchone()[2])
      #print s
      con.close()
      return res
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])           

  def checkConnection(self):    
    db_username,db_password=self.getLogin() 
    try:
      con = mdb.connect('localhost', db_username, db_password, 'ric_db')
      cur = con.cursor()
      cur.execute("SELECT VERSION()")
      ver = cur.fetchone()
      print "Database version : %s " % ver
      con.close()

    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])
      
  def DbHandler(self,req):
    
    print "waiting to receive"
      
    
    print "received something"
    res = DbWrapperSrvResponse()
    db_username,db_password=self.getLogin()        
    try:  
      con = mdb.connect('localhost', db_username, db_password, 'RappStore');            
      cur = con.cursor()
      ttt1="test_name"
      t2="users"	
      
      returncols=req.return_cols[0].data+","+req.return_cols[1].data+","+req.return_cols[2].data
      print returncols
      
      
      query=req.req_data[0].s[0].data+"=\""+req.req_data[0].s[1].data+"\""+" AND "+req.req_data[1].s[0].data+"=\""+req.req_data[1].s[1].data+"\""
      print query
      st1="SELECT "+returncols+" FROM tblUser WHERE "+query
      print st1
      cur.execute(st1)
      #s=str(cur.fetchone()[0])
      result_set = cur.fetchall()     
      #res.res_data=[]
      for i in range(len(result_set)):
        line=StringArrayMsg()
        #line=[]        
        for j in range(len(result_set[i])):
          temp_s=String(result_set[i][j])
          print temp_s
          line.s.append(String(data=temp_s))#=line.s+[String(data=temp_s)]
          #print line.s[0].data
        #res.res_data=res.res_data+ [StringArrayMsg(s=line)]
        print "skaei"
        print res.res_data
        print line
        #res.res_data=res.res_data+[line.s]
        res.res_data.append(line)
          
          
      #line=StringArrayMsg() 
      #line=[String(data="1"), String(data="2"), String(data="3")]
      #res.res_data=res.res_data+[StringArrayMsg(s=line)]
      
      
      
      
      #s=str(cur.fetchone()[2])
      #print s
      con.close()
      return res
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])   
    
    print "returned"
    myli=["1","2","3"]
    #mk.data=True
    
    
    
    res.success.data=True
    res.res_cols=req.return_cols
    print "after true"
    print res.res_data[0].s[0].data
    print "last true"
    #res.res_cols=req.return_cols
    return res      
    #self.checkConnection()
    #rospy.spin()

#db_wrapper=MySQLdbWrapper()
#db_wrapper.DbServer()

if __name__ == "__main__": 
  rospy.init_node('MySQLWrapper') 
  #db_wrapper=MySQLdbWrapper()
  MySQLWrapperNode = MySQLdbWrapper() 
  #MySQLWrapperNode.getColumnNames()
  rospy.spin()
