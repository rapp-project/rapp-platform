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

from rapp_platform_ros_communications.srv import *
import rospy
import MySQLdb as mdb
import sys

class MySQLdbWrapper:

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
      sys.exit(1)

  #def writePersonalData(self):     
      #TO BE DEVELOPED
      
      #con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');               
      #cur = con.cursor()
      #cur.execute("INSERT INTO users(Name,Job) VALUES('Takis','ydraylikos')")
      #cur.execute("INSERT INTO Users(Name,Job) VALUES('Sakis','hlektrologos')")
      #cur.execute("SELECT * FROM users")  

  def fetchPersonalData(self,req):
    db_username,db_password=self.getLogin()        
    try:  
      con = mdb.connect('localhost', db_username, db_password, 'ric_db');            
      cur = con.cursor()
      ttt1="test_name"			
      cur.execute("SELECT surname FROM users WHERE name = %s",req.a)
      s=str(cur.fetchone()[0])
      #s=str(cur.fetchone()[2])
      print s
      con.close()
      return s
    except mdb.Error, e:
      print "Error %d: %s" % (e.args[0],e.args[1])
      sys.exit(1)      

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
      sys.exit(1)

  def DbServer(self):
    rospy.init_node('MySQLWrapper')
    s = rospy.Service('ric/mysql_wrapper_service', DB, self.fetchPersonalData)          
    self.checkConnection()
    rospy.spin()

db_wrapper=MySQLdbWrapper()
db_wrapper.DbServer()

