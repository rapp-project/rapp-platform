#!/usr/bin/env python

from rapp_platform_ros_communications.srv import *

import rospy
import MySQLdb as mdb
import sys

class MySQLdbWrapper:

     def writePersonalData(self):
          con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');

          with con:
               
               cur = con.cursor()
               cur.execute("INSERT INTO Users(Name,Job) VALUES('Takis','ydraylikos')")
               cur.execute("INSERT INTO Users(Name,Job) VALUES('Sakis','hlektrologos')")
               cur.execute("SELECT * FROM Users")
  
               #rows = cur.fetchall()
               
               #for row in rows:
                    #print row

     def clearTable(self):
          con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');
          with con:
               cur=con.cursor()
               cur.execute("Truncate table Users")



     def fetchPersonalData(self,req):
          con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');

          with con:
                    
               cur = con.cursor()			
               cur.execute("SELECT * FROM Users WHERE Name = %s",req.a)
               s=str(cur.fetchone()[2])
               return s

     def createTable(self):
          con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');

          with con:
         
               cur = con.cursor()
               cur.execute("DROP TABLE IF EXISTS Users")
               cur.execute("CREATE TABLE Users(Id INT PRIMARY KEY AUTO_INCREMENT, \ Name VARCHAR(25), Job VARCHAR(25))")


     def checkConnection(self):
          try:
               con = mdb.connect('localhost', 'testuser', 'test623', 'testdb')
               cur = con.cursor()
               cur.execute("SELECT VERSION()")
               ver = cur.fetchone()
               print "Database version : %s " % ver
               #return true
          except mdb.Error, e:
               print "Error %d: %s" % (e.args[0],e.args[1])
               #sys.exit(1)
               #return false
          finally:    
               if con:    
                    con.close()


     def DbServer(self):
          rospy.init_node('MySQLWrapper')
          s = rospy.Service('MySQLWrapperService', DB, self.fetchPersonalData)          
          self.checkConnection()
          rospy.spin()


db_wrapper=MySQLdbWrapper()
db_wrapper.DbServer()
