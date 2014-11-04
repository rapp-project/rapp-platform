#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy
import MySQLdb as mdb
import sys

def writePersonalData():
	con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');

	with con:
		
		cur = con.cursor()
		cur.execute("INSERT INTO Users(Name,Job) VALUES('Takis','ydraylikos')")
		cur.execute("INSERT INTO Users(Name,Job) VALUES('Sakis','hlektrologos')")
		cur.execute("SELECT * FROM Users")
		rows = cur.fetchall()

		for row in rows:
			print row

def clearTable():
	con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');
	with con:
		cur=con.cursor()
		cur.execute("Truncate table Users")



def fetchPersonalData(name):
	con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');

	with con:
			
		cur = con.cursor()			
		cur.execute("SELECT * FROM Users WHERE Name = %s",name)
		rows = cur.fetchall()
		for row in rows:
			print row

def createTable():
	con = mdb.connect('localhost', 'testuser', 'test623', 'testdb');

	with con:
    
		cur = con.cursor()
		cur.execute("DROP TABLE IF EXISTS Users")
		cur.execute("CREATE TABLE Users(Id INT PRIMARY KEY AUTO_INCREMENT, \
					 Name VARCHAR(25), Job VARCHAR(25))")
	     
		
	
	




def checkConnection():
	try:
		con = mdb.connect('localhost', 'testuser', 'test623', 'testdb')
		cur = con.cursor()
		cur.execute("SELECT VERSION()")
		ver = cur.fetchone()
		print "Database version : %s " % ver
	except mdb.Error, e:
		print "Error %d: %s" % (e.args[0],e.args[1])
		#sys.exit(1)
	finally:    
		if con:    
			con.close()





def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    fetchPersonalData(req.a)
    return DBResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', DB, handle_add_two_ints)
    print "Ready to add two ints."
    checkConnection()
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
