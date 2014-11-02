#!/usr/bin/python
# -*- coding: utf-8 -*-

#we want a package



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



def fetshPersonalData(name):
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

  

checkConnection()
clearTable()
createTable()
writePersonalData()
fetshPersonalData("Takis")

