#!/bin/bash 
pth=$(pwd)
a="/schemaOnlyMyslDump.sql"
pth=$pth$a
#echo $pth

echo -e "\e[1m\e[103m\e[31m [RAPP] MySQL setup \e[0m"
# Setup sources list
#echo $1
sudo apt-get install mysql-client mysql-server
sudo apt-get install python-mysqldb
echo "create database RappStore" | mysql -u root -p
mysql -u root -p RappStore < $pth

