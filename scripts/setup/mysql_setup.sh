#!/bin/bash 
pth=$(pwd)
a="/schemaOnlyMyslDump.sql"
pth=$pth$a
echo -e "\e[1m\e[103m\e[31m [RAPP] MySQL setup \e[0m"

# Setup sources list
sudo apt-get -y install mysql-client mysql-server
sudo apt-get -y install python-mysqldb
sudo printf "dummyUser\changeMe" > /etc/db_credentials1
echo "Insert MySQL root Passowrd"
echo "create database RappStore" | mysql -u root -p
echo "Insert MySQL root Passowrd"
mysql -u root -p RappStore < $pth
echo "Creating dummyUser with username = 'dummyUser' and password = 'changeMe' and granting all on RappStore DB"
echo "Insert MySQL root Passowrd"
echo "CREATE USER 'dummyUser'@'localhost' IDENTIFIED BY 'changeMe'" | mysql -u root -p
echo "Insert MySQL root Passowrd"
echo "GRANT ALL ON RappStore.* TO 'dummyUser'@'localhost'" | mysql -u root -p
echo "If promted insert root sudo password"
sudo printf "dummyUser\nchangeMe" >/etc/db_credentials
