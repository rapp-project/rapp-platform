#!/bin/bash 

path=$(pwd)
dump="/RappPlatformMySqlDbSchema.sql"
pth=$path$dump
echo -e "\e[1m\e[103m\e[31m [RAPP] MySQL RAPP database import \e[0m"

sudo printf "dummyUser\changeMe" > /etc/db_credentials1
echo "Insert MySQL root Password"
echo "Create database RappStore" | mysql -u root -p
echo "Insert MySQL root Password"
mysql -u root -p RappStore < $pth


