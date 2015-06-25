#!/bin/bash

i=0;
for j in `cat /etc/db_credentials` 
do
   array[$i]=$j; 
   #do manipulations here
    i=$(($i+1)); 
   
done 
echo "Value of third element in my array : ${array[1]} ";
username=${array[0]};
password=${array[1]};

echo "enter root mysql password"
echo "drop database RappStore" | mysql -u root -p
echo "enter root mysql password"
echo "create database RappStore" | mysql -u root -p
echo "enter root mysql password"
mysql -u root -p RappStore < $HOME/$1
