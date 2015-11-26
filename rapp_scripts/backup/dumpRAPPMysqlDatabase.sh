#!/bin/bash

DATE=`date +%Y-%m-%d:%H:%M:%S`
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

mysqldump -u $username -p$password RappStore > $HOME/mysqlBckup_$DATE
