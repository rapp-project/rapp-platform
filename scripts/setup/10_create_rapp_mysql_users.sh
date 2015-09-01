#!/bin/bash 

echo -e "\e[1m\e[103m\e[31m [RAPP] Create MySQL RAPP user \e[0m"

echo "Creating dummyUser with username = 'dummyUser' and password = 'changeMe' and granting all on RappStore DB"
echo "Insert MySQL root Password"
echo "CREATE USER 'dummyUser'@'localhost' IDENTIFIED BY 'changeMe'" | mysql -u root -p
echo "Insert MySQL root Password"
echo "GRANT ALL ON RappStore.* TO 'dummyUser'@'localhost'" | mysql -u root -p
echo "If prompted insert root sudo password"
sudo sh -c 'printf "dummyUser\nchangeMe" >/etc/db_credentials'
