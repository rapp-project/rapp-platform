#Setup scripts

These scripts can be executed after a clean Ubuntu 14.04 installation, in order
to install the appropriate packages and setup the environment.

##Step 0 - Github keys setup

Since our Github repositories are private, a public key to Github should be 
set in order to be able to clone the repositories. This can be performed by 
following the instructions:

https://help.github.com/articles/generating-ssh-keys/

If you already have a public/private key, copy it in the ```.ssh``` folder
of your new installation.

##Updates / ROS install / Github repos
--------------------------------------------

Performs:
- initial system updates 
- installs ROS Indigo 
- downloads all Github repos needed

Execute the following scripts IN THE ORDER provided. Follow in between steps.

```
- sh system_updates.sh
- sh ros_setup.sh
- !Restart system
- sh rosjava.sh
- !Restart system
- sh knowrob.sh
- sh rapp_platform_setup.sh
- sh mysql_setup.sh
- sh mysql_setup.sh
```

##What you must do manually

#create a new user, and grant all privileges on RappStore DB to him. Pick your username and password. run the following commands manually
mysql -u root -p (you will be asked for your root password)
GRANT USAGE ON *.* TO <username>@localhost IDENTIFIED BY '<password>';
GRANT ALL PRIVILEGES ON RappStore.* TO <username>@localhost;
FLUSH PRIVILEGES;
exit

#You must create a file named ... and store it in /etc/... which will have in it's first line the username and it's second line the password
example file:
username1
password1

