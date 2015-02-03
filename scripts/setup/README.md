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

You can execute it by:

```
sh clean_install.sh
```

##Todo

- HOP installation / setup
- MySQL installation / setup
