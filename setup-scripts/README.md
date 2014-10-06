Setup scripts
=============

These scripts can be executed after a clean Ubuntu 14.04 installation, in order
to install the appropriate packages and setup the environment.

Step 1 - Updates / ROS install / HOP install
--------------------------------------------

The first step performs the initial system updates, installs ROS Indigo and
the latest HOP distribution. You can execute it by:

```
sudo sh 1-clean-install.sh
```

Step 1.5 - Setup public key for Github
--------------------------------------

Since out Github repositories are private, a public key to Github should be 
set in order for you to be able to clone the repositories. This can be 
performed following the instructions here:

https://help.github.com/articles/generating-ssh-keys/

Step 2 - Setup Github repositories
----------------------------------

This step sets up the RAPP Github repositories destined for the platform. You 
can execute it by running:

```
sudo sh 2-git-repos-setup.sh
```
