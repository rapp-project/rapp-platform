WARNING - READ FIRST

Directories: 
`php` contains all php files, some are standalone php, some use html as well
`css` contains all CSS stylesheets
`db`  contains sql dumps, used for backups of the store whilst being developed
`scripts` contains all javascript files
`userdir` is a sandbox which contains automatically generated directories for storing user-submitted files for RAPPS


NOTE - The store assumes to be running on RAPP-VM, a VirtualBox with ROS, Apache2, MySQL (headless or not).

NOTE - For cross-compilation (arm-based-robots) we need a Qemu/Chroot enviroment.

I am in the process of setting up, within the RAPP-VM, a QEMU ARM chroot environment.
In order to do so, I will try to use Linaro images: http://releases.linaro.org/14.09/ubuntu/trusty-images/developer

The goal, is to be able to compile gcc-4.8 c++11 RAPPs, for ARM cpus (INRIA)
Since NAO uses i386, I should be able to compile without the need for QEMU, albeit NAO uses a VM for its SDK.


