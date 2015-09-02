#Deploy scripts

The purpose of these scripts is to deploy RAPP-Platform.

##Repositories preparation

##Deployment 

There are two files aimed for deployment:

- ```deploy_rapp_ros.sh```: Deploys the RAPP Platform back-end, i.e. all the ROS nodes
- ```deploy_hop_services.sh```: Deploys the corresponding HOP services

Follow the next steps to deploy the RAPP Platform software:

- ```screen```
- ```./deploy_rapp_ros.sh```
- Press Ctrl + a + d to detach
- ```screen```
- ```./deploy_hop_services```
- Press Ctrl + a + d to detach
- ```screen -ls``` to check that 2 screen sessions exist

To reattach to screen session:
```
screen -r [pid.]tty.host
```

Screen how-to: http://www.rackaid.com/blog/linux-screen-tutorial-and-how-to/

