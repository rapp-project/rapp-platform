#download and compile sphinx4 extra libraries
sudo apt-get install swig
sudo apt-get install autoconf
sudo apt-get install python-scipy
cd ~/
git clone git@github.com:skerit/cmusphinx
cd ./cmusphinx/cmuclmtk
./autogen.sh
sudo make install
cd ..
cd multisphinx
./autogen.sh
sudo make install
cd ~/rapp-platform-catkin-ws/src/rapp-platform/rapp_speech_detection_sphinx4/src
bash buildJava.sh
