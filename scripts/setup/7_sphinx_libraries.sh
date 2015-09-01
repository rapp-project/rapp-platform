#download and compile sphinx4 extra libraries
echo -e "\e[1m\e[103m\e[31m [RAPP] Installing Sphinx4 Libraries \e[0m"
sudo apt-get -y install swig
sudo apt-get -y install autoconf
sudo apt-get -y install python-scipy
sudo apt-get -y install bison
sudo apt-get -y install sox 
cd $HOME/rapp_platform
git clone git@github.com:skerit/cmusphinx
cd cmusphinx/cmuclmtk
./autogen.sh
make
sudo make install

cd $HOME/rapp_platform
git clone https://github.com/cmusphinx/sphinxbase.git
cd sphinxbase
./autogen.sh
make
sudo make install
cd ~/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_speech_detection_sphinx4/src
bash buildJava.sh
