run.sh needs cmuclmtk to run: http://cmusphinx.sourceforge.net/wiki/cmuclmtkdevelopment

Create a directory and execute this in it:
```
wget -e robots=off --cut-dirs=3 --user-agent=Mozilla/5.0 --reject="index.html*" --no-parent --recursive --relative --level=5 https://svn.code.sf.net/p/cmusphinx/code/trunk/cmuclmtk/
cd svn.code.sf.net/trunk/cmuclmtk
chmod +x autogen.sh
./autogen.sh && make && sudo make install
sudo apt-get install sphinxbase-utils
sudo ldconfig
```
