#!/bin/bash
rm Sphinx4.class
javac -cp ".:$HOME/rapp-platform-catkin-ws/src/rapp-platform-supplementary-material/rapp_sphinx4_java_libraries/sphinx4-core-1.0-SNAPSHOT.jar" Sphinx4.java
