rm Sphinx4.java
rm Sphinx4.class
cp /home/thanos/NetBeansProjects/Sphinx4Maven/src/main/java/Sphinx4.java ./
javac -cp ".:sphinx4-core-1.0-SNAPSHOT.jar:sphinx4-data-1.0-SNAPSHIOT.jar" Sphinx4.java
#java -cp .:/home/thanos/local_catkin_workspaces/catkin_ws/src/rapp-platform/rapp_ric/sphinx4_wrapper/src/sphinx4-core-1.0-SNAPSHOT.jar:/home/thanos/#local_catkin_workspaces/catkin_ws/src/rapp-platform/rapp_ric/sphinx4_wrapper/src/sphinx4-data-1.0-SNAPSHOT.jar:/home/thanos/NetBeansProjects/#Sphinx4Maven/target/classes/  Sphinx4
