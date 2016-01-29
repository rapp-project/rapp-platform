
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');
var timeout = 1000;
var deleteEvent = 3000;

var count = 0;

(function loop(){
  setTimeout( function(){
    count += timeout;
    console.log("CloseOpen event");
    ros.closeConnection();
    ros.connect();
    loop();
  }, timeout);
})()
