
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');
var timeout = 1000;
var deleteEvent = 3000;

var count = 0;

(function loop(){
  setTimeout( function(){
    count += timeout;
    if(count > deleteEvent){
      console.log("Disconnect event");
      ros.closeConnection();
      return;
    }
    ros.getServices(function(data){
      //console.log(data);
    });
    loop();
  }, timeout);
})()
