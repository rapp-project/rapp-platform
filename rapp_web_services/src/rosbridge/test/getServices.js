
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');
var timeout = 1000;


(function loop(){
  setTimeout( function(){
    ros.getServices(function(data){
      console.log(data);
    });
    loop();
  }, timeout);
})()
