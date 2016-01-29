
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');
var timeout = 1000;
var param = 'rosversion';


(function loop(){
  setTimeout( function(){
    ros.getParam(param, function(data){
      console.log(data);
    });
    loop();
  }, timeout)
})()

