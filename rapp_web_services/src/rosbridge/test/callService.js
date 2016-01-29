
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');
var timeout = 1000;
var srvName = '/rapp';
var args = {};

(function loop(){
  setTimeout( function(){
    ros.callService(srvName, args, function(data){
      console.log(data);
    });
    loop();
  }, timeout);
})()
