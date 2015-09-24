
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');

var param = 'rosversion';
ros.getServices(function(data){
  console.log(data);
})
