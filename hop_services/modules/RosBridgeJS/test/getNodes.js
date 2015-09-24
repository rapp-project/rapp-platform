
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');

ros.getNodes(function(data){
  console.log(data);
})
