
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');
var timeout = 1000;

(function loop(){
  setTimeout( function(){
    ros.getNodes(function(data){
      var result = (data) ? 'Success' : 'Failed'
      console.log("- GetNodes: %s", result);
    });
    ros.getServices(function(data){
      var result = (data) ? 'Success' : 'Failed'
      console.log("- GetServices: %s", result);
    });
    ros.getTopics(function(data){
      var result = (data) ? 'Success' : 'Failed'
      console.log("- GetTopics: %s", result);
    });
    loop();
  }, timeout)
})()
