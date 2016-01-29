
var ROS = require( __dirname + '/../src/Rosbridge.js');
var ros = new ROS('','');
var timeout = 1000;
var srvName = [];
var CLIENTS = 50;

for(i=0;i<CLIENTS;i++)
{
  srvName.push('/rosapi/topics');
}

var args = {};

setTimeout( function(){
  for(i in srvName){
    console.log('[Client %s]', i)
    ros.callService(srvName[i], args, function(data){
      console.log(data);
    });
  }
}, timeout);
