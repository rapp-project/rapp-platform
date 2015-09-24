
var bridge = require( '../src/core/bridge.js' );
var rosbridge = new bridge('','');

var param = 'rosversion';
rosbridge.getParam(param, function(data){
  console.log(data);
})
