/*!
 * @file serve.js
 * @brief Serve prototype/class definition.
 *
 * Implemeted with static methods.
 *
 * Holds methods used for communicating with external services.
 */


/*######################-<Global Variables Scope here>-#######################*/


/*############################################################################*/



/*!
 * @brief File Streaming Class/Object definition.
 */
function RosUtils(){
  this.kind = 'RosUtils';
  this.rosbridgeURL = '';
  /*-----Static methods goes here (this.getX = function(){})*/
  var rosConnect;
  this.runTime = 0;
  this.timeOutValue = 10000;
  /*--------------------------------------------------------*/
};


/*#######################-<Static Methods/Members>-##########################*/

RosUtils.init_bridge = function (_rosbridgeURL){
  if(_rosbridgeURL==''){
    this.rosbridgeURL = 'ws://localhost:9090';
  }
  else{
    this.rosbridgeURL = _rosbridgeURL;
  }
  rosConnect = new WebSocket(this.rosbridgeURL);
  rosConnect.onopen = function(){
    console.log('\033[0;33mROS_bridge Connection established!\033[0;0m');
  }
  this.runTime = new Date().getTime();
};


RosUtils.callService = function (_serviceName, _args){
  var currentTime = new Date().getTime();
  var header = {
    "seq": 1,
    "stamp": currentTime,
    "frame_id": " "
  };
  var msg = {
    "op": "call_service",
    "service": _serviceName,
    "args": _args
  };
  var flag = false;
  var retMessage = '';
  rosConnect.send(JSON.stringify(msg));
  var startT = new Date().getTime();
  var endT;
  while(flag==false)
  {
    rosConnect.onmessage = function (event)
    {
      var received = event.value;
      retMessage = JSON.parse (received);
      flag = true;
    }
    endT = new Date().getTime();
    if( (endT - startT) > this.timeOutValue )
    {
      console.log("\033[01;31mROS SERVICE COMMUNICATION TIMED OUT!");
      retMessage = "ROS RESPONSE TIMEOUT!";
      break;
    }
  }
  return retMessage;
}


/*########################################################################*/


//exports the class as a module
module.exports = RosUtils; 

