

var timeOutValue_ = 10000;  //Set global connection timeout value.
var runTime_ = 0;
//var rosbridgeURL_ = '';  //Global variable for rosBridgeURL.

/*!
 * @brief Make call to a ROS service.
 * @param _serviceName To be called service name.
 * @param _args Arguments of the specific ROS service.
 */
function callRosService ( _serviceName, _args, _rosbridgeURL ){
  _rosbridgeURL = _rosbridgeURL || 'ws://localhost:9090';

  var currentTime = new Date().getTime(); //Get current time.
  var header = {
    "seq": 1,
    "stamp": currentTime,
    "frame_id": " "
  };

  var args = {};
  args[ "header" ] = header;

  for ( var key in _args ){
    args[ key.toString() ] = _args[ key ];
  }
   
  var msg = {
    "op": "call_service",
    "service": _serviceName,
    "args": args
  };

  var flag = false;
  var retMessage = '';

  var rosBridgeSocket = new WebSocket(_rosbridgeURL);

  rosBridgeSocket.onopen = function(event){
    console.log('\033[0;33mROS_bridge Connection established!\033[0;0m');
  }
  
  rosBridgeSocket.onerror = function( error ){
    console.log( '\033[0;31mWebsocket error: ' + error );
  }

  rosBridgeSocket.send( JSON.stringify(msg) );

  var endT;
  var startT = new Date().getTime();
  while(flag==false)
  {
    rosBridgeSocket.onmessage = function (event)
    {
      var received = event.value;
      retMessage = JSON.parse (received);
      flag = true;
    }
    endT = new Date().getTime();
    if( (endT - startT) > timeOutValue_ )
    {
      console.log("\033[01;31mROS SERVICE COMMUNICATION TIMED OUT!\033[0;0m");
      retMessage = "ROS RESPONSE TIMEOUT!";
      break;
    }
  }
  //rosBridgeSocket.close();
  var funcs = []
  for(var name in rosBridgeSocket) {
    if(typeof rosBridgeSocket[name] === 'function') {
        funcs.push(name)
    }
  }
  console.log(funcs);

  rosBridgeSocket = null;
  delete rosBridgeSocket; 
  return retMessage;
}



module.exports = {
  callRosService: callRosService
}

