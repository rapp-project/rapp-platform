var user = process.env.LOGNAME;

var faceImage = '/home/' + user + '/Desktop/hop_tests/hop-rosbridge/Lenna.png'
var faceDetectionService = '/ric/face_detection_service';
var msg = undefined;
var responseFlag = false;
/*var _this = this;*/

/* Websocket event callback handlers */
/*=====================================*/

var rosWS = new WebSocket('ws://localhost:9090');
rosWS.onopen = function(event){
  console.log('\033[0;33m[Rosbridge-connection]\033[0m: Connected to rosbridge_websocket_server');
}

// Define websocket onmessage callback
rosWS.onmessage = function(event){
  console.log('\033[0;33m[Rosbridge-connection]\033[0m: Received message');
  /*_this.*/responseFlag = true;
  /*_this*/msg = JSON.parse(event.value);
  console.log('\033[0;32m[Onmessage received]\033[0m:', /*_this.*/msg);
  this.close();
};
/*=====================================*/

/* Craft faceDetection ROS service message */
/*=========================================*/
var args = {};

var header = {
  'seq': 1,
  'stamp': timeNow,
  'frame_id': " "
};

args['header'] = header;
args['imageFilename'] = faceImage;

//craft faceDetection ros service message
var srvMsg = {
  'op': 'call_service',
  'service': faceDetectionService,
  'args': args,
  'id': ' '
};

/*=========================================*/
var timeNow = new Date().getTime();

rosWS.send(JSON.stringify(srvMsg)); //Invoke a call to rosbridge_websocket_server
var endT; 
var startT = new Date().getTime();

setTimeout(function(){
  if(!responseFlag){
    console.log('\033[0;33m[Rosbridge-connection]\033[0m: \033[0;31mRos service communication timed out\033[0m');
  }
}, 1000);



