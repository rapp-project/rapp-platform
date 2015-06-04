var faceImage = '/home/klpanagi/Desktop/hop_rosbridge_test/Lenna.png'
var faceDetectionService = '/ric/face_detection_service';

var WebSocket = require('ws');

var timeNow = new Date().getTime();

var args = {};
var header = {
  'seq': 1,
  'stamp': timeNow,
  'frame_id': " "
};

args['header'] = header;
args['imageFilename'] = faceImage;

var msg = undefined;




//craft faceDetection ros service message
var srvMsg = {
  'op': 'call_service',
  'service': faceDetectionService,
  'args': args,
  'id': '1'
};
// === WebSocket Event Callbacks ==================
// ================================================
var rosWS = new WebSocket('ws://localhost:9090');

rosWS.on('open', function(){
  console.log('\033[0;33m[Rosbridge-connection]\033[0m: Connected to rosbridge_websocket_server');
  wsOpenFlag = true;
  rosWS.send(JSON.stringify(srvMsg)); //Invoke a call to rosbridge_websocket_server
});

// Define websocket onmessage ASYNC callback
rosWS.on('message', function(message){
  console.log('\033[0;33m[Rosbridge-connection]\033[0m: Received message');
  console.log('\033[0;32m[Onmessage received]\033[0m:',message);
  msg = JSON.parse(message);
  console.log(msg.values);
});

rosWS.on('error', function(err){
  console.log('[WS-error]: ', err);
  setTimeout(poll, 1000);
});

rosWS.on('close', function(){
  console.log('[Rosbridge-connection]: Closed ');
});
//}
//==================================================
//while(!wsOpenFlag){};






