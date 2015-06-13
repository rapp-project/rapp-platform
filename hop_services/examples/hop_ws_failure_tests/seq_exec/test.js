
var faceImage = __dirname + '/Lenna.png'
var faceDetectionService = '/rapp/rapp_face_detection/detect_faces';


var timeNow = new Date().getTime();

var args = {};
var header = {
  'seq': 1,
  'stamp': timeNow,
  'frame_id': " "
};

//args['header'] = header;
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
var rosWS = new WebSocket('ws://localhost:9090');

rosWS.onopen = function(){
  console.log('\033[0;33m[Rosbridge-connection]\033[0m: Connected to rosbridge_websocket_server');
  rosWS.send(JSON.stringify(srvMsg)); //Invoke a call to rosbridge_websocket_server
};

// Define websocket onmessage ASYNC callback
rosWS.onmessage = function(event){
  timeNow = new Date().getTime();
  console.log('\033[0;33m[Rosbridge-connection]\033[0m: Received message %s', timeNow.toString());
  //console.log('\033[0;32m[Onmessage received]\033[0m:',message);
  msg = JSON.parse(event.value);
  //rosWS.send(JSON.stringify(srvMsg)); //Invoke a call to rosbridge_websocket_server
  process.exit(1); // Kill Hop process after received a message from rosbridge
};

//rosWS.on('error', function(err){
  //console.log('[WS-error]: ', err);
//});

rosWS.onclose = function(){
  console.log('[Rosbridge-connection]: Closed ');
};
//}






