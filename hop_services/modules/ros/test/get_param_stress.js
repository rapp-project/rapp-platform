var RosParamClient = require( '../rosParamClient.js' );
var rosParam = new RosParamClient({});

var qrThreads = 0;
var numCalls = 10

for (var ii = 0; ii < numCalls; ii++){
  rosParam.getParam('/rapp_qr_detection_threads', function(data){
    qrThreads = data;
    console.log("Qr detection threads param: %s", data);
  });
}


