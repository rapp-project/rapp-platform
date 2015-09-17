var RosParamClient = require( '../rosParamClient.js' );
var rosParam = new RosParamClient({});

var qrThreads = 0;

rosParam.getParam('/rapp_qr_detection_threads', function(data){
  qrThreads = data;
});


console.log("Qr detection threads param: %s", qrThreads);
setTimeout(function(){
  console.log("Qr detection threads param: %s", qrThreads);
  if(qrThreads == 10){
    console.log("\033[1;32mSuccess!\033[0m");
  }
  else {
    console.log("\033[1;31mFailed!\033[0m");
  }
}, 100)



