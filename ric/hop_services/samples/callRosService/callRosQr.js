/* this example communicates with a ros service through websockets. Be sure to select a valid path
 * for the image filename. The ros node qr_detection accepts a header and the path of qr file (.png or jpg)
 * be sure that rosbridge and the ros qr_detection_service node are runnning. * 
 * to run hop -v -p 9001 callRosQr.js
 * */

var rosbridgeURL = 'ws://localhost:9090';
var hop = require( "hop" );
var rs = new WebSocket (rosbridgeURL);
console.log ('ROSbridge connection opened');
var sd1 = new Date().getTime();
var t1={"seq":1,"stamp":sd1,"frame_id":" "};
console.log(t1);
var send =    //this message is a query that the dbWrapperHop service will pass to the db_wrapper ros node
{ 
  "op": "call_service",
  "service":"/ric/ros_nodes/qr_detection_service",        
  "args": 
  {
    "header": t1,
    "imageFilename":"/home/thanos/Desktop/sample.png" //filenamePATH    
  }   
}; 
var flag=false;
rs.send (JSON.stringify(send));
  var startDate = new Date().getTime();  
  //this while is used to wait for the rs.onmessage response. the timeout period is set inside
  while(flag==false)
  {
    rs.onmessage = function (event)
    {
      var text = event.value;
      ret_message = JSON.parse (text);
      flag = true;
    }   
    var endDate = new Date().getTime();  
    if((endDate-startDate)>10000)
    {
      console.log("Ros timeout, exiting..");
      ret_message="Ros timeout";
      break;
    }
  }
console.log(ret_message);
