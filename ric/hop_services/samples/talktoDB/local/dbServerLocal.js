
/*
This service runs on the server and receives requests that passes on to the ROS node dbWrapper through the rosbridge.
*to run
hop -v -p 9001 dbWrapperHop
*/

var rosbridgeURL = 'ws://localhost:9090';
var hop = require( "hop" );
var rs = new WebSocket (rosbridgeURL);
service dbService(message) {
  //on message receive, open websocket to rosbridge and pass the message.
  
  console.log ('ROSbridge connection opened');
  console.log(message);
  console.log(message.args.req_data[0].s);
  rs.send (JSON.stringify(message));
  //var ret_message="old";   
  var flag = false;
    //improvised solution to the wait for websocket response problem.
  var startDate = new Date().getTime();  
  while(flag==false)
  {
    rs.onmessage = function (event)
    {
      var text = event.value;
      ret_message = JSON.parse (text);
      flag = true;
      //console.log(ret_message.values.report.data);
      //console.log(ret_message.values.res_data[0].s[0].data+" "+ret_message.values.res_data[0].s[1].data); //values.res_data[0]         
      //console.log("returning "+ret_message);
      //return ret_message;
    }   
    var endDate = new Date().getTime();  
    if((endDate-startDate)>1000)
    {
      console.log("Ros timeout, exiting..");
      ret_message="Ros timeout";
      break;
    }
  }
  
  console.log("returning");
  console.log(ret_message);
  return ret_message;
}

