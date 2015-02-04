
/*
This service runs on the server and receives requests that passes on to the ROS node dbWrapper through the rosbridge.
*to run
hop -v -p 9001 dbWrapperHop
*/
//ric/db/mysql_wrapper_service/fetchPersonalData
import service retriever (message);
var rosbridgeURL = 'ws://localhost:9090';
var hop = require( "hop" );
var w1 = new Worker ("./retriever.js");
//rosbridge.advertise_service("/ric/db/mysql_wrapper_service/fetchPersonalData", "ros_wrappers", "AddTwoInts", addTwoInts, {host:"localhost", port:"9000"});


service svc1(message) {
  //on message receive, open websocket to rosbridge and pass the message.
  //var rs = new WebSocket (rosbridgeURL);
  //console.log ('ROSbridge connection opened');
  //console.log(message);
  //console.log(message.args.req_data[0].s);
  //rs.send (JSON.stringify(message));
  //var ret_message="old";   
  //var flag = false;
    ////improvised solution to the wait for websocket response problem. Needs to be addressed.
  //while(flag==false)
  //{
    //rs.onmessage = function (event)
    //{
      //var text = event.value;
      //ret_message = JSON.parse (text);
      //flag = true;
      ////console.log(ret_message.values.report.data);
      ////console.log(ret_message.values.res_data[0].s[0].data+" "+ret_message.values.res_data[0].s[1].data); //values.res_data[0]         
      ////console.log("returning "+ret_message);
      ////return ret_message;
    //}   
  //}
  function retrieve (message) {
    
    var tmp2 = retriever(message).post(function (value){
	    console.log ("direct use: ", value * value);
	    tmp1 = value * value;
	    return value * value;},	{asynchronous:false});
  
  
    console.log ("tmp1: ", tmp1);
    console.log ("tmp2: ", tmp2);
    
  }
  setTimeout (run(message), 10);
  console.log("returning");
  console.log(ret_message);
  return ret_message;
}

