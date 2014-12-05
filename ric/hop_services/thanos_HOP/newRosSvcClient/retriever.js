var rosbridgeURL = 'ws://localhost:9090';
var hop = require( "hop" );

service retriever (message){
  var rs = new WebSocket (rosbridgeURL);
  console.log ('ROSbridge connection opened');
  console.log(message);
  console.log(message.args.req_data[0].s);
  rs.send (JSON.stringify(message));
  var ret_message="old";   
  var flag = false;
    //improvised solution to the wait for websocket response problem. Needs to be addressed.
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
  }
}
