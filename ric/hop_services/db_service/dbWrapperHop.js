/*
this service receives a string from a nother hop service and passes it on to the rosbridge

to run
hop -v -p 9001 dbWrapperHop
*/

var hop = require ("hop");
var wss = new WebSocketServer( "dbWrapperHop" );
var rosbridgeURL = 'ws://localhost:9090';
//

wss.onconnection = function( event ) {
   var ws = event.value;
   

   console.log( "connection established:", ws.socket );
   //var tf= ["5","6"];
  // console.log(tf);
  // var sendback="eee";
  // console.log("sendback "+sendback);
   
   ws.onmessage = function( event ) {
      
      //var text = event.value;
      //var message = JSON.parse (text);
	    //console.log(message)
      
      
      var rs = new WebSocket (rosbridgeURL);
      console.log ('ROSbridge connection opened');
      var ret_cols=[{"data":"firstname"},{"data":"lastname"}];
      var ins =[{"data":"firstname"},{"data":"Alex"}];
      var req_data=[{"s":ins}];

      var send =
      {
        "op": "call_service",
        "service":"/ric/db/mysql_wrapper_service/fetchPersonalData",
        "args": { "return_cols": ret_cols,"req_data":req_data}            
      };    
       
      rs.send (JSON.stringify(send));    
      
        rs.onmessage = function (event)
        {
          var text = event.value;
          var message = JSON.parse (text);
          console.log(message.values.report.data);
          console.log(message.values.res_data[0].s[0].data+" "+message.values.res_data[0].s[1].data); //values.res_data[0]
        }


   };
   
};
//var rosbridge = connectToRosbridge (rosbridgeURL);
