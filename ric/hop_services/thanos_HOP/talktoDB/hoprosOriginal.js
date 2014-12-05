// Copyright © 2014 Inria
//

// This program implements the well-known 'add_two_ints' service found
// in ROS tutorials, using directly the ROSbridge protocol to
// advertise the service, and to reply to service requests.

var hop = require ("hop");

// The ROSbridge WebSocket URL.
var rosbridgeURL = 'ws://localhost:9090';
//var ws = new WebSocket( "ws://localhost:9999/hop/wss" );

// Return an opened WebSocket to ROSbridge, with all the handlers
// installed.

function connectToRosbridge ()
{
    var ws = new WebSocket (rosbridgeURL);
    console.log ('ROSbridge connection opened');
    //response="ggg";
    
    var ret_cols=[{"data":"firstname"},{"data":"lastname"}];
    var ins =[{"data":"firstnam7e"},{"data":"Alex"}];
    var req_data=[{"s":ins}];

    var send =
          {
            "op": "call_service",
            "service":"/ric/db/mysql_wrapper_service/fetchPersonalData",
            "args": { "return_cols": ret_cols,"req_data":req_data}
              
          };
          
        //  send="takis";
    ws.send (JSON.stringify(send));
    
    ws.onmessage = function (event)
    {
      var text = event.value;
      var message = JSON.parse (text);
	console.log(message.values.report.data)


      console.log(message.values.res_data[0].s[0].data+" "+message.values.res_data[0].s[1].data); //values.res_data[0]


    }
    // Handle WebSocket messages, optionally relaying them to the clients.
    //ws.send("5");
    //ws.onmessage = function (event)
    //{
      //// XXX: As of HOP 3.0.0-pre12, 'event.target' is different
      //// from 'ws', and not actually usable.
      //// var ws = event.target;
      //var text = event.value;
      //var message = JSON.parse (text);

      //if (message.request_id !== undefined) {
          //// Reply to the service call.
          //var result = { "sum": message.args.a + message.args.b };
          //var response =
          //{
              //"op": "service_response",
              //"request_id": message.request_id,
              //"data": result
          //};

          //console.log ("sending service response", message.request_id,
          //response);
          //ws.send (JSON.stringify (response));
      //}
    //};

    // Handle the 'open' event on the WebSocket.
    //ws.onopen = function (event)
    //{
        //console.log ("websocket opened\n");

        //// Advertise a ROS service.
        //var msg =
        //{
            //"op": "advertise_service",
            //"service_module": "thanos_testing",
            //"service_type": "AddTwoInts",
            //"service_name": "add_two_ints"
        //};
        //ws.send (JSON.stringify (msg));
    //}

    ws.onclose = function (event)
    {
        console.log ("ROSbridge connection closed");
    }

    return ws;
}

// Open a client websocket to ROSbridge.
var rosbridge = connectToRosbridge (rosbridgeURL);
