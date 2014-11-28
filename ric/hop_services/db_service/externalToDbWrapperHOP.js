/*
this service passes a string to the dbWrapperHop service.

to run
hop -v -p 9002 dbWrapperHop
*/

var ws = new WebSocket( "ws://localhost:9001/hop/dbWrapperHop" );


//service fileGet( path ) {
   //return hop.HTTPResponseFile( path );
//}
//ws.onopen = function( event ) {
   //var file = fileGet.resource( "santorini.jpg" );
   

      var ret_cols=[{"data":"firstname"},{"data":"lastname"}];
      var ins =[{"data":"firstname"},{"data":"Alex"}];
      var req_data=[{"s":ins}];

      var send =
      {
        "vlvs":{
               //"op": "call_service",
              //"service":"/ric/db/mysql_wrapper_service/fetchPersonalData",        
              "args": {
                 "return_cols": ret_cols,
                 "req_data":req_data
                 }      
              }
      };       

      ws.send (JSON.stringify(send))
//};

//ws.onmessage = function( event ) {
   //console.log( "received [%s]", event.value );
//};
