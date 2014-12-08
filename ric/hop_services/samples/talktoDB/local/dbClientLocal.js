/*
this service passes a string to the dbWrapperHop service.

to run
hop -v -p 9002 dbClientLocal.js
*/

import service dbService(message);

var ret_cols=[{"data":"firstname"},{"data":"lastname"}];
var ins =[{"data":"firstname"},{"data":"Alex"}];
var req_data=[{"s":ins}];

var send =    //this message is a query that the dbWrapperHop service will pass to the db_wrapper ros node
{
  "op": "call_service",
  "service":"/ric/db/mysql_wrapper_service/fetchPersonalData",        
  "args":
  {
    "return_cols": ret_cols,
    "req_data":req_data
  }       
};   

var result= dbService(send).post(
    function(result){return result},
    {
      asynchronous: false,
      host: "localhost",
      port: 9001,
      // user: "rapp",         //these lines are for authentication
      // password: "R@pPpaSS!",
      fail: function( err ) { console.log( "connection refused: ", err ) }
    }
  );
console.log(result);


  
