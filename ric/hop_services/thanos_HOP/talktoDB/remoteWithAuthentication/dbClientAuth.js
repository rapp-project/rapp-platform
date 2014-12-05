/*
this service passes a string to the dbWrapperHop service.

to run
hop -v -p 9002
*/

import service dbService(message);
//import service private(type,message);

//function connect() {
   ///* accepted connection */
   //private("password","something").post( console.log,
		  //{ async: false,
		    //host: "155.207.19.37",
		    //port: 9001,
		    //user: "foo",
		    //password: "foo",
		    //fail: function( err ) {
		       //console.log( "private, with pwd, connection refused: ", err )
		    //} } );
      //}
//connect();




var ret_cols=[{"data":"firstname"},{"data":"lastname"}];
var ins =[{"data":"firstname"},{"data":"Alex"}];
var req_data=[{"s":ins}];

var send =
{       "op": "call_service",
        "service":"/ric/db/mysql_wrapper_service/fetchPersonalData",        
        "args": {
           "return_cols": ret_cols,
           "req_data":req_data  }       
};   
var result1="GAMW";
var waitbo=false;
var ls =false;
//var result;
//var result1="old";
//var result1=svc1 ({message:send}).post(function(result){console.log(result);return result;},{ host: "localhost", port: "9001" }); //function(result){return result;},


var tak= dbService(send).post(function(result){return result},
		  { asynchronous: false,
		    host: "155.207.19.37",
		    port: 9001,
		    user: "rapp",
		    password: "R@pPpaSS!",
		    fail: function( err ) { console.log( "private, with pwd, connection refused: ", err ) }
		    } );
console.log(tak);


  
