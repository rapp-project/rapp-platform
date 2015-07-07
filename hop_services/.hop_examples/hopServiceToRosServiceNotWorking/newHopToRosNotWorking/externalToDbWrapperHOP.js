/*
this service passes a string to the dbWrapperHop service.

to run
hop -v -p 9002
*/

import service svc1(message);
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

var tak="old";
var tak=svc1 (send).post(function rls(result){return result;},{ host: "localhost",asynchronous: false, port: "9001" });
console.log(tak);
//function run_remote () {
 //var tmp1;
 //var tmp2 = my_service(1).post(function (value){
 //console.log ("direct use: ", value  value);
 //tmp1 = value  value;
 //return value * value;},
 //{asynchronous: false, port: 8888});
 //console.log ("tmp1: ", tmp1);
 //console.log ("tmp2: ", tmp2);
//}

//run_remote();

  
