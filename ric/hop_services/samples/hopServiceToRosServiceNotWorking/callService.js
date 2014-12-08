/**
 * This program implements the well-known 'add_two_ints' service found
 * in ROS tutorials, using directly the ROSbridge protocol to
 * advertise the service, and to reply to service requests.
 * @module
 * @copyright  2014 Inria
 * 
 */

var ns = require("./RosBridge.js");
var hop = require( "hop" );

//service addTwoInts(args) {        
        //console.log("addTwoInts");
        //var result = {};
        //result.sum = args.a + args.b;
        //console.log(result.sum," = ",args.a," + ",args.b)
        //return result;
//}

service add_two_ints (a, b) {
return hop.HTTPResponseAsync (function (handler) {
 rosbridge.call_service("/ric/db/mysql_wrapper_service/fetchPersonalData",
 { return_cols:a, req_data:b},
 function (result) {
 handler (result.sum );
 })});
}

var rosbridge = new ns.RosBridge('ws://localhost:9090');
rosbridge.advertise_service("/ric/db/mysql_wrapper_service/fetchPersonalData", add_two_ints, {host:"localhost", port:"9000"});

//rapp_platform_ros_communications
//DbWrapperSrv
//rosbridge.advertise_service("/hop/increaseCounter", "std_srvs", "Empty", increaseCounter, {host:"localhost", port:"9000"});
//rosbridge.advertise_service("/hop/decreaseCounter", "std_srvs", "Empty", decreaseCounter, {host:"localhost", port:"9000"});
//rosbridge.call_service("/add_two_ints", { a:19, b:30}, function (result) { console.log("result = ", result.sum ); } );
//rosbridge.call_service("/add_two_ints", { a:1, b:3}, function (result) { console.log("result = ", result.sum ); } );
var ret_cols=[{"data":"firstname"},{"data":"lastname"}];
var ins =[{"data":"firstname"},{"data":"Alex"}];
var req_data=[{"s":ins}];
//rosbridge.call_service("/ric/db/mysql_wrapper_service/fetchPersonalData", { return_cols:ret_cols, req_data:req_data}, function (result) { console.log("result = "+ result ); } );


//var ls= hop.HTTPResponseAsync (function (handler) {
 //rosbridge.call_service("/ric/db/mysql_wrapper_service/fetchPersonalData", { return_cols:ret_cols, req_data:req_data}, function (result) { console.log(result)});
 //})});
add_two_ints (ret_cols, req_data);
//console.log(rosbridge);
//rosbridge.close()

