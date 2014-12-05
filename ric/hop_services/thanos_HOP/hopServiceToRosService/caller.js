//var hop = require( 'hop' );
var EventEmmiter2 = require ("./eventemitter2.min.js");
//src:"http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js";
var ROSLIB = require ("./roslib.min.js");

	         //<HEAD> {
				          //<TITLE> { "Example of roslibjs" },
				            //<SCRIPT> { src: "http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js" },
					    //<SCRIPT> { src: "http://cdn.robotwebtools.org.s3.amazonaws.com/roslibjs/current/roslib.min.js" },
					    //~{ 
								    
  // Connecting to ROS
  // -----------------

 // var ros = new ROSLIB.Ros({
    //url : 'ws://vbox-14-04-lts-x86:9090'
 //   url : 'ws://localhost:9090'
 // });

  //ros.on('connection', function() {
    //console.log('Connected to websocket server.');
  //});

  //ros.on('error', function(error) {
    //console.log('Error connecting to websocket server: ', error);
  //});

  //ros.on('close', function() {
    //console.log('Connection to websocket server closed.');
  //});

  

  //// Calling a service
  //// -----------------

  //var addTwoIntsClient = new ROSLIB.Service({
    //ros : ros,
    //name : '/add_two_ints',
    //serviceType : 'rospy_tutorials/AddTwoInts'
  //});

  //var request = new ROSLIB.ServiceRequest({
    //a : 1,
    //b : 2
  //});

  //addTwoIntsClient.callService(request, function(result) {
    //console.log('Result for service call on '
      //+ addTwoIntsClient.name
      //+ ': '
      //+ result.sum);
  //});
  


