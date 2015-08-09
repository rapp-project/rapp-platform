/*!
 * @file ontology_is_subsuperclass_of.service.js
 * @brief Is_subsuperclass_of HOP service.
 *
 */

var __DEBUG__ = false;


/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var module_path = '../utilities/js/'
/*----------------------------------------------*/
var RandStringGen = require ( module_path + 'randStringGen.js' );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var ros_service_name = "/rapp/rapp_knowrob_wrapper/is_subsuperclass_of";
var hop = require('hop');

/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/

/* -- Set timer values for websocket communication to rosbridge -- */
var timer_tick_value = 100 // ms
var max_time = 2000 // ms
var max_tries = 2
//var max_timer_ticks = 1000 * max_time / tick_value;
/* --------------------------------------------------------------- */

var __hopServiceName = 'ontology_is_subsuperclass_of';
var __hopServiceId = null;
var __masterId = null;
var __storeDir = '~/.hop/cache/';

register_master_interface();


/*!
 * @brief Ontology is_subsuperclass_of query, HOP Service.
 *
 * @param parent_class
 * @param child_class
 * @param recursive Used to define a recursive behavior.
 */
service ontology_is_subsuperclass_of ( {parent_class: '', child_class: '', recursive: false} )
{
  /**** Boolean parameters are passed as strings onto the URL payload ****/

  /* -- Handling string to boolean casting -- */
  if (recursive == 'True' || recursive == 'true')
  {
    // String 'True' --> booleand true
    recursive = true;
  }
  else
  {
    recursive = false;
  }
  /* ---------------------------------------- */

  postMessage( craft_slaveMaster_msg('log', 'client-request') );

 /*----------------------------------------------------------------- */
 return hop.HTTPResponseAsync(
   function( sendResponse ) {

     var args = {};
     args[ "parent_class" ] = parent_class;
     args[ "child_class" ] = child_class;
     args[ "recursive" ] = recursive;

    /*=============================TEMPLATE======================================================*/
      var rosbridge_connection = true;
      var respFlag = false;

      // Create a unique caller id
      var unqCallId = randStrGen.createUnique();
      var rosbridge_msg = craft_rosbridge_msg(args, ros_service_name, unqCallId);

      /* ------ Catch exception while open websocket communication ------- */
      try{
        var rosWS = new WebSocket('ws://localhost:9090');

        // Register WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;

          var logMsg = 'Connection to rosbridge established';
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          this.send(JSON.stringify(rosbridge_msg));
        }

        // Register WebSocket.onclose callback
        rosWS.onclose = function(){
          var logMsg = 'Connection to rosbridge closed';
          postMessage( craft_slaveMaster_msg('log', logMsg) );
        }

        // Register WebSocket.onmessage callback
        rosWS.onmessage = function(event){
          var logMsg = 'Received message from rosbridge';
          postMessage( craft_slaveMaster_msg('log', logMsg) );

          //console.log(event.value);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( unqCallId );
          sendResponse( resp_msg );
        }
      }
      catch(e){
        rosbridge_connection = false;
        //console.log(e);

        var logMsg = 'ERROR: Cannot open websocket' +
          'to rosbridge [ws//localhost:9090]\r\n' + e;
        postMessage( craft_slaveMaster_msg('log', logMsg) );

        var resp_msg = craft_error_response;
        sendResponse( resp_msg );
        return;
      }
      /*------------------------------------------------------------------ */

      var timer_ticks = 0;
      var elapsed_time = 0;
      var retries = 0;

      // Set Timeout wrapping function
      function asyncWrap(){
        setTimeout( function(){
         timer_ticks += 1;
         elapsed_time = timer_ticks * timer_tick_value;

         if (respFlag == true)
         {
           return
         }
         else if (respFlag != true && elapsed_time > max_time ){
           timer_ticks = 0;
           retries += 1;

           var logMsg = 'Reached rosbridge response timeout' +
             '---> [' + elapsed_time + '] ms ... Reconnecting to rosbridge.' +
             'Retry-' + retries;
           postMessage( craft_slaveMaster_msg('log', logMsg) );

           if (retries > max_tries) // Reconnected for max_tries times
           {
             var logMsg = 'Reached max_retries [' + max_tries + ']' +
               ' Could not receive response from rosbridge...';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             var respMsg = craft_error_response();

             //  Close websocket before return
             rosWS.close();
             rosWS = undefined;
             sendResponse( respMsg );
             return;
           }

           if (rosWS != undefined)
           {
             rosWS.close();
           }
           rosWS = undefined;

           /* --------------< Re-open connection to the WebSocket >--------------*/
           try{
             rosWS = new WebSocket('ws://localhost:9090');

             /* -----------< Redefine WebSocket callbacks >----------- */
             // Register Websocket.onopen callback
             rosWS.onopen = function(){
               var logMsg = 'Connection to rosbridge established';
               postMessage( craft_slaveMaster_msg('log', logMsg) );
               this.send(JSON.stringify(rosbridge_msg));
             }

             // Register Websocket.onclose callback
             rosWS.onclose = function(){
               var logMsg = 'Connection to rosbridge closed';
               postMessage( craft_slaveMaster_msg('log', logMsg) );
             }

             // Register Websocket.onmesasge callback
             rosWS.onmessage = function(event){
               var logMsg = 'Received message from rosbridge';
               postMessage( craft_slaveMaster_msg('log', logMsg) );

               var resp_msg = craft_response( event.value );
               //console.log(resp_msg);

               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket
               respFlag = true;
               //Remove the unqCallId so it can be reused
               randStrGen.removeCached( unqCallId );
               sendResponse( resp_msg ); //Return response to client
             }
           }
           catch(e){
             rosbridge_connection = false;
             //console.log(e);

             var logMsg = 'ERROR: Cannot open websocket' +
               'to rosbridge --> [ws//localhost:9090]';
             postMessage( craft_slaveMaster_msg('log', logMsg) );

             var resp_msg = craft_error_response();
             sendResponse( resp_msg );
             return;
           }

         }
         /*--------------------------------------------------------*/
         asyncWrap(); // Recall timeout function

       }, timer_tick_value); //Timeout value is set at 100 ms.
     }
     asyncWrap();
/*==============================================================================================*/
   }, this );
};



/*!
 * @brief Crafts the form/format for the message to be returned to client
 * @param rosbridge_msg Return message from rosbridge
 * @return Message to be returned from service
 */
function craft_response(rosbridge_msg)
{
  var msg = JSON.parse(rosbridge_msg);
  var results = msg.values.results;
  var trace = msg.values.trace;
  var success = msg.values.success;
  var error = msg.values.error;
  var call_result = msg.result;

  var crafted_msg = {results: [], trace: [], error: ''};

  if (call_result)
  {
    for (var ii = 0; ii < results.length; ii++)
    {
      crafted_msg.results.push(results[ii]);
    }
    for (var ii = 0; ii < trace.length; ii++)
    {
      crafted_msg.trace.push(trace[ii]);
    }

    crafted_msg.error = error;
    logMsg = 'Returning to client.';

    if (error != '')
    {
      logMsg += ' ROS service [' + ros_service_name + '] error'
        ' ---> ' + error;
    }
    else
    {
      logMsg += ' ROS service [' + ros_service_name + '] returned with success'
    }
  }
  else
  {
    logMsg = 'Communication with ROS service ' + ros_service_name +
      'failed. Unsuccesful call! Returning to client with error' +
      ' ---> RAPP Platform Failure';
    crafted_msg.error = 'RAPP Platform Failure';
  }

  //console.log(crafted_msg);
  postMessage( craft_slaveMaster_msg('log', logMsg) );
  return JSON.stringify(crafted_msg)
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  var errorMsg = 'RAPP Platform Failure!'
  var crafted_msg = {results: [], trace: [], error: 'RAPP Platform Failure'};

  var logMsg = 'Return to client with error --> ' + errorMsg;
  postMessage( craft_slaveMaster_msg('log', logMsg) );

  return JSON.stringify(crafted_msg);
}


/*!
 * @brief Crafts ready to send, rosbridge message.
 *   Can be used by any service!!!!
 */
function craft_rosbridge_msg(args, service_name, id){

  var rosbrige_msg = {
    'op': 'call_service',
    'service': service_name,
    'args': args,
    'id': id
  };

  return rosbrige_msg;
}


function register_master_interface()
{
  // Register onexit callback function
  onexit = function(e){
    console.log("Service [%s] exiting...", __hopServiceName);
    var logMsg = "Received termination command. Exiting.";
    postMessage( craft_slaveMaster_msg('log', logMsg) );
  }

  // Register onmessage callback function
  onmessage = function(msg){
    if (__DEBUG__)
    {
      console.log("Service [%s] received message from master process",
        __hopServiceName);
      console.log("Msg -->", msg.data);
    };

    var logMsg = 'Received message from master process --> [' +
      msg.data + ']';
    postMessage( craft_slaveMaster_msg('log', logMsg) );

    exec_master_command(msg.data);
  }

  // On initialization inform master and append to log file
  var logMsg = "Initiated worker";
  postMessage( craft_slaveMaster_msg('log', logMsg) );
}


function exec_master_command(msg)
{
  var cmd = msg.cmdId;
  var data = msg.data;
  switch (cmd)
  {
    case 2055:  // Set worker ID
      __hopServiceId = data;
      break;
    case 2050:
      __masterId = data;
      break;
    default:
      break;
  }
}


function craft_slaveMaster_msg(msgId, msg)
{
  var msg = {
    name: __hopServiceName,
    id:   __hopServiceId,
    msgId: msgId,
    data: msg
  }
  return msg;
}
