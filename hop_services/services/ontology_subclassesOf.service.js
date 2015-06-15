/*!
 * @file ontology_subclassesOf.service.js
 * @brief Ontology query "Subclasses Of" hop service.
 * @bug Currently NOT Operational!!
 *
 */

console.log("Initiating Ontology-SubclassesOf front-end service");

// TODO -- Get ontology_subclassesOf rosservice name

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var module_path = '../utilities/js/'
/*----------------------------------------------*/
var RandStringGen = require ( module_path + 'randStringGen.js' );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var ros_service_name = "/rapp/rapp_knowrob_wrapper/subclasses_of";
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


/*!
 * @brief Ontology SubclassOf database query, HOP Service Core.
 *
 * @param queryStr Ontology query (String).
 */
service ontology_subclassesOf ( {queryStr:''} )
{
  var randStr = randStrGen.createUnique();
  console.log("[Ontology-subclassesOf]: Client Request");
  console.log('[Ontology-subclassesOf]: Query -->', queryStr);

 /*----------------------------------------------------------------- */
 return hop.HTTPResponseAsync(
   function( sendResponse ) { 

     var args = {};
     args[ "ontology_class" ] = queryStr;

    /*=============================TEMPLATE======================================================*/
      var rosbridge_connection = true;
      var respFlag = false;

      // Create a unique caller id
      var uniqueID = randStrGen.createUnique();
      var rosbridge_msg = craft_rosbridge_msg(args, ros_service_name, uniqueID);

      /* ------ Catch exception while open websocket communication ------- */
      try{
        var rosWS = new WebSocket('ws://localhost:9090');
      }
      catch(e){
        rosbridge_connection = false; // Could not open websocket to rosbridge websocket server
        console.error('[Ontology-subclassesOf] ERROR: Cannot open websocket to rosbridge' +  
          '--> [ws//localhost:9090]' );
        // Print exception 
        console.log(e);
        // Craft return to client message
        var resp_msg = craft_error_response();
        // Return to Client
        sendResponse( JSON.stringify(resp_msg) ); 
        console.log("[Ontology-subclassesOf]: Returning to client with error");
        return
      }
      /* ----------------------------------------------------------------- */
     
      /* ------- Add into a try/catch block to ensure safe access -------- */
      try{
        // Implement WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;
          console.log('[Ontology-subclassesOf]: Connection to rosbridge established');
          this.send(JSON.stringify(rosbridge_msg));
        }
        // Implement WebSocket.onclose callback
        rosWS.onclose = function(){
          console.log('[Ontology-subclassesOf]: Connection to rosbridge closed');
        }
        // Implement WebSocket.message callback
        rosWS.onmessage = function(event){
          console.log('[Ontology-subclassesOf]: Received message from rosbridge');
          //console.log(event.value);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket 
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( uniqueID ); 
          sendResponse( resp_msg );
          console.log("[Ontology-subclassesOf]: Returning to client");
        }
      }
      catch(e){
        rosbridge_connection = false;
        console.error('[Ontology-subclassesOf] --> ERROR: Cannot open websocket' + 
          'to rosbridge --> [ws//localhost:9090]' );
        console.log(e);
        var resp_msg = craft_error_response;
        sendResponse( resp_msg ); 
        console.log("[Ontology-subclassesOf]: Returning to client with error");
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

         if (respFlag != true && elapsed_time > max_time ){
           timer_ticks = 0;
           retries += 1;

           console.log("[Ontology-subclassesOf]: Reached rosbridge response timeout" + 
             "---> [%s] ms ... Reconnecting to rosbridge. Retry-%s", 
             elapsed_time.toString(), retries.toString());

           if (retries > max_tries) // Reconnected for max_tries times
           {
             console.log("[Ontology-subclassesOf]: Reached max_retries (%s)" + 
               "Could not receive response from rosbridge... Returning to client",
               max_tries);
             var respMsg = craft_error_response();
             sendResponse( JSON.stringify(respMsg) );
             console.log("[Ontology-subclassesOf]: Returning to client with error");
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
             rosWS.onopen = function(){
             console.log('[Ontology-subclassesOf]: Connection to rosbridge established');
             this.send(JSON.stringify(rosbridge_msg));
             }

             rosWS.onclose = function(){
               console.log('[Ontology-subclassesOf]: Connection to rosbridge closed');
             }

             rosWS.onmessage = function(event){
               console.log('[speech-detection-sphinx4]: Received message from rosbridge');
               var resp_msg = craft_response( event.value ); 
               //console.log(resp_msg);
               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket 
               respFlag = true;
               randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
               console.log("[Ontology-subclassesOf]: Returning to client");
             }
           }
           catch(e){
             rosbridge_connection = false;
             console.error('[Ontology-subclassesOf] ---> ERROR: Cannot open websocket' + 
               'to rosbridge --> [ws//localhost:9090]' );
             console.log(e);
             var resp_msg = craft_error_response(); 
             sendResponse( JSON.stringify(resp_msg) ); 
             console.log("[Ontology-subclassesOf]: Returning to client with error");
             return
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
 * @brief Crafts the form/format for the message to be returned
 * from the faceDetection hop-service.
 * @param srvMsg Return message from ROS Service.
 * return Message to be returned from the hop-service
 */
function craft_response(srvMsg)
{
  // TODO --Implement
  var results = JSON.parse(srvMsg).values.results;
  var trace = JSON.parse(srvMsg).values.trace;
  var success = JSON.parse(srvMsg).values.success;
  var error = JSON.parse(srvMsg).values.error;
  var call_result = JSON.parse(srvMsg).result;

  var craftedMsg = {results: [], trace: [], error: ''};

  if (call_result)
  {
    for (var ii = 0; ii < results.length; ii++)
    {
      craftedMsg.results.push(results[ii]);
    }
    for (var ii = 0; ii < trace.length; ii++)
    {
      craftedMsg.trace.push(trace[ii]);
    }
    craftedMsg.error = error;
  }
  else
  {
    craftedMsg.error = "RAPP Platform Failure";
  }

  //console.log(craftedMsg);
  return JSON.stringify(craftedMsg);
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var craftedMsg = {results: [], trace: [], error: 'RAPP Platform Failure'};
  return JSON.stringify(craftedMsg);
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


