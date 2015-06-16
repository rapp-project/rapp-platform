/*!
 * @file ontology_superclasses_of.service.js
 * @brief Ontology query "SuperclassesOf" hop service.
 *
 */

console.log("Initiated Ontology-Superclasses-Of front-end service");

// TODO -- Get ontology_subclassesOf rosservice name

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var module_path = '../utilities/js/'
/*----------------------------------------------*/
var RandStringGen = require ( module_path + 'randStringGen.js' );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var ros_service_name = "/rapp/rapp_knowrob_wrapper/superclasses_of";
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
 * @brief Ontology-SuperclassesOf database query, HOP Service Core.
 *
 * @param query Ontology query given in a string format
 * @return Results. 
 */
service ontology_superclasses_of ( {query:''} )
{
  var randStr = randStrGen.createUnique();
  console.log("[Ontology-Superclasses-Of]: Client Request");
  console.log('[Ontology-Superclasses-Of]: Query -->', query);

 /*----------------------------------------------------------------- */
 var respFlag = false;
 return hop.HTTPResponseAsync(
   function( sendResponse ) { 
 
     var args = {};
     args[ "ontology_class" ] = query;

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
        console.error('[Ontology-Superclasses-Of] ERROR: Cannot open websocket to rosbridge' +  
          '--> [ws//localhost:9090]' );
        // Print exception 
        console.log(e);
        // Craft return to client message
        var resp_msg = craft_error_response();
        // Return to Client
        sendResponse( resp_msg ); 
        console.log("[Ontology-Superclasses-Of]: Returning to client with error");
        return
      }
      /* ----------------------------------------------------------------- */
     
      /* ------- Add into a try/catch block to ensure safe access -------- */
      try{
        // Implement WebSocket.onopen callback
        rosWS.onopen = function(){
          rosbridge_connection = true;
          console.log('[Ontology-Superclasses-Of]: Connection to rosbridge established');
          this.send(JSON.stringify(rosbridge_msg));
        }
        // Implement WebSocket.onclose callback
        rosWS.onclose = function(){
          console.log('[Ontology-Superclasses-Of]: Connection to rosbridge closed');
        }
        // Implement WebSocket.message callback
        rosWS.onmessage = function(event){
          console.log('[Ontology-Superclasses-Of]: Received message from rosbridge');
          //console.log(event.value);
          var resp_msg = craft_response( event.value ); // Craft response message
          this.close(); // Close websocket 
          rosWS = undefined; // Ensure deletion of websocket
          respFlag = true; // Raise Response-Received Flag

          // Dismiss the unique rossrv-call identity  key for current client
          randStrGen.removeCached( uniqueID ); 
          sendResponse( resp_msg );
          console.log("[Ontology-Superclasses-Of]: Returning to client");
        }
      }
      catch(e){
        rosbridge_connection = false;
        console.error('[Ontology-Superclasses-Of] --> ERROR: Cannot open websocket' + 
          'to rosbridge --> [ws//localhost:9090]' );
        console.log(e);
        var resp_msg = craft_error_response;
        sendResponse( resp_msg ); 
        console.log("[Ontology-Superclasses-Of]: Returning to client with error");
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

           console.log("[Ontology-Superclasses-Of]: Reached rosbridge response timeout" + 
             "---> [%s] ms ... Reconnecting to rosbridge. Retry-%s", 
             elapsed_time.toString(), retries.toString());

           if (retries > max_tries) // Reconnected for max_tries times
           {
             console.log("[Ontology-Superclasses-Of]: Reached max_retries (%s)" + 
               "Could not receive response from rosbridge... Returning to client",
               max_tries);
             var respMsg = craft_error_response();
             sendResponse( respMsg );
             console.log("[Ontology-Superclasses-Of]: Returning to client with error");
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
             console.log('[Ontology-Superclasses-Of]: Connection to rosbridge established');
             this.send(JSON.stringify(rosbridge_msg));
             }

             rosWS.onclose = function(){
               console.log('[Ontology-Superclasses-Of]: Connection to rosbridge closed');
             }

             rosWS.onmessage = function(event){
               console.log('[Ontology-Superclasses-Of]: Received message from rosbridge');
               var resp_msg = craft_response( event.value ); 
               //console.log(resp_msg);
               this.close(); // Close websocket
               rosWS = undefined; // Decostruct websocket 
               respFlag = true;
               randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
               sendResponse( resp_msg ); //Return response to client
               console.log("[Ontology-Superclasses-Of]: Returning to client");
             }
           }
           catch(e){
             rosbridge_connection = false;
             console.error('[Ontology-Superclasses-Of] ---> ERROR: Cannot open websocket' + 
               'to rosbridge --> [ws//localhost:9090]' );
             console.log(e);
             var resp_msg = craft_error_response(); 
             sendResponse( resp_msg ); 
             console.log("[Ontology-Superclasses-Of]: Returning to client with error");
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
 * @param rosbridge_msg Return message from rosbridge.
 * return Message to be returned from the hop-service
 */
function craft_response(rosbridge_msg)
{
  // TODO --Implement
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
  }
  else
  {
    crafted_msg.error = "RAPP Platform Failure"
  }

  //console.log(crafted_msg);
  return JSON.stringify(crafted_msg);
}


/*!
 * @brief Crafts response message on Platform Failure
 */
function craft_error_response()
{
  // Add here to be returned literal
  var crafted_msg = {results: [], trace: [], error: 'RAPP Platform Failure'};
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


