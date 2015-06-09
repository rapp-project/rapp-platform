/*!
 * @file ontology_subclassesOf.service.js
 * @brief Ontology query "Subclasses Of" hop service.
 * @bug Currently NOT Operational!!
 *
 */

console.log("Initiated Ontology-SubclassesOf front-end service");

// TODO -- Get ontology_subclassesOf rosservice name

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var module_path = '../utilities/js/'
/*----------------------------------------------*/
var RandStringGen = require ( module_path + 'randStringGen.js' );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var rosService = "/rapp/rapp_knowrob_wrapper/subclasses_of";
var hop = require('hop');
/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/


/*!
 * @brief Ontology SubclassOf database query, HOP Service Core.
 *
 * @param queryString Ontology query given in a string format
 * @return Message response from speech2Text ROS Node service. --JSON--
 */
service ontology_subclassesOf ( {queryStr:''} )
{
  var randStr = randStrGen.createUnique();
  console.log("[Ontology-subclassesOf]: Client Request");
  console.log('[Ontology-subclassesOf]: Query -->', queryStr);

  /* --< Perform renaming on the reived file. Add uniqueId value> --- */
  var unqExt = randStrGen.createUnique();
  randStrGen.removeCached(unqExt);

  
 /*----------------------------------------------------------------- */
 var respFlag = false;
 return hop.HTTPResponseAsync(
   function( sendResponse ) { 

     var args = craft_srv_msg(queryStr);
     var uniqueID = randStrGen.createUnique();
     var ros_srv_call = {
       'op': 'call_service',
        'service': rosService,
        'args': args,
        'id': uniqueID
     };

     var rosWS = new WebSocket('ws://localhost:9090');
     rosWS.onopen = function(){
       console.log('[Ontology-subclassesOf]: Connection to rosbridge established');
       this.send(JSON.stringify(ros_srv_call));
     }
     rosWS.onclose = function(){
       console.log('[Ontology-subclassesOf]: Connection to rosbridge closed');
     }
     rosWS.onmessage = function(event){
       console.log('[Ontology-subclassesOf]: Received message from rosbridge');
       //console.log(event.value);
       var resp_msg = craft_response(event.value);
       this.close();
       rosWS = undefined;
       respFlag = true;
       randStrGen.removeCached( uniqueID );
       sendResponse( resp_msg );
     }

     function asyncWrap(){
       setTimeout( function(){
         if (respFlag != true){
           console.log('[Ontology-subclassesOf]: Connection timed out! rosWs = undefined');
           //sendResponse('Timeout');
           if (rosWS != undefined)
       {
         rosWS.close();
       }
       rosWS = undefined;
       /* --< Re-open connection to the WebSocket >--*/
       rosWS = new WebSocket('ws://localhost:9090');
       /* -----------< Redefine WebSocket callbacks >----------- */
       rosWS.onopen = function(){
         console.log('[Ontology-subclassesOf]: Connection to rosbridge established');
         this.send(JSON.stringify(ros_srv_call));
       }

       rosWS.onclose = function(){
         console.log('[Ontology-subclassesOf]: Connection to rosbridge closed');
       }

       rosWS.onmessage = function(event){
         console.log('[Ontology-subclassesOf]: Received message from rosbridge');
         var resp_msg = craft_response(event.value);
         //console.log(resp_msg);
         this.close(); // Close the connection to the websocket
         rosWS = undefined; // Decostruct the websocket object
         respFlag = true;
         randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
         sendResponse( resp_msg ); //Return response to client
       }
       /*--------------------------------------------------------*/
       asyncWrap();
         }
       }, 8000); //Timeout value is set at 10 seconds
     }
     asyncWrap();

   }, this ); // do not forget the <this> argument of hop.HTTResponseAsync 
};



/*!
 *  * @brief Crafts the ROS-Service call form
 *   * @param queryString The query to db (String)
 *    * @return Ros-Service call form
 *     */
function craft_srv_msg( queryString )
{
  var query_term = {
    "data": queryString
  };
  var args = {};
  args[ "ontology_class" ] = query_term;

  return args;
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

  for (var ii = 0; ii < results.length; ii++)
  {
    craftedMsg.results.push(results[ii].data);
  }
  for (var ii = 0; ii < trace.length; ii++)
  {
    craftedMsg.trace.push(trace[ii].data);
  }
  craftedMsg.error = error
  return JSON.stringify(craftedMsg);
}
