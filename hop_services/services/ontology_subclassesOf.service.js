/*!
 * @file ontology_subclassesOf.service.js
 * @brief Ontology query "Subclasses Of" hop service.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
/*----------------------------------------------*/
var RandStringGen = require ( rapp_hop_path + "utilities/randStringGen.js" );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var rosService = "/ric/knowrob/subclasses_of";
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
       var resp_msg = event.value;
       sendResponse( resp_msg );
       //console.log(resp_msg);
       this.close();
       rosWS = undefined;
       respFlag = true;
       randStrGen.removeCached( uniqueID );
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
         var resp_msg = event.value; 
         sendResponse( resp_msg ); //Return response to client
         console.log(resp_msg);
         this.close(); // Close the connection to the websocket
         rosWS = undefined; // Decostruct the websocket object
         respFlag = true;
         randStrGen.removeCached( uniqueID ); //Remove the uniqueID so it can be reused
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
