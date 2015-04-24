/*!
 * @file ontology_subclassesOf.service.js
 * @brief Ontology query "Subclasses Of" hop service.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/rapp_ric/hop_services/";
/*----------------------------------------------*/
var ROSbridge = require(/*rapp_hop_path +*/"../utilities/./rosbridge.js");
var RandStringGen = require ( /*rapp_hop_path +*/ "../utilities/./randStringGen.js" );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var subclassesOf_rosService = "/ric/knowrob/subclasses_of";
/*---Initiatess Communication with RosBridge (Global)---*/
var rosbridge = new ROSbridge();
rosbridge.connect();
/*------------------------------------------------------*/
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
service ontology_subclassesOf ( queryString )
{
  console.log("[SubclassesOf]: Client Request");
  //rosbridge.connect();
  
  var args = createServiceArgs( queryString );
  /*-----<Call subclassesOf ROS service through rosbridge>-----*/
  var returnMessage = rosbridge.callServiceSync( subclassesOf_rosService, args );
  //rosbridge.close();
  /*--<Returned message from qr ROS service>--*/
  return  JSON.stringify( returnMessage.values.results )// JSON msg
};


function createServiceArgs( queryString )
{
  var query_term = {
    "data": queryString
  };
  var args = {};
  args[ "query_term" ] = query_term;

  return args;
};
