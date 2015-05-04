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
//rosbridge.connect();
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
service ontology_subclassesOf ( queryStr )
{
  console.log("[SubclassesOf]: Client Request");
  rosbridge.connect();
  
  var args = createServiceArgs( JSON.parse(queryStr).toString() );
  /*-----<Call subclassesOf ROS service through rosbridge>-----*/
  var returnMessage = rosbridge.callServiceSync( subclassesOf_rosService, args );
  rosbridge.close();
  queryRet = craftRetMsg(returnMessage);
  /*--<Returned message from qr ROS service>--*/
  return queryRet 
};


/*!
 * @brief Crafts the ROS-Service call form
 * @param queryString The query to db (String)
 * @return Ros-Service call form
 */
function createServiceArgs( queryString )
{
  var query_term = {
    "data": queryString
  };
  var args = {};
  args[ "query_term" ] = query_term;

  return args;
};


/*!
 * @brief Crafts the form/format for the message to be returned
 * from the ontology_subclassesOf hop-service.
 * @param srvMsg Return message from ROS Service.
 * return String vector that contains answers from knowrob db query
 */
function craftRetMsg( srvMsg )
{
  queryAns = srvMsg.values.results;
  var craftedMsg = [];
  for (var ii = 0; ii < queryAns.length; ii++)
  {
    craftedMsg.push(queryAns[ii].data);
  }
  return JSON.stringify(craftedMsg)
};
