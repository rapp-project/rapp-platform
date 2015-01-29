/*!
 * @file speech2Text.service.js
 * @brief SpeechToText hop service running on Remote host.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";
/*----------------------------------------------*/
/*--------------Load required modules-----------*/
var Fs = require( /*rapp_hop_path +*/ "../utilities/./fileUtils.js" );
var ROSbridge = require(/*rapp_hop_path +*/"../utilities/./rosbridge.js");
var RandStringGen = require ( /*rapp_hop_path +*/ "../utilities/./randStringGen.js" );
/*----------------------------------------------*/
/*-----<Defined Name of QR Node ROS service>----*/
var speech2TextRosService = "/ric/speech_to_text_service";
/*--<Defines the directory where images received are stored>--*/
var storePath = "/home/klpanagi/hop_temps/"; 
/*---Initiatess Communication with RosBridge (Global)---*/
var rosbridge = new ROSbridge();
//ros.init_bridge('');
/*------------------------------------------------------*/
/*----<Random String Generator configurations---->*/
var stringLength = 5;
var randStrGen = new RandStringGen( stringLength );
/*------------------------------------------------*/


/*!
 * @brief Speech2Text HOP Service Core.
 *
 * @param _data Audio data in BINARY encoding/format.
 *
 * @return Message response from speech2Text ROS Node service.
 */
service speech2Text ( _data )
{
  rosbridge.connect();
  var randStr = randStrGen.createUnique();
  var fileName = "speech-" + randStr + ".flac";
  console.log("[Speech2Text] Client Request");
  
  var audioFileUrl = Fs.resolvePath( storePath + fileName );
  var args = createServiceArgs( audioFileUrl );
  /*-----<Stores received image data>-----*/
  Fs.writeFileSync( audioFileUrl, _data.data );
  /*-----<Call FaceDetection ROS service through rosbridge>-----*/
  //var rosbridge = new ROSbridge();
  //rosbridge.connect();
  var returnMessage = rosbridge.callServiceSync( speech2TextRosService, args );
  rosbridge.close();
  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  return returnMessage; 
};


function createServiceArgs( _audioFileUrl )
{
  var filename = {
    "data": _audioFileUrl
  };
  var args = {};
  args[ "filename" ] = filename;

  return args;
};
