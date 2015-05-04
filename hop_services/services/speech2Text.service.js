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
var speech2TextRosService = "/ric/speech_detection_sphinx4";
var speech2TextConfigRosService = "/ric/speech_detection_sphinx4_configure";
/*--<Defines the directory where images received are stored>--*/
var storePath = "/home/" + user + "/hop_temps/"; 
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
 * @param _file An Object literral that specifies a "data"
 *  property. Data must be raw_binary from buffer.
 *
 * @return Message response from speech2Text ROS Node service.
 */
service speech2Text ( fileData )
{
  rosbridge.connect();
  var randStr = randStrGen.createUnique();
  var fileName = "speech-" + randStr + ".wav";
  console.log("[Speech2Text] Client Request");
  
  var audioFileUrl = Fs.resolvePath( storePath + fileName );
  /*-----<Stores received image data>-----*/
  console.log("Is Buffer?: ", typeof fileData);
  Fs.writeFileSync( audioFileUrl, fileData );
  var args = createServiceArgs( audioFileUrl );

  var vocabulary = ['ναι', 'όχι', 'ίσως'];
  var sentences = ['ναι', 'όχι', 'ίσως'];
  var grammar = [];
  var confArgs = craft_s2tConfig_args('', vocabulary, grammar, sentences);
  /*-----<Call FaceDetection ROS service through rosbridge>-----*/
  var returnMessage = rosbridge.callServiceSync( speech2TextConfigRosService, confArgs, 0 );
  console.log(returnMessage);
  returnMessage = rosbridge.callServiceSync( speech2TextRosService, args, 0 );
  console.log(returnMessage.values.words);
  rosbridge.close();
  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  var wordsFound = returnMessage.values.words;
  return JSON.stringify(wordsFound); 
};

function craft_s2tConfig_args( language, words, grammar, sentences )
{
  var args = {};
  args[ 'words' ] = words;
  args[ 'language' ] = "gr";
  args[ 'grammar' ] = grammar;
  args[ 'sentences' ] = sentences;
  return args
}

function createServiceArgs( _audioFileUrl )
{
  var filename = {
    "data": _audioFileUrl
  };
  var args = {};
  args[ "path" ] = filename;
  return args;
};
