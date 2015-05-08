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
var speech2TextService_batch = "/ric/speech_detection_sphinx4_batch";


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
  Fs.writeFileSync( audioFileUrl, fileData );
  //var args = createServiceArgs( audioFileUrl );

  var vocabulary = ['yes', 'no'];
  var sentences = ['yes', 'no'];
  var grammar = [];
  /* -- ------temp-----------------*/
  var audio_source = "nao_wav_1_ch";

  var confArgs = craft_s2tConfig_args('en', vocabulary, grammar, sentences, audio_source, audioFileUrl);
  
  var recognizedWords = undefined;

  console.log(confArgs);
  var returnMessage = rosbridge.callServiceSync( speech2TextService_batch, confArgs, 0 );
  console.log(returnMessage);
  rosbridge.close();

  /*--<Removes the file after return status from rosbridge>--*/
  Fs.rmFileSync( storePath + fileName );
  randStrGen.removeCached( randStr );
  /*--<Returned message from qr ROS service>--*/
  if (returnMessage.result == true){
    recognizedWords = returnMessage.values.words;
  }
  else{
    recognizedWords = 'Speech2Text Ros node returned undefined!!';
    console.log('Ros service [%s] returned error!!', speech2TextRosService);
  }
  //return JSON.stringify(recognizedWords); 
};


function craft_s2tConfig_args( language, words, grammar, sentences, audio_source, audioFilePath )
{
  var args = {};
  args[ 'words' ] = words;
  args[ 'language' ] = language;
  args[ 'grammar' ] = grammar;
  args[ 'sentences' ] = sentences;
  args[ 'audio_source' ] = audio_source;
  args[ 'path' ] = audioFile_path
  return args;
};


function createServiceArgs( _audioFileUrl )
{
  var filename = {
    "data": _audioFileUrl
  };
  var args = {};
  args[ "path" ] = filename;
  args[ 'audio_source' ] = 'nao_wav_1_ch';// 'nao_ogg' || 'nao_wav_1_ch'
  return args;
};

