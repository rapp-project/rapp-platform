
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";
/*Require File Utilities module => make use of writeBinFileSync function*/
var Fs = require( rapp_hop_path + "utilities/./fileUtils.js" );


/*!
 * @brief Service to send binary files from local to host.
 *
 * @param _fileName Path+Name of the file where binary data will be stored.
 * @param _data File data (BINARY).
 *
 * May change to store requested data at a standard authenticated directory 
 */
service storeFile ( _destPath, _data )
{
  //var destPath = Fs.resolvePath(_destPath);
  console.log("\033[01;36mReceived File in binary encoding\033[0;0m");
  Fs.writeBinFileSync( _destPath, _data );
}
