
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";
/*Require File Utilities module => make use of writeBinFileSync function*/
var Fs = require( /*rapp_hop_path +*/ "../utilities/./fileUtils.js" );


/*!
 * @brief Service to send binary files from local to host.
 *
 * @param _fileName Path+Name of the file where binary data will be stored.
 * @param _file An Object literral that specifies a "data"
 *  property. Data must be raw_binary from buffer.
 */
service storeFile ( _destPath, _file )
{
  //var destPath = Fs.resolvePath(_destPath);
  console.log("[StoreFile] Client Request");
  var filesize = Fs.writeFileSync( _destPath, _file.data );
  //return filesize;
}
