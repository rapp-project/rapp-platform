
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user 
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
var module_path = rapp_hop_path +  'utilities/js/'

/*Require File Utilities module => make use of writeBinFileSync function*/
var Fs = require( module_path + 'fileUtils.js' );


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
