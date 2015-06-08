
var user = process.env.LOGNAME;
var module_path = '../utilities/js/'

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
