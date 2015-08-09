/**
 * This service is used to upload files under request.
 *
 * A client-side serveFile service must be up and running!!!!
 *
 */
var user = process.env.LOGNAME;
var module_path = '../utilities/js/'

var hop = require ( 'hop' );
/*<Require File Utilities module>*/
var Fs = require( module_path + 'fileUtils.js' );
var hopServices = require( module_path + 'hopServices.js' ); 

var hop = new hopServices();

var startT, endT;

//import service serveFile ( _filePath );

/*!
 * @brief Upload file to cloud service.
 * @param _filePath File name to be uploaded, provided by path.
 * @param  
 *
 * Requested file name might be provided without path in the future. 
 * This means that the user might me authenticated to have access to specific 
 * directories.
 */
service uploadFile( _filePath, _destPath, _clientParams ) 
{  
  /*---<Resolves the path where the file will be stored>---*/
  //var destpath = Fs.resolvePath( _destPath );
  //var file = uploadFile.resource (_filePath);
  startT = new Date();
  hop.serveFile( _filePath, _destPath, _clientParams );
  endT = new Date();
}


