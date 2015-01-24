
var version = "0.0.1"; //Current module version

var fs = require('fs');
var Path = require('path');
/*!
 * @brief Returns module version number.
 * @return Module Version.
 */
function getVersion()
{
  return version;
};


/*!
 * @brief Resolve input path to absolute path.
 * @param _path Path to be resolved to absolute.
 * @return Resolved absolute path.
 */
function resolvePath(_path)
{
  var regexp = /~/g;
  var newPath = '';
  if ( _path.match(regexp) )
  {
    var user = process.env.LOGNAME;
    /*<Replaces "~" with "/home/user">*/
    newPath = _path.replace(regexp, '/home/' + user);
  }
  else
  {
    newPath = Path.resolve(_path);
  }
  return newPath;
};


/*!
 * @brief Wrapping Node.js readFileSync function.
 * @param _file File to be read, specified by path.
 * @return Returns data readen from file.
 */
function readFileSync( _file, _encoding, _debug )
{
  var path = resolvePath(_file);
  if(fs.existsSync(path))
  {
    console.log("\033[0;33mReading requested file: [%s]\033[0;0m", path);
    return fs.readFileSync(path);
  }
  else
  {
    console.log("\033[01;31mCannot access the requested file. File does not exist.\033[0;0m");
    return 0;
  }
};




/*!
 * @brief Wrapping Node.js writeFileSync function
 * @param _dest Destination file name to write the data, specified by path.
 * @param _data Data to be written.
 * @return Undefined.
 */
function writeFileSync(_dest, _data, _encoding, _debug)
{
  var path =  resolvePath(_dest);
  if(fs.existsSync(path)){
    console.log("\033[0;36mFile [%s] allready exists. Overwriting...\033[0;0m", path);
  }
  else{
    console.log("\033[0;36mWriting requested data @ [%s]\033[0;0m", path);
  }

  fs.writeFileSync(path, _data);
  console.log("\033[0;36mFinished writing requested data @ [%s]\033[0;0m", path);
};


/*!
 * @brief Wrapping Node.js unlinkSync function.
 * @param _file File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
function rmFileSync( _file, _debug )
{
  var path =  resolvePath(_file);
  if(fs.existsSync(path))
  {
    fs.unlinkSync(path);
    if ( _debug && _debug==true ){
      console.log("Successfully deleted file: [%s]", path);
    }
    return true;
  }
  else
  {
    if ( _debug && _debug==true ){
      console.log("\033[0;31mFile [%s] does not exist!\033[0;0m", path);
    }
    return false;
  }
};


/*!
 * @brief Reads the contents of a given directory path.
 * @param _dir Directory path. Works both with relative and absolute paths.
 * @return List of the contents of the specific directory (Array).
 */
function getFilesListSync( _dir )
{
  var fileList = [];
  var dir = resolvePath( _dir );
  var files = fs.readdirSync(dir);
  for(var i in files)
  {
    var fullPath = dir + '/' + files[i];
    if (fs.statSync(fullPath).isDirectory())
    {
      continue;
    }
    else{
      fileList.push( files[i] );
    }
  }
  return fileList;
};


/*!
 * @brief Writes ascii encoded strings in a give file.
 * @param _data Data to be written. Can be both a buffer or string.
 * @param _filePath Destination file path.
 * @return Undefined.
 */
function text2File ( _data, _filePath ){
  if ( Buffer.isBuffer( _data ) ){
    var data = _data;
  }
  else if ( typeof _data == 'string' ){
    var data = new Buffer( _data.length );
    data.write( _data );
  }
  else{
    console.log( "\033[01;31mInvalid Type of input parameter." +
      "Only String and Buffer data are valid!\033[0;0m" );
    return;
  }

  var fd = fs.openSync( _filePath, 'w' );
  var numBytes = fs.writeSync( fd, data, 0, data.length, null );
  fs.close( fd );
};


/*!
 * @brief Writes ascii encoded strings in a give file 
 *  with a newLine character at the end of the given string (\n).
 * @param _data Data to be written. Can be both a buffer or string.
 * @param _filePath Destination file path.
 * @return Undefined.
 */
function writeLine ( _data, _filePath ){
  if ( Buffer.isBuffer( _data ) ){
    var data = new Buffer( _data.length + 1 );
    data.write( _data.toString() + '\n' );

  }
  else if ( typeof _data == 'string' ){
    var data = new Buffer( _data.length + 1 );
    data.write( _data + '\n' );
  }
  else{
    console.log( "\033[01;31mInvalid Type of input parameter." + 
     " Only String and Buffer data are valid!\033[0;0m" );
    return;
  }

  var fd = fs.openSync( _filePath, 'a' );
  var numBytes = fs.writeSync( fd, data, 0, data.length, null );
  fs.close( fd );
}


/*!
 * @brief Getting File Size without Reading Entire File.
 * @param _fileURL File System Url.
 * @return Size of the file in bytes.
 */
function getFilesizeInBytes( _fileURL ) {
  var path =  resolvePath( _fileURL );
  var stats = fs.statSync( path );
  var fileSizeInBytes = stats["size"];
 return fileSizeInBytes;
}


/*!
 * @brief fileUtils module exports.
 */
module.exports = {
  version: getVersion,
  resolvePath: resolvePath,
  readFileSync: readFileSync,
  readBinFileSync: readBinFileSync,
  writeFileSync: writeFileSync,
  writeBinFileSync: writeBinFileSync,
  rmFileSync: rmFileSync,
  getFilesListSync: getFilesListSync,
  text2File: text2File,
  writeLine: writeLine,
  getFilesizeInBytes: getFilesizeInBytes
}
