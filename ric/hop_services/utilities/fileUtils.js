
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
function readFileSync(_file)
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
 * @brief Reads the contents of a file and stores them in a binary format.
 * @param _file File to be read, specified by path.
 * @return Returns binary data readen from file.
 */
function readBinFileSync(_file)
{
  var path = resolvePath(_file);
  if(fs.existsSync(path))
  {
    console.log("\033[0;33mReading requested file (binary encoding): [%s]\033[0;0m", path);
    var datadatafer = fs.readFileSync(path);
    var dataBin = datadatafer.toString('binary', 0, datadatafer.length);
    return dataBin;
  }
  else
  {
    console.log("\033[01;31mCannot access the requested file. File does not exist.\033[0;0m");
    return 0;
  }
}


/*!
 * @brief Wrapping Node.js writeFileSync function
 * @param _dest Destination file name to write the data, specified by path.
 * @param _data Data to be written.
 * @return Undefined.
 */
function writeFileSync(_dest, _data)
{
  var path =  resolvePath(_dest);
  if(fs.existsSync(path)){
    console.log("\033[0;36mFile [%s] allready exists. Overwriting...\033[0;0m", path);
  }
  else{
    console.log("\033[0;36mWriting requested data @ [%s]\033[0;0m", path);
  }
  fs.writeFileSync(path, _data);
};


/*!
 * @brief Writes binary data to a file. 
 * @param _dest Destination file name to write the data, specified by path.
 * @param _data Data to be written.
 * @return Undefined.
 */
function writeBinFileSync( _dest, _data )
{
  var path =  resolvePath(_dest);
  if(fs.existsSync(path)){
    console.log("\033[0;36mFile [%s] allready exists. Overwriting...\033[0;0m", path);
  }
  else{
    console.log("\033[0;36mWriting requested data @ [%s]\033[0;0m", path);
  }

  var p = #:open-output-file( #:js-tostring( path, #:%this ) );
  #:display( #:js-tostring( _data, #:%this ), p );
  #:close-output-port( p );
}


/*!
 * @brief Wrapping Node.js unlinkSync function.
 * @param _file File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
function rmFileSync(_file)
{
  var path =  resolvePath(_file);
  if(fs.existsSync(path))
  {
    fs.unlinkSync(path);
    console.log("Successfully deleted file: [%s]", path);
    return true;
  }
  else
  {
    console.log("\033[0;31mFile [%s] does not exist!\033[0;0m", path);
    return false;
  }
};


function getFilesListSync(_dir)
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


function text2File ( _data, _filePath ){
  if ( Buffer.isBuffer( _data ) ){
    var data = _data;
  }
  else if ( typeof _data == 'string' ){
    var data = new Buffer( _data.length );
    data.write( _data );
  }
  else{
    console.log( "\033[01;31mInvalid Type of input parameter. Only String and datafer data are valid!\033[0;0m" );
    return;
  }

  var fd = fs.openSync( _filePath, 'w' );
  var numBytes = fs.writeSync( fd, data, 0, data.length, null );
  fs.close(fd);
};


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
    console.log( "\033[01;31mInvalid Type of input parameter. Only String and datafer data are valid!\033[0;0m" );
    return;
  }

  var fd = fs.openSync( _filePath, 'a' );
  var numBytes = fs.writeSync( fd, data, 0, data.length, null );
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
  writeLine: writeLine
}
