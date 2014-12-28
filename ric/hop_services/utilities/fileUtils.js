
var version = "0.0.1"; //Current module version

var Fs = require('fs');
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
  if(Fs.existsSync(_file))
  {
    console.log("\033[0;33mReading requested file: [%s]\033[0;0m", _file);
    return Fs.readFileSync(_file);
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
function writeFileSync(_dest, _data)
{
  var path =  resolvePath(_dest);
  if(Fs.existsSync(path))
  {
    console.log("\033[0;36mFile [%s] allready exists. Overwriting...\033[0;0m", path);
  }
  else
  {
    console.log("\033[0;36mWriting requested data @ [%s]\033[0;0m", path);
  }
  Fs.writeFileSync(path, _data);
};


/*!
 * @brief Wrapping Node.js unlinkSync function.
 * @param _file File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
function rmFileSync(_file)
{
  var path =  resolvePath(_file);
  if(Fs.existsSync(path))
  {
    Fs.unlinkSync(path);
    console.log("Successfully deleted file: [%s]", path);
    return true;
  }
  else
  {
    console.log("\033[0;31mFile [%s] does not exist!\033[0;0m", path);
    return false;
  }
};


/*!
 * @brief fileUtils module exports
 */
module.exports = {
  version: getVersion,
  resolvePath: resolvePath,
  readFileSync: readFileSync,
  writeFileSync: writeFileSync,
  rmFileSync: rmFileSync
}
