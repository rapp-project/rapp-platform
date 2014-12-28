/*!
 * @file fileStream_static.js
 * @brief fileStreamer prototype/class definition.
 *
 * Holds methods used for transfering data from and to a file.
 * Methods are implemented as static members. 
 * Implemented method can be accesed via fileStream.foo()
 *
 */

/*##########-<Global use variables>-################################*/
var Fs = require('fs');
var Path = require('path'); 

/*##################################################################*/


/*!
 * @brief File Streaming Class/Object definition.
 */
function fileStream(){
  /*-----Private methods/properties goes here (var getX = {})*/
  var kind = 'fileStream_Static';
  var replace = function(string, find, replace){
    return str.replace(new RegExp(find, 'g'), replace);
  }
  /*--------------------------------------------------------*/

  /*-----Privileged methods goes here (this.getX = function(){})*/
  this.getKind = function(){
    return kind;
  }
  /*--------------------------------------------------------*/
};


/*###################-<Static Methods/Members>-####################*/

/*!
 * @brief Wrapping Node.js readFileSync function. Static member method.
 * @param _file File to be read, specified by path.
 * @return Returns data readen from file.
 */
fileStream.readSync = function (_file){
  var path =  this.resolvePath(_file);
  if(Fs.existsSync(_file)){
    console.log("\033[0;33mReading requested file: %s", _file);
    return Fs.readFileSync(_file);
  }
  else{
    console.log("\033[0;33mCannot access the requested file. File does not exist.");
    return 0;
  }
};


/*!
 * @brief Wrapping Node.js writeFileSync function. Static member method.
 * @param _dest Destination file to write the data, specified by path.
 * @param _data Data to be written.
 * @return Undefined.
 */
fileStream.writeSync = function (_dest, _data){
  var path =  this.resolvePath(_dest);
  if(Fs.existsSync(path)){
    console.log("\033[0;36mFile [%s] allready exists. Overwriting...", path);
  }
  else{
    console.log("\033[0;36mWriting requested data @ [%s]", path);
  }
  Fs.writeFileSync(path, _data);
};


/*!
 * @brief Wrapping Node.js unlinkSync function. Static member method.
 * @param _file File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
fileStream.rmSync = function (_file){
  var path =  this.resolvePath(_file);
  if(Fs.existsSync(path)){
    Fs.unlinkSync(path);
    console.log("Successfully deleted file: [%s]", path);
    return true;
  }
  else{
    console.log("\033[0;31mFile [%s] does not exist!", path);
    return false;
  }
};


/*!
 * @brief Resolve input path to absolute path.
 * @param _path Path to be resolved to absolute.
 * @return Resolved absolute path.
 */
fileStream.resolvePath = function(_path){
  /*<Regular expression to match "~" from _path string>*/
  var regexp = /~/g;
  var newPath = '';
  if ( _path.match(regexp) ){
        /*<Get username of currently loged in user>*/
    var user = process.env.LOGNAME;
    newPath = _path.replace(regexp, '/home/' + user);
  }
  else{
    newPath = require('path').resolve(_path);
  }
  return newPath;
}


/*########################################################################*/

//exports the class as a module
module.exports = fileStream; 
