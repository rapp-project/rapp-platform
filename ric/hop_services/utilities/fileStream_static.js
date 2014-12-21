/*!
 * @file fileStream_static.js
 * @brief fileStreamer prototype/class definition.
 *
 * Holds methods used for transfering data from and to a file.
 * Methods are implemented as static members. 
 * Implemented method can be accesed via fileStream.foo()
 *
 */


/*######################-<Private Variables here>-#######################*/
var fs = require('fs');

/*#######################################################################*/

/*######################-<Private Functions/Members here>-#######################*/

function replace(string, find, replace){
  return str.replace(new RegExp(find, 'g'), replace);
}

/*#######################################################################*/

/*!
 * @brief File Streaming Class/Object definition.
 */
function fileStream(){
  this.kind = 'fileStream';

  /*-----Static methods goes here (this.getX = function(){})*/

  /*--------------------------------------------------------*/
};


/*#######################-<Static Methods/Members>-##########################*/

/*!
 * @brief Wrapping Node.js readFileSync function. Static member method.
 * @param _file File to be read, specified by path.
 * @return Returns data readen from file.
 */
fileStream.readSync = function (_file){
  var path =  this.resolvePath(_file);
  if(fs.existsSync(_file)){
    console.log("\033[01;33mReading requested file: %s", _file);
    return fs.readFileSync(_file);
  }
  else{
    console.log("\033[01;31mCannot access the requested file. File does not exist.");
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
  if(fs.existsSync(path)){
    console.log("\033[01;34mFile [%s] allready exists. Overwriting...", path);
  }
  else{
    console.log("\033[01;34mWriting requested data @ [%s]", path);
  }
  fs.writeFileSync(path, _data);
};

/*!
 * @brief Wrapping Node.js unlinkSync function. Static member method.
 * @param _file File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
fileStream.rmSync = function (_file){
  var path =  this.resolvePath(_file);
  if(fs.existsSync(path)){
    fs.unlinkSync(path);
    console.log("Successfully deleted file: [%s]", path);
    return true;
  }
  else{
    console.log("File [%s] does not exist!", path);
    return false;
  }
};

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
    newPath = fs.realpathSync(_path);
  }
  return newPath;
}


/*########################################################################*/

//exports the class as a module
module.exports = fileStream; 
