/*!
 * @file fileStream.js
 * @brief fileStreamer prototype/class definition.
 *
 * Holds methods used for transfering data from and to a file.
 * Must initialize a fileStream object (Dynamic).
 *
 */


/*######################-<Private Variables here>-#######################*/
var fs = require('fs');

/*#######################################################################*/


/*######################-<Private Functions here>-#######################*/

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


/*#######################-<Prorotype Methods>-##########################*/

/*!
 * @brief 
 */
fileStream.prototype.getTotalObjects = function(){
  return total;
};

/*!
 * @brief Prototype kind get(er)
 * @return Prototype kind.
 */
fileStream.prototype.getKind = function() {
  return this.kind;
};

/*!
 * @brief Wrapping Node.js readFileSync function.
 * @param _file File to be read, specified by path.
 * @return Returns data readen from file.
 */
fileStream.prototype.readSync = function (_file){
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
 * @brief Wrapping Node.js writeFileSync function.
 * @param _destPath File to write the data, specified by path.
 * @param _data Data to be written.
 * @return Undefined.
 */
fileStream.prototype.writeSync = function (_destPath, _data)
{
  if(fs.existsSync(_destPath)){
    console.log("\033[01;34mFile [%s] allready exists. Overwriting...", _destPath);
  }
  else{
    console.log("\033[01;34mWriting requested data @ [%s]", _destPath);
  }
  fs.writeFileSync(_destPath, _data);
};

/*!
 * @brief Wrapping Node.js unlinkSync function.
 * @param _filePath File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
fileStream.prototype.rmSync = function (_filePath){
  if(fs.existsSync(_filePath)){
    fs.unlinkSync(_filePath);
    console.log("Successfully deleted file: [%s]", _filePath);
    return true;
  }
  else{
    console.log("File [%s] does not exist!", _filePath);
    return false;
  }
};

fileStream.prototype.resolve = function(_path){
  var _newPath = _path;  
  /*<Regular expression to match "~" from _path string>*/
  var regexp = /~/g;

  if ( _path.match(regexp) ){
        /*<Get username of currently loged in user>*/
    var _user = process.env.LOGNAME;
    var _newPath = _path.replace(regexp, '/home/' + _user);
  }
  return _newPath;
}


/*########################################################################*/

//exports the class as a module
module.exports = fileStream; 

