/*!
 * @file fileStream.js
 * @brief fileStreamer prototype/class definition.
 *
 * Holds methods used for transfering data from and to a file.
 * Must initialize a fileStream object (Dynamic).
 *
 */


/*######################-<Global Variables here>-#######################*/
//var this.fsModule() = require('this.fsModule()');
//var Path = require('path'); 

/*#######################################################################*/

/*!
 * @brief File Streaming Class/Object definition.
 */
function fileStream(){
  /*-----Private methods/properties goes here (var getX = {})----*/
  var kind_ = 'fileStream';
  var Path_ = require('path');
  var Fs_ = require('fs');

  var replace = function(string, find, replace){
    return str.replace(new RegExp(find, 'g'), replace);
  }

  /*-------------------------------------------------------------*/

  /*---Privileged methods goes here (this.getX = function(){})---*/
  
  /*!
   * @brief Prototype kind get(er).
   * @return Prototype kind value ('fileStream').
   */ 
  this.getKind = function(){
    return kind_;
  } 

  /*!
   * @brief Access path module object.
   * @return path module object variable.
   */ 
  this.pathModule = function(){
    return Path_;
  }

  /*!
   * @brief Access fs module object.
   * @return fs module object variable.
   */ 
  this.fsModule = function(){
    return Fs_;
  }
  /*-------------------------------------------------------------*/
};


/*#######################-<Prorotype Methods>-##########################*/

/*!
 * @brief Prototype kind get(er)
 * @return Prototype kind.
 */
fileStream.prototype.getKind = function() {
  return kind;
};


/*!
 * @brief Wrapping Node.js readFileSync function.
 * @param _file File to be read, specified by path.
 * @return Returns data readen from file.
 */
fileStream.prototype.readSync = function (_file){
  var path =  this.resolvePath(_file);
  if(this.fsModule().existsSync(path)){
    console.log("\033[0;33mReading requested file: %s", path);
    return this.fsModule().readFileSync(path);
  }
  else{
    console.log("\033[0;31mCannot access the requested file. File does not exist.");
    return 0;
  }
};


/*!
 * @brief Wrapping Node.js writeFileSync function.
 * @param _dest File to write the data, specified by path.
 * @param _data Data to be written.
 * @return Undefined.
 */
fileStream.prototype.writeSync = function (_dest, _data)
{
  var path =  this.resolvePath(_dest);
  if(this.fsModule().existsSync(path)){
    console.log("\033[0;36mFile [%s] allready exists. Overwriting...", path);
  }
  else{
    console.log("\033[0;36mWriting requested data @ [%s]", path);
  }
  this.fsModule().writeFileSync(path, _data);
};


/*!
 * @brief Wrapping Node.js unlinkSync function.
 * @param _file File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
fileStream.prototype.rmSync = function (_file){
  var path =  this.resolvePath(_file);
  if(this.fsModule().existsSync(path)){
    this.fsModule().unlinkSync(path);
    console.log("\033[0;35mSuccessfully deleted file: [%s]", path);
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
fileStream.prototype.resolvePath = function(_path){
  var newPath = '';  
  /*<Regular expression to match "~" from _path string>*/
  var regexp = /~/g;

  if ( _path.match(regexp) ){
    /*<Get username of currently loged in user>*/
    var user = process.env.LOGNAME;
    var newPath = _path.replace(regexp, '/home/' + user);
  }
  else{
    newPath = this.pathModule().resolve(_path);
  }
  return newPath;
}


/*########################################################################*/

//exports the class as a module
module.exports = fileStream; 

