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
var total = 0; 
var fs = require('fs');

/*#######################################################################*/

/*!
 * @brief File Streaming Class/Object definition.
 */
function fileStream(){
  total++;
  this.kind = 'fileStream';

  /*-----Static methods goes here (this.getX = function(){})*/
};


/*#######################-<Static Methods/Members>-##########################*/

/*!
 * @brief Wrapping Node.js readFileSync function. Static member method.
 * @param _file File to be read, specified by path.
 * @return Returns data readen from file.
 */
fileStream.readSync = function (_file){
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
 * @param _destPath File to write the data, specified by path.
 * @param _data Data to be written.
 * @return Undefined.
 */
fileStream.writeSync = function (_destPath, _data)
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
 * @brief Wrapping Node.js unlinkSync function. Static member method.
 * @param _filePath File to be removed, specified by path.
 * @return True if file existed and removed, false otherwise.
 */
fileStream.rmSync = function (_filePath){
  var _msg = new String("Successfully deleted file: ");
  if(fs.existsSync(_filePath)){
    fs.unlinkSync(_filePath);
    return true;
  }
  else{
    console.log("File [%s] does not exist!", _filePath);
    return false;
  }
};

/*########################################################################*/

//exports the class as a module
module.exports = fileStream; 
