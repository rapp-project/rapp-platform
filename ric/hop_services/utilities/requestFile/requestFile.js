var fsbin = require("./fsbinary.js");
import service serveFile_relative (name);

/*!
 * @brief   Call this to request a file transmission.
 * @usage   This method can be called at any time.
 * @param _file   Requested file name. 
 * @param _fileDestPath   Destination Path to store the requested file (locally).
 * @param _serverParams   Server parameters (look at serverParams Prototype)
 */
function requestFile(_file, _DestPath, _serverParams) {
  console.log("Loaded custom binary file stream method @", fsbin);  
  /*var file = */serveFile_relative (_file).post(
    function(data){
      console.log("File transfered");
      fsbin.writeFileSync(_DestPath, data);
      //return data;
    },
    {
      asynchronous: /*false*/_serverParams.asynchronous,
      host: _serverParams.host,
      port: _serverParams.port,
      user: _serverParams.user,
      password: _serverParams.password,
      fail: function(err) {console.log("Connection refused: ", err)}
    }
  );
  //return file;
}

exports.requestFile = requestFile;
