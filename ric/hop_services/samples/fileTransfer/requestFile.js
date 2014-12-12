
var fsbin = require("./fsbinary.js");
import service serveFile (name);

/*!
 * @brief Server Parameters Prototype 
 * @param asynch  TRUE for asynchronous file transfer request, false otherwise.
 * @param host    Host's name
 * @param port    Server listening port.
 * @param user    Login username.
 * @param psswd   Login password.
 */  
function serverParams(asynch, host, port, user, psswd) {
  this.asynchronous = asynch;
  this.hostName = host;
  this.port = port;
  this.userName = user;
  this.password = psswd;
}

/*!
 * @brief   Call this to request a file transmission.
 * @usage   This method can be called at any time.
 * @param _file   Requested file name. 
 * @param _fileDestPath   Destination Path to store the requested file (locally).
 * @param _serverParams   Server parameters (look at serverParams Prototype)
 */
function requestFile(_file, _DestPath, _serverParams) {
  console.log("Loaded custom binary file stream method @", fsbin);  
  serveFile (_file).post(
    function(data){
      console.log("File transfered");
      fsbin.writeFileSync(_DestPath, data);
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
}


/**
 * Simple file transfer test
 * Tested -> OK!
 */
var params = new serverParams(false, "localhost", 9001, "", "");
var file = new String();
file = "Robot_Blue.jpg";
destPath = "Robot_Blue_copy.jpg"
requestFile(file,destPath,params);
