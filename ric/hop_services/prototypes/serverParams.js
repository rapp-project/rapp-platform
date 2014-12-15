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
};

module.exports = serverParams; 
