
/*######-<Private Variables here>-#######*/
var total = 0; 


/*#######################################*/

/*!
 * @brief Server Parameters Class - Constructor
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
  this.fail = function(err){
    console.log("\033[01;31m[ERROR] Connection refused: \033[0;0m", err);
  }
};

/*###########-<Class Methods>-##########*/


/*######################################*/

//exports the class
module.exports = serverParams; 
