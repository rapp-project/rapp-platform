

var Fs = require('./fileUtils.js');

function HopServiceUtils()
{
  var serverParams_ = {};

  /*!
   * @brief Stored server params get(er).
   * @return Server Parameters object literal.
   */
  this.getServerParams = function(){
    return serverParams_;
  }

  /*!
   * @brief Sets asynchronous value on server parameters object literal.
   * @param value Asynchronous value (Boolean).
   */
  this.setAsync = function( value ){
    serverParams_.asynchronous = value;
  }

  /*!
   * @brief Sets Host value of server parameters object literal.
   * @param hostName Host name value (String).
   */
  this.setHostName = function( hostName ){
    serverParams_.host = hostName;
  }

  /*!
   * @brief Sets Port value of server parameters object literal.
   * @param portNumber Port Number value.
   */
  this.setPortNumber = function( portNumber ){
    serverParams_.port = portNumber;
  }

  /*!
   * @brief Sets User value of server parameters object literal.
   * @param userName User name value (String). 
   */
  this.setUserName = function( userName ){
    serverParams_.user = userName;
  }

  /*!
   * @brief Sets Password value of server parameters object literal.
   * @param passwd User password value (String). 
   */
  this.setPassword = function( passwd ){
    serverParams_.password = passwd;
  }

  /*!
   * @brief Sets fail callback of server parameters object literal.
   * @param fail Connection refused/fail callback function.
   */
  this.setFail = function( fail ){
    serverParams_.fail = fail;
  }

  /*!
   * @brief Sets server parameters object literal.
   * @para serverParams ServerParameters Object.
   */
  this.setServerParams = function ( serverParams ){
    serverParams_ = serverParams;
  }
  
};


HopServiceUtils.prototype.init = function (_serverParams)
{
  this.setServerParams(_serverParams);
  //this.setAsync(_serverParams.asynchronous);
  //this.setHostName(_serverParams.hostName);
  //this.setPortNumber(_serverParams.port);
  //this.setUserName(_serverParams.userName);
  //this.setPassword(_serverParams.password);
};


HopServiceUtils.prototype.sendFile = function (_filePath, _destPath)
{
  /*Call to read file in binary encoding so we can pass it 
    as argument to the service*/
  var dataBin = Fs.readBinFileSync(_filePath);
  /*Resolves destination path to relative if possible*/
  var destPath = Fs.resolvePath(_destPath);

  /*Importing the specific service*/
  import service storeFile (_destPath, _data);
  storeFile(destPath, dataBin).post(
    function(returnMessage){
      //if (returnMessage == true){
        //console.log("\033[0;33mFile data succesfully transfered");
      //}
      //else{
        //console.log("\033[01;31m[ERROR Transfering requested file data]");
      //}
    },
    this.getServerParams()
  );

};


HopServiceUtils.prototype.getFile = function (_filePath, _destPath)
{
  import service serveFile (_filePath);
  /*Resolves from relative to absolute path (if).*/
  var filePath = Fs.resolvePath(_filePath);

  var dataBin = serveFile( filePath ).post(
    function(data)
    {
      console.log("Transmitting Requested file: \033[0;35m[%s]", filePath);
      return data;
    },
    this.getServerParams()
  );
  Fs.writeBinFileSync( _destPath, dataBin);
  return dataBin;
}

HopServiceUtils.prototype.qrNode = function (_qrImagePath)
{
  import service qrNode (_qrImage);
  var qrData = Fs.readBinFileSync( _qrImagePath ); 
  var params = this.getServerParams();
  
  /*-------Call QR_Node service-------*/
  var retMessage = qrNode(qrData).post(
    function(data){
      return data;
    },
    this.getServerParams()
  );
  /*----------------------------------*/
  return retMessage;
}


HopServiceUtils.prototype.faceNode = function (_faceImagePath)
{
  import service faceNode (_faceImage);
  var faceData = Fs.readBinFileSync( _faceImagePath ); 
  var params = this.getServerParams();
  
  /*-------Call QR_Node service-------*/
  var retMessage = faceNode(faceData).post(
    function(data){
      return data;
    },
    this.getServerParams()
  );
  /*----------------------------------*/
  return retMessage;
}


/*Exporting the HopServiceUtils module*/
module.exports = HopServiceUtils;
