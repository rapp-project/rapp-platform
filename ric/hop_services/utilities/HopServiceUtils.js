

var Fs = require('./fileUtils.js');

function HopServiceUtils()
{
  var serverParams_ = {
    asynchronous: false,
    host: "",
    port: 9001,
    user: "",
    password: "",
    fail: function(err){
      console.log("\033[01;31m[ERROR] Connection refused: ", err)
    }
  }

  this.getServerParams = function(){
    return serverParams_;
  }
  
  this.setAsync = function(value){
    serverParams_.asynchronous = value;
  }

  this.setHostName = function(hostName){
    serverParams_.host = hostName;
  }

  this.setPortNumber = function(portNumber){
    serverParams_.port = portNumber;
  }

  this.setUserName = function(userName){
    serverParams_.user = userName;
  }

  this.setPassword = function(passwd){
    serverParams_.password = passwd;
  }
  
};


HopServiceUtils.prototype.init = function (_serverParams)
{
  this.setHostName(_serverParams.host);
  this.setPortNumber(_serverParams.port);
  this.setUserName(_serverParams.user);
  this.setPassword(_serverParams.passwd);
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
  var retMessage = qrNode(qrData).post(
    function(data){
      return data;
    },
    this.getServerParams() 
  );
  return retMessage;
}


/*Exporting the HopServiceUtils module*/
module.exports = HopServiceUtils;
