
var rts = require ( './randStringGen.js' );
var runTime = require( './runtime.js' );
var logger = require( './logger.js' );


function Master( )
{
  var randStrLength_ = 5;
  this.randStrGen_ = new rts( randStrLength_ );
  // Craft unique id for master process
  var masterId_ = this.randStrGen_.createUnique();

  //  Workers map
  this.workers_ = {};
  this.services_ = [];
  this.workerId_ = {};

  var logDir_ = '~/.hop/log/';
  logger.setLogDir(logDir_);


  /*!
   * @brief Registers given worker to logger.
   */
  this.registerWorkerToLogger = function(workerName)
  {
    logger.createLogFile(workerName);
  };


  /*!
   * @brief Registers the input service.
   */
  this.registerService = function(srv)
  {
    this.services_ = this.services_.concat(srv);
  };


  /*!
   * @brief Used to handle WorkerToMaster messages.
   */
  this.handleMsg = function (msg)
  {
    var srvName = msg.name;
    var srvId = msg.id;
    var msgId = msg.msgId;
    var data = msg.data;

    var timeNow = runTime.get_time();

    switch (msgId)
    {
      case 'log':
        var logMsg = '[' + timeNow + ']: ' + data;
        logger.appendToLogFile(srvName, logMsg);
        break;
      case 'up-services':
        var msg = {
          id: masterId_,
          cmdId: 2060,
          data: this.services_
        };
        this.workers_[srvName].postMessage(msg);
        break;
      default:
        break;
    }
  };


  /*!
   * @brief Creates a unique id for worker and call worker to store it.
   */
  this.setWorkerId = function(workerName)
  {
    var unqId = this.randStrGen_.createUnique();

    var msg = {
      id: masterId_,
      cmdId: 2055,
      data: unqId
    };

    // ------- <Send workerId> -------- //
    this.workers_[ workerName ].postMessage(msg);
    this.workerId_[ workerName ] = unqId;
  };


  /*!
   * @brief broadcasts master id to all worker slaves.
   */
  this.broadcastId = function()
  {
    var msg = {
      id: masterId_,
      cmdId: 2050,
      data: masterId_
    };

    for (var workerName in this.workers_)
    {
      this.workers_[ workerName ].postMessage(msg);
    }
  };


  /*!
   * @brief Send masterId to slave worker.
   */
  this.sendMasterId = function(workerName)
  {
    var msg = {
      id: masterId_,
      cmdId: 2050,
      data: masterId_
    };
    this.workers_[ workerName ].postMessage(msg);
  };
}


/*!
 * @brief Master Interface. Handles worker service interfaces
 */
Master.prototype =
{
  /*!
   * @brief Kill worker given by Name
   */
  killWorker: function(workerName)
  {
    if (this.serviceExists(workerName))
    {
      this.workers_[workerName].terminate();
      console.log("Terminated worker: %s", workerName);
    }
  },


  /*!
   * @brief Register worker services
   */
  registerWorkers: function(srvFileList, srvDir, srvList)
  {
    for (var i in srvFileList)
    {
      this.registerService( srvList[i] );
      this.workers_[ srvList[i] ] = new Worker ( srvDir + "/" + srvFileList[i] );
      this.setWorkerId( srvList[i] );
      this.sendMasterId( srvList[i] );
      this.registerWorkerToLogger( srvList[i] );

      console.log( "Created worker for service:" +
        "\033[0;32m[%s]\033[0;0m", srvList[i] );

      _this = this;

      //  Register worker onmessage callback
      this.workers_[ srvList[i] ].onmessage = function(msg){
        //if (__DEBUG__)
        //{
        //console.log("Received message from slave service:\033[0;32m \033[0m"
          //, msg);
        //}
        _this.handleMsg(msg.data);
      }
    }
  },


  /*!
   * @brief Checks if service given exists in registered services
   */
  serviceExists: function(srvName)
  {
    if (this.services_.indexOf(srvName) > -1) {return true;}
    else {return false;}
  }

}

// Export Module
module.exports.Master = Master;
