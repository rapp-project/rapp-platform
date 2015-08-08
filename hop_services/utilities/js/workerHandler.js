/*!
 * @file master.js
 */

/**
 * Global Module variables are assigned with one underscore "_xx" as an
 * extenstion
 *
 * Global variables are assigned with two underscores "__xx" infront
 * of the variable decleration.
 *
 * Loaded module variables declerations start with Capital letters.
 */

var Rsg_ = require ( './randStringGen.js' );
var RunTime_ = require( './runtime.js' );
var Logger_ = require( './logger.js' );

/* --------< Initiate Random String Generator Module >------- */
var __randStrLength = 5;
var RandStrGen_ = new Rsg_( __randStrLength );
/* ---------------------------------------------------------- */

// Craft unique id for master process
var __masterId = '';

/* ----- < Workers info hold >------ */
var __workers = {};
var __services = [];
var __workerId = {};
/* --------------------------------- */

assignMasterId()

var __logDir = '~/.hop/log/' + __masterId + '-' + RunTime_.getDate() + '/';
Logger_.setLogDir(__logDir);
Logger_.createLogDir();

/*!
 * Assigns A new unique ID to the master process
 */
function assignMasterId()
{
  __masterId = RandStrGen_.createUnique();
}

/*!
 * @brief Registers given worker to logger.
 */
function registerWorkerToLogger(workerName)
{
  Logger_.createLogFile(workerName);
};


/*!
 * @brief Registers the input service.
 */
function registerService(srv)
{
  __services = __services.concat(srv);
};


/*!
 * @brief Used to handle WorkerToMaster messages.
 */
function handleMsg(msg)
{
  var srvName = msg.name;
  var srvId = msg.id;
  var msgId = msg.msgId;
  var data = msg.data;

  var timeNow = RunTime_.getTime();

  switch (msgId)
  {
    case 'log':
      var logMsg = '[' + timeNow + ']: ' + data;
      Logger_.appendToLogFile(srvName, logMsg);
      break;
    case 'up-services':
      var msg = {
        id: __masterId,
        cmdId: 2060,
        data: __services
      };
      __workers[srvName].postMessage(msg);
      break;
    default:
      break;
  }
};


/*!
 * @brief Creates a unique id for worker and call worker to store it.
 */
function setWorkerId(workerName)
{
  var unqId = RandStrGen_.createUnique();

  var msg = {
    id: __masterId,
    cmdId: 2055,
    data: unqId
  };

  // ------- <Send workerId> -------- //
  __workers[ workerName ].postMessage(msg);
  __workerId[ workerName ] = unqId;
};


/*!
 * @brief broadcasts master id to all worker slaves.
 */
function broadcastId()
{
  var msg = {
    id: __masterId,
    cmdId: 2050,
    data: __masterId
  };

  for (var workerName in __workers_)
  {
    __workers_[ workerName ].postMessage(msg);
  }
};


/*!
 * @brief Send masterId to slave worker.
 */
function sendMasterId(workerName)
{
  var msg = {
    id: __masterId,
    cmdId: 2050,
    data: __masterId
  };
  __workers[ workerName ].postMessage(msg);
};



/*!
 * @brief Kill worker given by Name
 */
function killWorker(workerName)
{
  if (serviceExists(workerName))
  {
    __workers[workerName].terminate();
    console.log("Terminated worker: %s", workerName);
  }
}


/*!
 * @brief Register worker services
 * @param worker [{file: <absolute path to worker file>, name: <workerName>}]
 *  (Object literal Array)
 */
function registerWorkerProcess(worker)
{
  if ( (worker.file || worker.name) == undefined)
  {
    console.log("Input worker literal must define both file absolute path + " +
      "worker process name --> worker = {file:'', name: ''}");
    return false;
  }

  try{
  __workers[ worker.name ] = new Worker ( worker.file );
  }
  catch(e){
    console.log("Error on launching worker process ---> file:[%s], name:[%s]",
      worker.file, worker.name);
    console.log("Error ---> %s", e);
    return false;
  }
  registerService( worker.name );

  setWorkerId( worker.name );
  sendMasterId( worker.name );
  registerWorkerToLogger( worker.name );

  console.log( "Created worker for service:" +
    "\033[0;32m [%s]\033[0;0m", worker.name );

  //  Register 'this' worker onmessage callback
  __workers[ worker.name ].onmessage = function(msg){
    handleMsg(msg.data);
  }
}


/*!
 * @brief Checks if service given exists in registered services
 */
function serviceExists(srvName)
{
  if (__services.indexOf(srvName) > -1) {return true;}
  else {return false;}
}

/*!
 * @brief Terminates WorkerHandler. Sends termination signal to workers
 * and finalize logger
 */
function terminate()
{
  for(var i in __services)
  {
    var logMsg = 'Termination signal. Exiting...';
    Logger_.appendToLogFile(__services[i], logMsg);
  }
}


// Export Module
module.exports = {
  registerWorker: registerWorkerProcess,
  terminate: terminate
}
