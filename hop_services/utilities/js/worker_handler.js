/*!
 * @file worker_handler.js
 * @brief TODO
 */

var rts = require ( './randStringGen.js' );
var randStrLength
var randStrGen = new rts( randStrLength );

// Craft unique id for master process
var masterId = randStrGen.createUnique();



//  Workers map
var workers = {};

var services = [];


function register_services(srvList)
{
  services = srvList;
}


function service_exists(srvName)
{
  if (services.indexOf(srvName) > -1) {return true;}
  else {return false;}
}


function register_workers(srvFileList, srvDir, srvList)
{
  register_services(srvList);
  for (var i in srvFileList)
  {
    workers[ srvList[i] ] = new Worker ( srvDir + "/" + srvFileList[i] );

    // ------- <Send workerId> -------- //
    var msg = {
      id: masterId,
      cmdId: 2055,
      data: set_worker_id()
    };
    workers[ srvList[i] ].postMessage(msg);
    // -------------------------------- //

    console.log( "Created worker for service:" +
      "\033[0;32m[%s]\033[0;0m", srvList[i] );

    //  Register worker onmessage callback
    workers[ srvList[i] ].onmessage = function(msg){
      //if (__DEBUG__)
      //{
      console.log("Received message from slave service:\033[0;32m \033[0m"
        , msg);
      //}
      //worker_msg_handler(msg.data);
    }
  }
};


function kill_worker(workerName)
{
  console.log("Received termination command for worker [%s].", workerName);
  if (service_exists(workerName))
  {
    workers[workerName].terminate();
    console.log("Terminated worker: %s", workerName);
  }
};


// Slave -> Master msg format:
// TODO

//function worker_msg_handler(msg)
//{
  //srvName = msg.name;
  //srvId = msg.id;
  //error = msg.error;
  //strMsg = msg.strMsg;

  //console.log(msg);

  //switch (msg.data)
  //{
    //case 'client-request':

      //break;
  //}
//}


function set_worker_id(workerName)
{
  var unqId = randStrGen.createUnique();
  return unqId;
}

module.exports = {
  version: "0.0.1",
  register_workers: register_workers,
  kill_worker: kill_worker
}
