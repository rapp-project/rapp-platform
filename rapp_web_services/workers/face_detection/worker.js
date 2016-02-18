var path = require('path');

var ENV = require( path.join(__dirname, '../..', 'env.js') );

// Include it even if not used!!! Sets properties to the thread's global scope.
var workerUtils = require(path.join(ENV.PATHS.INCLUDE_DIR, 'common',
    'worker_utils.js'));

// Set worker thread name under the global scope. (WORKER.name)
workerUtils.setWorkerName('face_detection');

// Declare the worker's onmessage callback function.
onmessage = workerUtils.onMessage;

// Launch all services assigned to this worker thread.
// Search in workers.json config file for assigned web services.
workerUtils.launchSvcAll();
