/***
 * Copyright 2015 RAPP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Konstantinos Panayiotou
 * Contact: klpanagi@gmail.com
 *
 */


var path = require('path');

var ENV = require( path.join(__dirname, '../..', 'env.js') );
// Include it even if not used!!! Sets properties to the thread's global scope.
var workerUtils = require(path.join(ENV.PATHS.INCLUDE_DIR, 'common',
    'worker_utils.js'));

// Set worker thread name under the global scope. (WORKER.name)
workerUtils.setWorkerName('news_explore');

// Declare the worker's onmessage callback function.
onmessage = workerUtils.onMessage;

// Launch all services assigned to this worker thread.
// Search in workers.json config file for assigned web services.
workerUtils.launchSvcAll();
