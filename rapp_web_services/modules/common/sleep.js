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


/**
 * @module
 * @description Sleep timer (Synchronous).
 *
 * @author Konstantinos Panayiotou
 * @copyright Rapp Project EU 2015
 *
 */


/**
 * @description Sleep execution process for given time in milliseconds.
 * Be aware that while in sleep mode, events do not arrive until the sleep
 * execution ends. For more information, read on Node.js Event Loop:
 *    https://nodesource.com/blog/understanding-the-nodejs-event-loop
 *
 * @param {Number} ms - Time to sleep the process, in milliseconds.
 * @returns {undefined}
 */
function sleepMS( ms )
{
  var start = new Date().getTime();
  for (var i = 0; i < 1e7; i++) {
    if ((new Date().getTime() - start) > ms){
      break;
    }
  }
}


module.exports = {
  sleepMS: sleepMS
};
