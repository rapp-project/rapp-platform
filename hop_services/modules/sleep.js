/*!
 * @file sleep.js
 * @brief Sleep functions used to sleep current thread for a given time,
 * in milliseconds.
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


/**
 * This module exports.
 */
module.exports = {
  sleepMS: sleepMS
}
