/*!
 * @file runtime.js
 * @brief High precision timer
 */

var startT = process.hrtime();

function get_time()
{
  var upTime = process.hrtime(startT);
  return upTime[0] * 1e9 + upTime[1];
}

module.exports = {
  get_time: get_time
}
