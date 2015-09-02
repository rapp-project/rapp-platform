/*!
 * @file runtime.js
 * @brief High precision timer
 */

var startT = process.hrtime();


/*!
 * @brief Returns a number that represents the uprunning time in
 * milliseconds.
 * High Precision using process information.
 */
function getUpTime()
{
  var upTime = process.hrtime(startT);
  return upTime[0] * 1e3 + upTime[1] / 1e6;
}


/*!
 * @brief Returns a number that represents the number of milliseconds since
 * midnight January 1, 1970.
 */
function getTime()
{
  return new Date().getTime();
}


/*!
 * @brief Returns a String that represents current Date + time
 */
function getDate()
{
  var today = new Date();
  var dd = today.getDate();
  var mm = today.getMonth()+1; //January is 0!
  var yyyy = today.getFullYear();

  if(dd<10) {dd = '0' + dd;}

  if(mm<10) {mm = '0' + mm;}

  fullDate = mm + '-' + dd + '-' + yyyy;

  time = today.getTime();

  return fullDate + '-' + time;
}

module.exports = {
  getTime: getTime,
  getDate: getDate
}
