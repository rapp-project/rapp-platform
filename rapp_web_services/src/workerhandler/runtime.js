/*!
 * @file runtime.js
 * @brief Timer functionalities.
 */

/**
 *  MIT License (MIT)
 *
 *  Copyright (c) <2014> <Rapp Project EU>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  Authors: Konstantinos Panayiotou
 *  Contact: klpanagi@gmail.com
 *
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
