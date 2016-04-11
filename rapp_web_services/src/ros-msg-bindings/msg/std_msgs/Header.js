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

var time = require('./Time.js');

/**
 * Generaly used to communicate timestamped data in a particular
 * coordinate frame.
 *
 * @param {Number} seq - sequence ID: consecutively increasing ID
 * @param {Time} stamp
 * @param {String} frame_id - Frame this data is associated with
 *  0: no frame
 *  1: global frame
 */
function Header( seq, stamp ){
  this.seq = seq || 0;
  this.stamp = stamp || new time();
  this.frame_id = frame_id || '';
}

Header.prototype.equals = function( obj ){
  if ( ! (this.constructor == obj.constructor) ){
    throw new Error("Object is not of type Header");
  }
  var isEqual = ((obj.secs == this.secs) && (obj.nsecs == this.nsecs) &&
                  this.stamp.equals(obj.stamp));
  return isEqual;
}

module.exports = Header;
