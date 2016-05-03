/***
 * Copyright 2016 RAPP
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


/***
 * @fileOverview
 *
 * RAPP Authentication prototype implementation
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2016
 */



/*!
 * @brief Rapp Authentication prototype implementation.
 */
function RappAuth(ros) {
  this.rosSvcName = '/rapp/rapp_application_authentication/authenticate_token';
  this.ros = ros;

  this.getToken = function(req_) {
    return req_.header['accept-token'];
  };
}


/*!
 * @brief Call the authentication ROS node.
 *
 * @param req The request object.
 * @param onSuccess {Function} The on-authentication-success callback.
 * @param onFailed {Function} The on-authentication-failure callback.
 */
RappAuth.prototype.call= function(req_, resp_, onSuccess_, onFailed_) {
  var that = this;
  var token = this.getToken(req_);

  function callback(data){
    var success = data.success;
    var error = data.error;
    var username = data.username;
    if (error){
      onFailed_(error);
    }
    else{
      onSuccess_(username);
    }
  }

  function onerror(e){
    onFailed_(e);
  }

  // The authentication Service ROSMsg.
  var rosMsg = {
    token: this.getToken(req_)
  };

  this.ros.callService(this.rosSvcName, rosMsg, {
    success: callback,
    fail: onerror});
};


module.exports.RappAuth = RappAuth;
