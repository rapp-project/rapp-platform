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
var GUIPARTS = require( path.join(__dirname, 'guiParts.js') );


var INDEX = function( attrs ){
  attrs = attrs || {};
  var _user = attrs.user || '';

  return <html lang="en">
    ${GUIPARTS.HEADER( {user: _user} )}
    <body>
      ${GUIPARTS.NAVBAR()}
      <div class="container">
        <div class="row-fluid">
          ${GUIPARTS.PAGE_HEADER()}
        </div>
      </div>
      <div class="container">
        <div class="row-fluid row-centered">
          ${GUIPARTS.BTN_GROUP()}
        </div>
      </div>
      <div class="container">
        <div class="row-fluid row-centered" style="margin-top:50px">
          <div class="col-xs-3 col-sm-3 col-md-3 col-lg-3" style="word-wrap:break-word;">
            <div class="show" id="user-info-panel-outer">
              ${GUIPARTS.USER_INFO_PANEL()}
            </div>
            <div class="show" id="user-history-panel-outer">
              ${GUIPARTS.USER_HISTORY_PANEL()}
            </div>
          </div>
          <div class="col-xs-9 col-sm-9 col-md-9 col-lg-9" style="word-wrap:break-word;">
            ${GUIPARTS.PLOT_GRAPH()}
          </div>
        </div>
      </div>
      ${GUIPARTS.FOOTER()}
    </body>
  </html>
}


exports.INDEX =  INDEX;
