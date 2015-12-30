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


/***
 * @fileOverview
 *
 * [Rapp-Platform-Status] RAPP Platform front-end web service. HTML response.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2015
 */


var hop = require('hop');
var path = require('path');

var ENV = require( path.join(__dirname, '..', 'env.js') );

var __includeDir = path.join(__dirname, '..', 'modules');
var __configDir = path.join(__dirname, '..', 'config');

var Fs = require( path.join(__includeDir, 'common', 'fileUtils.js') );

var ROS = require( path.join(__includeDir, 'RosBridgeJS', 'src',
    'Rosbridge.js') );

var VIEW = require( path.join(__dirname, '..', 'gui', 'src', 'cognitive_system',
    'view.js') );


// Initiate communication with rosbridge-websocket-server
var ros = new ROS({hostname: ENV.ROSBRIDGE.HOSTNAME, port: ENV.ROSBRIDGE.PORT,
  reconnect: true, onconnection: function(){
    // .
  }
});


service cognitive_system_plot_data( {history_data: {}} )
{
  /***
   * Asynchronous http response.
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      var data = [];
      /***
       * Sort history records by timestamp
       */
      for ( var prop in history_data ){
        history_data[prop].sort( function(obj1, obj2){
          return obj1.timestamp - obj2.timestamp;
        });

        var attrs = {
          mode: 'lines+markers',
         name: prop,
         marker: { size: 2 },
         line: { width: 1 },
        };

        var trace = createTraceData(history_data[prop], attrs);
        data.push(trace);

        console.log(trace)
      }

      var layout = {
        title: 'Cognitive Exercises Activity History'
      };


      sendResponse( hop.HTTPResponseJson( { data: data, layout: layout } ) );
    }, this);
}


function createTraceData(dataArray, attrs){
  var _mode = attrs.mode || 'lines+markers';
  var _name = attrs.name || '';
  var _marker = attrs.marker || { size: 2 };
  var _line = attrs.line || { width: 1 };

  var trace = {
    x: [],
    y: [],
    mode: _mode,
    name: _name,
    marker: _marker,
    line: _line
  };

  if( ! (dataArray instanceof Array) ){
    return trace;
  }

  for( var ii = 0; ii < dataArray.length; ii++ ){
    trace.x.push( dataArray[ii].timestamp );
    trace.y.push( dataArray[ii].score );
  }

  return trace;
}

/****************************************************************************/

function service_url(srvName){
  return 'http://' + hop.hostname + ':' + hop.port + '/hop/' + srvName;
}

