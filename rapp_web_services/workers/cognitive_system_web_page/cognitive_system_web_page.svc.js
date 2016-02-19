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

var INCLUDE_DIR = path.join(__dirname, '..', 'modules');
var CONFIG_DIR = path.join(__dirname, '..', 'config');


var VIEWS = require( path.join(__dirname, '..', 'gui', 'src', 'cognitive_system',
    'views.js') );


service cognitive_system( {view: 'index'} )
{
  // Static for 'rapp' user. Change when Authentication is well known!
  var connectedUser = 'rapp';
  return VIEWS.INDEX({user: connectedUser});
}


service plot_data_form( {data: {}, x_axis:'', y_axis:'',
  sort: true, title: ''} )
{
  /* -- String to boolean translation -- */
  if ( typeof sort !== "boolean" ) {
    if ( sort === 'True' || sort === 'true' ) { sort = true; }
    else { sort = false; }
  }

  /***
   * Asynchronous http response.
   */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {

      var dataset = [];
      for ( var prop in data ){
        if ( sort ){
          // Pass object by reference
          sortByProperty(data[prop], x_axis);
        }

        var attrs = {
          mode: 'lines+markers',
         name: prop,
         marker: { size: 2 },
         line: { width: 1 },
        };

        var trace = createTraceData(data[prop], x_axis, y_axis, attrs);
        dataset.push(trace);
      }

      var layout = {
        title: title
      };

      var respObject = {
        dataset: dataset,
        layout: layout
      };
      sendResponse( hop.HTTPResponseJson( respObject ) );
    }, this);
}


/**
 * @brief Sort data by key identifier.
 *
 * @param data {Object} - The dataset object literal
 * @param key {String} - Dataset property key name used for sorting.
 *
 * @returns {Object} sortedDataObject.
 */
function sortByProperty(data, key){
  data.sort( function(obj1, obj2){
    return obj1[key] - obj2[key];
  });
}


function createTraceData(dataArray, x_axis, y_axis, attrs){
  attrs = attrs || {};
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

  if( ( !(dataArray instanceof Array) ) || ( !(x_axis && y_axis) ) ){
    return trace;
  }

  for( var ii = 0; ii < dataArray.length; ii++ ){
    trace.x.push( dataArray[ii][x_axis] );
    trace.y.push( dataArray[ii][y_axis] );
  }

  return trace;
}

