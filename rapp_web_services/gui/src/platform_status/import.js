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

var filePath = path.dirname( module.filename );


function resolve( ){
  var args = arguments;
  var regexp = /~/g;
  var home = process.env.HOME;
  var joinedPaths = "";

  for( var i in args ){
    joinedPaths = path.join(joinedPaths, args[i]);
  }

  return joinedPaths.replace(regexp, home);
}


var _JS = [
  resolve(filePath, '../../js/jquery-1.11.3.js'),
  resolve(filePath, '../../js/jquery-ui-1.11.4/jquery-ui.min.js'),
  resolve(filePath, '../../js/bootstrap.min.js'),
  resolve(filePath, '../../js/bootstrap-select.js')
];


var _CSS = [
  resolve(filePath, '../../css/bootstrap.min.css'),
  resolve(filePath, '../../css/bootstrap-theme.min.css'),
  resolve(filePath, '../../js/jquery-ui-1.11.4/jquery-ui.css'),
  resolve(filePath, '../../css/bootstrap-select.css'),
  resolve(filePath, '../../css/klpanagi.css')
];


var _REQUIRE = [
];


var _CLIENT_SCRIPTS = function(){
  import service active_ros_nodes();
  import service active_ros_topics();
  import service active_ros_services();
  import service available_services();

  return ~{
    function appendOption(elemId, items){
      $.each(items, function (i, item) {
        $('#'+elemId).append($('<option>', {
          value: item.value,
          text : item.text
        }));
      });
      $('#'+elemId).selectpicker('refresh');
    };

    function selectPickerStyle(elemId, _style){
      $('#'+elemId).selectpicker({
        style: _style
      }).selectpicker('refresh');
    }


    $(document).ready(function() {
      $('.selectpicker').selectpicker({
        style: 'btn-inverse',
        size: 'auto'
      });

      var hopsrvsel = document.getElementById('hop-services');
      hopsrvsel.onchange = function(){
        var selhopSrv = document.getElementById('selected-hop-srv');
        selhopSrv.innerHTML = hopsrvsel.value;
      };

      ${active_ros_nodes}().post(function(resp){
        var rosNodes = resp.ros_nodes;
        for(var i = 0; i < rosNodes.length; i++){
          appendOption('ros-nodes', [{text:rosNodes[i], value:rosNodes[i]}]);
        }
        var style = (rosNodes.length > 1) ? "btn-success" : "btn-danger";
        selectPickerStyle('ros-nodes', style);
      });

      ${active_ros_topics}().post(function(resp){
        var rosTopics = resp.ros_topics;
        for(var i = 0; i < rosTopics.length; i++){
          appendOption('ros-topics', [{text:rosTopics[i], value:rosTopics[i]}]);
        }
        var style = (rosTopics.length > 1) ? "btn-success" : "btn-danger";
        selectPickerStyle('ros-topics', style);
      });

      ${active_ros_services}().post(function(resp){
        var rosSrvs= resp.ros_srvs;
        for(var i = 0; i < rosSrvs.length; i++){
          appendOption('ros-services', [{text:rosSrvs[i], value:rosSrvs[i]}]);
        }
        var style = (rosSrvs.length > 1) ? "btn-success" : "btn-danger";
        selectPickerStyle('ros-services', style);
      });

      ${available_services}().post( function(response){
        var hopSrvs = response.services || [];
        for(var i = 0; i < hopSrvs.length; i++){
          appendOption('hop-services', [{text:hopSrvs[i], value:hopSrvs[i]}]);
        }
        var style = (hopSrvs.length > 1) ? "btn-success" : "btn-danger";
        selectPickerStyle('hop-services', style);
      });
    });
  }
}

exports.CSS = _CSS;
exports.JS = _JS;
exports.REQUIRE = _REQUIRE;
exports.CLIENT_SCRIPTS = _CLIENT_SCRIPTS();
