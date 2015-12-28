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


var _REQUIRE = [];


var _CLIENT_SCRIPTS = function( attrs ){
  attrs = attrs || {};
  var _user = attrs.user || '';
  import service rapp_user_info();
  import service cognitive_get_scores();
  import service cognitive_get_history();


  return ~{
    var connectedUser = ${_user}

    function appendOption(elemId, items){
      $.each(items, function (i, item) {
        $('#'+elemId).append($('<option>', {
          value: item.value,
          text : item.text
        }));
      });
    };

    function selectPickerStyle(elemId, _style){
      $('#'+elemId).selectpicker({
        style: _style
      }).selectpicker('refresh');
    }

    function formNewLine( text ){
      var str = '<p>' + text + "</p>";
      return str;
    }

    $(document).ready( function(){
      $('.selectpicker').selectpicker({
        style: 'btn-inverse',
        size: 'auto'
      });

      ${rapp_user_info}({user: connectedUser}).post( function(resp){
        var usrPanelBody = $('#user-info-panel').find('.panel-body');

        for( var prop in resp.user_info ){
          var txt = prop + ': ' + resp.user_info[prop];
          usrPanelBody.append( formNewLine(txt) );
        }
      });

      var args = {
        user: connectedUser,
        from_time: 0,
        to_time: 100000000000,
        test_type: ''
      };

      ${cognitive_get_history}(args).post( function(resp){
        var usrHistoryPanelBody = $('#user-history-panel').find('.panel-body');
        for( var ii = 0; ii < resp.records.length; ii++ ){
          var txt = JSON.stringify(resp.records[ii]);
          usrHistoryPanelBody.append( formNewLine(txt) );
        }
      })

    })
  }
}

exports.CSS = _CSS;
exports.JS = _JS;
exports.REQUIRE = _REQUIRE;
exports.CLIENT_SCRIPTS = _CLIENT_SCRIPTS;
