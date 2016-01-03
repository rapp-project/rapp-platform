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

var cssDir = path.join('..', '..', 'css');
var jsDir = path.join('..', '..', 'js');
var imports = require( path.join(__dirname, 'import.js') );
var guiCommons = require( path.join(__dirname, '../common', 'guiCommons.js') );


var NAVBAR = function( attrs ){
  attrs = attrs || {};

  var _logoSrc = attrs.logoSrc ||
    path.join(__dirname, '..', '..', 'img', 'RAPP_logo.png');

  var homeButton = <A>{
    class: "navbar-toggle",
    href: "#",
    "RAPP Cognitive System"
  };


  return <div class="navbar-custom navbar-inverse navbar-fixed-top" role="navigation">
    <div class="container">
      <div class="navbar-header">
        <a class="navbar-brand" rel="home" href="http://rapp-project.eu" title="RAPP logo">
          <img src=${_logoSrc} alt="RAPP logo" align="middle" style="margin-top:-10px" height="45">
        </a>
        ${homeButton}
      </div>
      <div class="collapse navbar-collapse navbar-menubuilder">
        <ul class="nav navbar-nav navbar-left">
          <li class="active"><a href="#">Home</a></li>
        </ul>
        <ul class="nav navbar-nav navbar-right">
          <li class=""><a href="http://rapp-project.eu/community/contact/">Contact Us</a></li>
        </ul>
      </div>
    </div>
  </div>
}


var HEADER = function( attrs ){
  attrs = attrs || {};

  var _title = attrs.title || 'RAPP Platform Cognitive Exercises';
  var _css = imports.CSS;
  var _js = imports.JS;
  var _require = imports.REQUIRE;
  var connectedUser = attrs.user || '';
  var _meta = guiCommons.METATAG();
  var _plain_scripts = imports.CLIENT_SCRIPTS( {user: connectedUser} );


  return guiCommons.HEADER({
    title: _title,
    css: _css,
    js: _js,
    require: _require,
    meta: _meta,
    plain_scripts: _plain_scripts
  })
}


var PAGE_HEADER = function( attrs ){
  attrs = attrs || {};
  var _title = attrs.title || "RAPP Cognitive System";
  var _text = attrs.text || "";

  return  <DIV>{
      class: "jumbotron",
      align: "center",
      <H3>{
        _title
      },
      <SMALL>{
        <EM>{
          class: "",
          _text
        }
      }
    }
}


var FOOTER = function( attrs ){
  attrs = attrs || {};

  return <DIV>{
    class: "footer navbar navbar-inverse navbar-fixed-bottom",
    <DIV>{
      class: "container",
      <DIV>{
        class: "navbar-text pull-left",
        <P>{
          class: "",
          "Copyright: RAPP EU 2015."
        }
      }
    }
  }
}


var USER_INFO_PANEL = function( attrs ){
  attrs = attrs || {};
  var _title = attrs.title || "RAPP User";

  return <div class="panel panel-primary" id="user-info-panel">
    <div class="panel-heading">
      <h3 class="panel-title">${_title}</h3>
    </div>
    <div class="panel-body" style="min-height: 10; max-height: 10;"></div>
  </div>
}


var USER_HISTORY_PANEL = function( attrs ){
  attrs = attrs || {};
  var _title = attrs.title || "User's History";

  return <div class="panel panel-primary" id="user-history-panel">
    <div class="panel-heading">
      <h3 class="panel-title">${_title}</h3>
    </div>
    <div class="panel-body" style="width=100%; height=100%; min-height:150px; max-height:300px; overflow:scroll"></div>
  </div>

}


var BTN_GROUP = function( attrs ){
  attrs = attrs || {};
  var _title = attrs.title || '';

  return  <div class="btn-group">
      <button type="button" class="btn btn-primary" id='btnUserInfo'>Rapp User</button>
      <button type="button" class="btn btn-primary" id="btnUserHistory">User's Activity</button>
      <button type="button" class="btn btn-primary" id="btnPlot">Plots</button>
    </div>
}

var PLOT_GRAPH = function( attrs ){
  attrs = attrs || {};
  var _divId = attrs.div_id || 'historyPlotDiv';
  var _style = attrs.style || "width: 100%; height: 100%";

  return <div id=${_divId} style=${_style}></div>
}


exports.NAVBAR = NAVBAR;
exports.HEADER = HEADER;
exports.PAGE_HEADER = PAGE_HEADER;
exports.USER_INFO_PANEL = USER_INFO_PANEL;
exports.USER_HISTORY_PANEL = USER_HISTORY_PANEL;
exports.BTN_GROUP = BTN_GROUP;
exports.FOOTER = FOOTER;
exports.PLOT_GRAPH = PLOT_GRAPH;
