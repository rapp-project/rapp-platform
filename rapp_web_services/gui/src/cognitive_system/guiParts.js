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

  var homeButton = <A>{
    class: "navbar-brand",
    href: "#",
    "RAPP Cognitive System"
  };

  return <NAV>{
    class: "navbar-custom navbar-inverse  navbar-fixed-top ",
    role: "navigation",
    <DIV>{
      class: "container",
      <DIV>{
        class: "navbar-header",
        <A>{
          class: "navbar-brand",
          rel: "home",
          href: "http://rapp-project.eu/",
          title: "RAPP logo",
          <IMG>{
            src: path.join(__dirname, '..', '..', 'img', 'RAPP_logo.png'),
            height: "45",
            style: "margin-top: -10px",
            alt: "RAPP Logo"
          }
        },
        homeButton
      },
      <DIV>{
        class: "collapse navbar-collapse navbar-menubuilder",
        <UL>{
          class: "nav navbar-nav navbar-right",
          <LI>{
            <A>{
              rel: "",
              href: "http://rapp-project.eu/community/contact/",
              "Contact Us"
            }
          },

        }
      }
    }
  }
}


var HEADER = function( attrs ){
  attrs = attrs || {};

  var _title = attrs.title || 'RAPP Platform Cognitive Exercises';
  var _css = imports.CSS;
  var _js = imports.JS;
  var _require = imports.REQUIRE;
  var _meta = <META>{
    name: 'viewpoint',
    content: 'width=device-width; initial-scale=1',
    charset: 'utf-8'
  };
  var _plain_scripts = imports.CLIENT_SCRIPTS;

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


var USERS_PANEL = function ( attrs ){
  attrs = attrs || {};
  var _title = attrs.title || "RAPP Users";

  return <div class="panel panel-primary" id="users-panel">
    <div class="panel-heading">${_title}</div>
    <div class="panel-body" style="min-height: 10; max-height: 10;"></div>
    ${guiCommons.SELECT_LIST()}
  </div>
}


function serviceUrl(srvName){
  return 'http://' + hop.hostname + ':' + hop.port + '/hop/' + srvName;
}


exports.NAVBAR = NAVBAR;
exports.HEADER = HEADER;
exports.PAGE_HEADER = PAGE_HEADER;
exports.USERS_PANEL = USERS_PANEL;
exports.FOOTER = FOOTER;

