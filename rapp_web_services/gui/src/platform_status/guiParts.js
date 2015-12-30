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

var quotes = require( path.join(__dirname, 'quotes.json') ).quotes;

function randomQuote(){
  var randQuote;

  var _quotes = quotes;

  var rnum = Math.floor(Math.random() * _quotes.length);
  randQuote = _quotes[rnum];
  return randQuote;
}

var NAVBAR = function( attrs ){
  var homeButton = <A>{
    class: "navbar-brand",
    href: "#",
    "RAPP Platform Status"
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


var HOP_SERVICES_FORM = function(){
  var attrs = {
    labelId: "hop-services-label",
    text: "Web Services",
    formId: "hop-services",
    value: "hop-services"
  };

  return guiCommons.FORM(attrs)
}


var ROS_SERVICES_FORM = function(){
  var attrs = {
    labelId: 'ros-services-label',
    text: "ROS-Services",
    formId: "ros-services",
  };

  return guiCommons.FORM(attrs)
}


var ROS_NODES_FORM = function(){
  var attrs = {
    labelId: 'ros-nodes-label',
    text: "ROS-Nodes",
    formId: "ros-nodes",
  };

  return guiCommons.FORM(attrs)
}


var ROS_TOPICS_FORM = function(){
  var attrs = {
    labelId: 'ros-topics-label',
    text: "ROS-Topics",
    formId: "ros-topics",
  };

  return guiCommons.FORM(attrs)
}


var HEADER = function(){
  import service active_ros_nodes();
  import service active_ros_topics();
  import service active_ros_services();
  import service available_services();

  var _title = 'RAPP Platform Status Web Page';
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


var TEST_PANEL = function(attrs){
  attrs = attrs || {};
  var txt = attrs.text || "Service invocation Panel";

  return  <DIV>{
        class: "panel panel-info text-center",
        <DIV>{
          class: "panel-heading",
          txt
        },
        <DIV>{
          class: "panel-body",
          <FORM>{
            class: "form-vertical",
            <DIV>{
              class: "form-group",
              <LABEL>{
                class: "",
                "Selected HOP Web Service:"
              },
              <P>{
                class: "form-control",
                id: "selected-hop-srv",
                "None"
              },
            },
            <DIV>{
              class: "btn-group inline",
            <BUTTON>{
              type: "button",
              class: "btn btn-default",
              id: 'btnExecTest',
              "Call Service"
            },
            <BUTTON>{
              type: "button",
              class: "btn btn-default",
              id: 'btnClearResults',
              "Clear Results"
            }
            }

          }
        }
      }
}


var TEST_RESULTS_PANEL = function(attrs){
  attrs = attrs || {};
  var txt = attrs.text || "HOP Web Service Response Panel";

  return  <DIV>{
      class: "panel panel-primary",
      id: "testResultsPanel",
      <DIV>{
        class: "panel-heading text-center",
        txt
      },
      <DIV>{
        class: "panel-body",
        <FORM>{
          class: "form-vertical",
          <DIV>{
            class: "form-group",
            style: "width=100%; height=100%; min-height:150px; max-height:300px; overflow:scroll",
            <LABEL>{
              class: "",
              "Service Input Parameters:"
            },
            <DIV>{
              class: "",
              id: "formRequest"
            }
          },
          <DIV>{
            class: "form-group",
            style: "width=100%; height=100%; min-height:150px; max-height:300px; overflow:scroll",
            <LABEL>{
              class: "",
              "Service Response:"
            },
            <DIV>{
              class: "",
              id: "formResponse"
            }
          }
        }
      }
    }
}


var ROW_SPACER = function( attrs ){
  attrs = attrs || {};
  return <DIV>{
    class: "row-spacer",
  }
}


var PAGE_HEADER = function( attrs ){
  attrs = attrs || {};
  var _title = attrs.title || "RAPP Platform Status";
  var _text = attrs.text || randomQuote();

  return  <DIV>{
      class: "jumbotron",
      align: "center",
      <H3>{
        "RAPP Platform Status"
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

function serviceUrl(srvName){
  return 'http://' + hop.hostname + ':' + hop.port + '/hop/' + srvName;
}


exports.NAVBAR = NAVBAR;
exports.HEADER = HEADER;
exports.PAGE_HEADER = PAGE_HEADER;
exports.ROS_SERVICES_FORM = ROS_SERVICES_FORM;
exports.ROS_TOPICS_FORM = ROS_TOPICS_FORM;
exports.ROS_NODES_FORM = ROS_NODES_FORM;
exports.HOP_SERVICES_FORM = HOP_SERVICES_FORM;
exports.TEST_PANEL = TEST_PANEL;
exports.TEST_RESULTS_PANEL = TEST_RESULTS_PANEL;
exports.FOOTER = FOOTER;

