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
var clientReq = require( '../common/clientRequire.js' );



var navBar = function( attrs ){
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


var form = function( attrs ){
  var labelId = (attrs.labelId || "").toString();
  var txt = (attrs.text || "").toString();
  var formId = (attrs.formId || "").toString();
  var _class = (attrs.class || "row-fluid").toString();
  var align = (attrs.align || "center").toString();
  var _style = (attrs.style || '');


  return <DIV>{
    class: _class,
    <LABEL>{
      class: "control-label",
      style: "display:inline",
      txt
    },
    <DIV>{
      class: "row-fluid",
      <SELECT>{
        class: "selectpicker",
        id: formId,
        style: _style,
        <OPTION>{
          value: "",
          txt
        }
      }
    }
  }
}


var header = function( attrs ){
  import service active_ros_nodes();
  import service active_ros_topics();
  import service active_ros_services();
  import service available_services();


  return <HEAD>{
    title: "Rapp Platform Status Web Page",
    css: clientReq.CSS,
    jscript: clientReq.JS,
    require: [],
    <META>{
      name: 'viewpoint',
      content: "width=device-width, initial-scale=1",
      charset: "utf-8"
    },
    ~{
      var selections = {};

      function windowOnload(){
        var hopsrvsel = document.getElementById('hop-services');
        hopsrvsel.onchange = function(){
          var selhopSrv = document.getElementById('selected-hop-srv');
          selhopSrv.innerHTML = hopsrvsel.value;
        }
      }

      window.onload = windowOnload;

      function appendOption(elemId, txt){
        var sel = document.getElementById(elemId);
        var option = document.createElement("option");
        option.text = txt;
        sel.appendChild(option);
      };

      ${active_ros_nodes}().post(function(resp){
        var rosNodes = resp.ros_nodes;
        for(var i = 0; i < rosNodes.length; i++){
          appendOption('ros-nodes', rosNodes[i]);
        }
        var sel = document.getElementById('ros-nodes');
        var style = (rosNodes.length > 1) ? "btn-success" : "btn-danger";
        sel.setAttribute('data-style', style)
      })

      ${active_ros_topics}().post(function(resp){
        var rosTopics = resp.ros_topics;
        for(var i = 0; i < rosTopics.length; i++){
          appendOption('ros-topics', rosTopics[i]);
        }
        var sel = document.getElementById('ros-topics');
        var style = (rosTopics.length > 1) ? "btn-success" : "btn-danger";
        sel.setAttribute('data-style', style)
      })

      ${active_ros_services}().post(function(resp){
        var rosSrvs= resp.ros_srvs;
        for(var i = 0; i < rosSrvs.length; i++){
          appendOption('ros-services', rosSrvs[i]);
        }
        var sel = document.getElementById('ros-services');
        var style = (rosSrvs.length > 1) ? "btn-success" : "btn-danger";
        sel.setAttribute('data-style', style)
      })

      ${available_services}().post( function(response){
        var hopSrvs = response.services || [];
        for(var i = 0; i < hopSrvs.length; i++){
          appendOption('hop-services', hopSrvs[i]);
        }
        var sel = document.getElementById('hop-services');
        var style = (hopSrvs.length > 1) ? "btn-success" : "btn-danger";
        sel.setAttribute('data-style', style)
      } )

    }
  }
}


var testPanel = function(attrs){
  attrs = (attrs || {});
  var gridSize = (attrs.gridSize || 3).toString();
  var txt = "HOP Web Service invocation Panel";
  var _class = (attrs.class || "row-fluid").toString();
  var gridClass = "col-xs-" + gridSize + " col-sm-" + gridSize + " col-md-" +
    gridSize;

  return <DIV>{
    class: "row-fluid text-center",
    <DIV>{
      class: gridClass,
      style: "",
      <DIV>{
        class: "panel panel-info",
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
            <BUTTON>{
              type: "button",
              class: "btn btn-default",
              onclick: ~{
                alert("Currently Unavailable")
              },
              "Call Service"
            }
          }
        }
      }
    }
  }
}


var testResultsPanel = function(attrs){
  attrs = attrs || {};
  var gridSize = (attrs.gridSize || 6).toString();
  var txt = "HOP Web Service Response Panel";
  var _class = (attrs.class || "row-fluid").toString();
  var gridClass = "col-xs-" + gridSize + " col-sm-" + gridSize + " col-md-" +
    gridSize;

  return <DIV>{
    class: "row-fluid text-center",
    <DIV>{
      class: gridClass,
      style: "",
      <DIV>{
        class: "panel panel-primary",
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
                "Service Input Parameters:"
              },
              <P>{
                class: "form-control",
                id: "selected-hop-srv",
                ""
              }
            },
            <DIV>{
              class: "form-group",
              <LABEL>{
                class: "",
                "Service Response:"
              },
              <P>{
                class: "form-control",
                id: "selected-hop-srv",
                ""
              }
            }
          }
        }
      }
    }
  }
}


var rowSpacer = function(attrs){
  attrs = attrs || {};
  return <DIV>{
    class: "row-spacer",
  }
}


var pageHeader = function(attrs){
  attrs = attrs || {};
  return <DIV>{
    class: "container",
    <DIV>{
      class: "jumbotron",
      align: "center",
      <H2>{
        "RAPP Platform Status",
      }
    }
  }
}

var footer = function(attrs){
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


exports.NAVBAR = navBar;
exports.HEADER = header;
exports.FORM = form;
exports.TEST_PANEL = testPanel;
exports.PAGE_HEADER = pageHeader;
exports.TEST_RESULTS_PANEL = testResultsPanel;
exports.FOOTER = footer;

