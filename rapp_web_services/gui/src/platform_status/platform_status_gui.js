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

  attrs = attrs || {};
  var _title = 'RAPP Platform Status Web Page';
  var _css = imports.CSS;
  var _js = imports.JS;
  var _require = imports.REQUIRE;
  var _meta = guiCommons.META;
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
  var text = attrs.text || "";
  var _class = attrs.class || "";

  return <DIV>{
    class: "container",
    <DIV>{
      class: "jumbotron",
      align: "center",
      <H3>{
        "RAPP Platform Status"
      },
      <SMALL>{
        <EM>{
          class: _class,
          text
        }
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
        },
        <P>{
          class: "",
          <A>{
            href: "mailto:klpanagi@gmail.com",
            "Konstantinos Panayiotou."
          }
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

