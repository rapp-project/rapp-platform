/*!
 * @file rapp_platform_status.service.js
 * @brief Returns available hop services located on RAPP Platform
 */


/**
 *  MIT License (MIT)
 *
 *  Copyright (c) <2014> <Rapp Project EU>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  Authors: Konstantinos Panayiotou
 *  Contact: klpanagi@gmail.com
 *
 */


/*--------------Load required modules-----------*/
var hop = require('hop');
var module_path = '../modules/'
var ROS = require( module_path + 'RosBridgeJS/src/Rosbridge.js' );
/*----------------------------------------------*/
var ros = new ROS('localhost','9090');

var domTree = require( '../html/rapp_platform_ros_conditions.html' );
var platformActiveServices = [];
var platformActiveNodes = [];
var rappRosServices  = [];
var pollCycle = 1000; // 10 sec


loopInf();

service platform_status(  )
{
  var services_literal =
    '<h2 align="center"><span class="label label-default">ROS Services</span></h2><br/><ul>';
  var rapp_srv_literal =
    '<h2 align="center"><span class="label label-default">RAPP ROS Services</span></h2><br/><ul>';
  var nodes_literal =
    '<h2 align="center"><span class="label label-default">ROS Nodes</span></h2><br/><ul>';

  for(index = 0 ; index < platformActiveServices.length ; index++)
  {
    services_literal += "<li>" + platformActiveServices[index] + '</li>';
  }
  services_literal += '</ul>';

  for(index = 0 ; index < rappRosServices.length ; index++)
  {
    rapp_srv_literal += "<li>" + rappRosServices[index] + '</li>';
  }
  rapp_srv_literal += '</ul>';

  for(index = 0 ; index < platformActiveNodes.length ; index++)
  {
    nodes_literal += "<li>" + platformActiveNodes[index] + '</li>';
  }
  nodes_literal += '</ul>';


  return <html>
    <head>
      <meta charset="utf-8"/>
      <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.4/css/bootstrap.min.css">
    </head>
    <body>
      <div class='jumbotron' align='center'>
        <h1>RAPP Platform information</h1>
        <div>
          To work, roscore and rosbridge_websocket must be executed
        </div>
      </div>
      <div class='container'>
        <div class='row-fluid'>
          <div class='col-md-4' style='word-wrap:break-word;'>
            <div id="nodes_div" class='panel panel-default'> ${nodes_literal} </div>
          </div>
          <div class='col-md-4' style='word-wrap:break-word;'>
            <div id="services_div" class="panel panel-default"> ${services_literal} </div>
          </div>
          <div class='col-md-4' style='word-wrap:break-word;'>
            <div id="rapp_services_div" class="panel panel-default"> ${rapp_srv_literal} </div>
          </div>
        </div>
      </div>

    </body>
  </html>;
}

function loopInf(){
  setTimeout(function(){
    ros.getServices(function(data){
      platformActiveServices = data;
      for(i in platformActiveServices){
        if(platformActiveServices[i].indexOf("rapp") > -1){
          rappRosServices.push(platformActiveServices[i])
        }
      }
    })
    ros.getNodes(function(data){
      platformActiveNodes = data;
    })

    loopInf();
  }, pollCycle)
}


