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
var ROSBRIDGE_IP = 'localhost';
var ROSBRIDGE_PORT = '9090';

var ros = new ROS(ROSBRIDGE_IP, ROSBRIDGE_PORT);

var domTree = require( '../html/rapp_platform_ros_conditions.html' );
var platformActiveServices = [];
var platformActiveNodes = [];
var platformActiveTopics = [];
var rappRosServices  = [];
var pollCycle = 1000; // 10 sec


loopInf();

var navbar =
    '<div class="navbar navbar-inverse navbar-fixed-top">' +
      '<div class="navbar-inner">' +
        '<div class="container">' +
          '<button type="button" class="btn btn-navbar" data-toggle="collapse" data-target=".nav-collapse">' +
            '<a class="btn btn-success" href="#">RAPP Platform Status</a>' +
          '</button>' +
          //'<a class="brand" href="#">Project name</a>' +
        '</div>' +
      '</div>' +
    '</div>';

service rapp_platform_status(  )
{
  var srv_class = (platformActiveServices.length > 1) ? "bg-success" : "bg-danger";
  var nodes_class = (platformActiveNodes.length > 1) ? "bg-success" : "bg-danger";
  var topics_class = (platformActiveTopics.length > 1) ? "bg-success" : "bg-danger";

  var rapp_srv_literal = '';
  var services_literal = '';
  var nodes_literal = '';
  var topics_literal = '';
  var nodes_label =
    '<h2 align="center"><p class=' + nodes_class +
    ' align="center">ROS Nodes</p></h2>';
  var rapp_services_label =
    '<h2 align="center"><p class=' + srv_class +
    ' align="center">ROS Services</p></h2>';
  var topics_label =
    '<h2 align="center"><p class=' + srv_class +
    ' align="center">ROS Topics</p></h2>';

  for(index = 0 ; index < platformActiveServices.length ; index++)
  {
    services_literal += "<li>" + platformActiveServices[index] + '</li>';
  }

  for(index = 0 ; index < rappRosServices.length ; index++)
  {
    rapp_srv_literal += "<option>" +   rappRosServices[index] + '</option>';
  }

  for(index = 0 ; index < platformActiveNodes.length ; index++)
  {
    nodes_literal += "<option>" + platformActiveNodes[index] + '</option>';
  }

  for(index = 0 ; index < platformActiveNodes.length ; index++)
  {
    topics_literal += "<option>" + platformActiveTopics[index] + '</option>';
  }


  return <html lang="en">
    <head>
      <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.4/css/bootstrap.min.css">
      <meta charset="utf-8"/>
      <meta name="viewpoint" content="width=device-width, initial-scale=1">
      <style>
        body {
          padding-top: 60px; /* 60px to make the container go all the way to the bottom of the topbar */
        }
      </style>
    </head>
    <body>
      ${navbar}
      <div class='jumbotron' align='center'>
        <h1>RAPP Platform information</h1>
        <div>
          To work, roscore and rosbridge_websocket must be executed
        </div>
      </div>
      <div class='container'>
        <div class='row-fluid'>
          <div class='col-md-4' style='word-wrap:break-word;'>
            ${rapp_services_label}
            <select id='SrvBox' class="form-control" onchange=~{
              var selected = this.options[this.selectedIndex].value
                alert(selected)
              }>
              ${rapp_srv_literal}
            </select>
          </div>
          <div class='col-md-4' style='word-wrap:break-word;'>
            ${nodes_label}
            <select id='NodesBox' class="form-control" onchange=~{
              var selected = this.options[this.selectedIndex].value
                alert(selected)
              }>
              ${nodes_literal}
            </select>
          </div>
          <div class='col-md-4' style='word-wrap:break-word;'>
            ${topics_label}
            <select id='NodesBox' class="form-control" onchange=~{
              var selected = this.options[this.selectedIndex].value
                alert(selected)
              }>
              ${topics_literal}
            </select>
          </div>
        </div>
      </div>
      <div style="position: absolute; bottom: 5px;">
        <address>
          <strong>The RAPP-Project</strong><br>
          <a href="rapp-project.eu">rapp-project.eu</a>
        </address>
      </div>
    </body>
  </html>;
}

function loopInf(){
  setTimeout(function(){
    if(!ros.connected() ){ros.connect(ROSBRIDGE_IP, ROSBRIDGE_PORT)}
    else{
      ros.getServices(function(data){
        platformActiveServices = data;
        //console.log(data)
        rappRosServices = [];
        for(i in platformActiveServices){
          if(platformActiveServices[i].indexOf("rapp") > -1){
            rappRosServices.push(platformActiveServices[i])
          }
        }
      })
      ros.getNodes(function(data){
        platformActiveNodes = data;
      })
      ros.getTopics(function(data){
        platformActiveTopics = data;
      })
    }
    loopInf();
  }, pollCycle)
}


