/*!
 * @file available_services.service.js
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


"use strict";


var __DEBUG__ = false;

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var hop = require('hop');
/*----------------------------------------------*/

/*-----<Defined Name of QR Node ROS service>----*/
var hopServiceName = 'available_services';
var hopServiceId = null;
/*------------------------------------------------------*/

/* -- Set timer values for websocket communication to rosbridge -- */
var timer_tick_value = 10; // ms
var max_time = 2000; // ms
var max_tries = 3;
//var max_timer_ticks = 1000 * max_time / tick_value;
/* --------------------------------------------------------------- */

var __workerId__ = null;
var __masterId__ = null;
var __updatedServiceList__ = true;
var __availableServices__ = [];

register_master_interface();


service available_services (  )
{
  postMessage( craft_slaveMaster_msg('log', 'client-request') );
  __updatedServiceList__ = false;
  postMessage( craft_slaveMaster_msg('up-services', '') );

  /* Async http response */
  /*----------------------------------------------------------------- */
  return hop.HTTPResponseAsync(
    function( sendResponse ) {
      var timer_ticks = 0;
      var elapsed_time = 0;
      var retries = 0;

      function asyncWrap(){
        setTimeout( function(){
          timer_ticks += 1;
          elapsed_time = timer_ticks * timer_tick_value;

          if (__updatedServiceList__)
          {
            var resp_msg = craft_response();

            var logMsg = 'Succesfully updated available service list.' +
              ' Returning to client with success';
            postMessage( craft_slaveMaster_msg('log', logMsg) );

            sendResponse(resp_msg);
            return;
          }
          else if (__updatedServiceList__ == false && elapsed_time > max_time)
          {
            retries += 1;
            timer_ticks = 0;

            if (retries >= max_tries)
            {
              var respMsg = craft_error_response();

              var logMsg = 'Reached max_retries while trying to retrieve' +
                'available services list from master. Returning to client' +
                'with error ---> [RAPP Platform Failure]';
              postMessage( craft_slaveMaster_msg('log', logMsg) );

              sendResponse(respMsg);
              return;
            }
          }
          asyncWrap();

        }, timer_tick_value);
      }
      asyncWrap();
    }, this);
}


function craft_response()
{
  var crafted_msg = {services: __availableServices__, error: ''};
  return JSON.stringify(crafted_msg);
}

function craft_error_respose()
{
  var crafted_msg = {services: [], error: 'RAPP Platform Failure'};
  return JSON.stringify(crafted_msg);
}

function register_master_interface()
{
  onexit = function(e){
    console.log("Service [%s] exiting...", hopServiceName);

    var logMsg = "Received termination command. Exiting.";
    postMessage( craft_slaveMaster_msg('log', logMsg) );
  }

  onmessage = function(msg){
    if (__DEBUG__)
    {
      console.log("Service [%s] received message from master process",
        hopServiceName);
      console.log("Msg -->", msg.data);
    };
    /* -------< Add to log file >---------- */
    var logMsg = 'Received message from master process --> [' +
      msg.data + ']';
    postMessage( craft_slaveMaster_msg('log', logMsg) );
    /* ------------------------------------ */

    exec_master_command(msg.data);
  }

  // On initialization inform master and append to log file
  var logMsg = "Initiated worker";
  postMessage( craft_slaveMaster_msg('log', logMsg) );
}


function exec_master_command(msg)
{
  var cmd = msg.cmdId;
  var data = msg.data;
  switch (cmd)
  {
    case 2055:  // Set worker ID
      __workerId__ = data;
      break;
    case 2050:  // Set master ID
      __masterId__ = data;
      break;
    case 2060:  // Master returns available hop services list
      __updatedServiceList__ = true;
      __availableServices__ = data;
      var logMsg = 'Received list of available-services from master';
      postMessage( craft_slaveMaster_msg('log', logMsg) );
    default:
      break;
  }
}


function craft_slaveMaster_msg(msgId, msg)
{
  var msg = {
    name: hopServiceName,
    id:   __workerId__,
    msgId: msgId,
    data: msg
  }
  return msg;
}

