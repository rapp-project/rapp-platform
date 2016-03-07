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
var util = require('util');
var sys = require('sys');
var spawn = require('child_process').spawn;
var exec = require('child_process').exec;

var serverCfg = require(path.join(__dirname, 'env.js')).SERVER;

var child;

var command = 'hop';
command += " --http-port " + serverCfg.run.port;
command = (serverCfg.run.clear_cache) ? (command + " --clear-cache ") : command;
command = (serverCfg.run.https.enable) ? (command + " --https " +
  " --https-pkey " + serverCfg.run.https.pkey_filepath +
  " --https-cert " + serverCfg.run.https.cert_filepath) : (command + " --no-https");
command += " --rc-file " + serverCfg.paths.hoprc;
command += " --cache-dir " + serverCfg.paths.cache_dir;
command += " --log-file " + serverCfg.paths.log_file;
command += " --scheduler " + serverCfg.run.scheduler;
command += "  --max-threads " + serverCfg.run.maxthreads;
command += " -v" + serverCfg.logging.verbosity;
command += " -w" + serverCfg.logging.warning;
command += " -g" + serverCfg.logging.debug;
command += " -s" + serverCfg.logging.security;
command = (serverCfg.logging.time) ? (command + " --time ") : command;
command = (! serverCfg.logging.color) ? (command + " --no-color ") : command;
for (var i in serverCfg.launch_files){
  command += serverCfg.launch_files[i] + " ";
}

console.log(command)

module.exports = command;
