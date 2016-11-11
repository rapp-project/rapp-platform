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

const cfg = require(path.join(__dirname, 'server.json'));

const home = process.env.HOME;

const hopDir = resolvePath(cfg.paths.hop_dir);
const logDir = resolvePath(cfg.paths.log_dir);

const serverCfg = {
  mode: cfg.mode,
  launch_files: [
    path.join(__dirname, "../..", "init.js")
  ],
  paths: {
    hoprc: (cfg.paths.hoprc || path.join(__dirname, "../hoprc/", "hoprc.hop")),
    hop_dir: hopDir,
    log_dir: path.join(hopDir, "log/server"),
    log_file: path.join(logDir, util.format("hop-server-%s.log",
        getUnixTime())),
    cache_dir: path.join(hopDir, "cache/server")
  },
  run: cfg.run,
  logging: cfg.logging,
  startTimestamp: getUnixTime()
};


function getUnixTime(){
 return (new Date().getTime() / 1000).toString();
}


function resolvePath( _path )
{
  var regexp = /~/g;
  var newPath = '';
  if ( _path.match( regexp ) )
  {
    /*<Replaces "~" with "/home/user">*/
    newPath = _path.replace( regexp, home );
  }
  else
  {
    newPath = path.resolve( _path );
  }
  return newPath;
}

module.exports = serverCfg;
