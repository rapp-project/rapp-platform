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
var GUIPARTS = require( path.join(__dirname, 'guiParts.js') );


var INDEX = function(){
 return   <HTML>{
    lang: "en",
    GUIPARTS.HEADER(),
    <BODY>{
      GUIPARTS.NAVBAR(),
      <DIV>{
        class: "container",
        <DIV>{
          class: "row-fluid",
          GUIPARTS.PAGE_HEADER()
        }
      },
      <DIV>{
        class: "container",
        <DIV>{
          class: "row-fluid",
        }
      },
      <DIV>{
        class: "container",
        <DIV>{
          class: "row-fluid",
          GUIPARTS.FOOTER()
        }
      }
    }
  }
}

exports.INDEX =  INDEX;
