/***
 * Copyright 2016 RAPP
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


/***
 * @fileOverview
 *
 * [Face-Detection] RAPP Platform front-end web service.
 *
 *  @author Konstantinos Panayiotou
 *  @copyright Rapp Project EU 2016
 */


/*!
 * @brief Parse 'json' post field
 */
function JsonParser() {
}


/*!
 * @brief Call to parse input request object
 *
 * @param req {Object} the HOP Request object.
 *
 * @returns {Object}
 */
JsonParser.prototype.parse = function(req) {
  var jsonField = req.body.json || '';
  var data = this.json(jsonField);
  for (var k in data){
    req.body[k] = data[k];
  }
  // Delete the json post field from the request body.
  delete req.body.json;
};


/*!
 * @brief ToJson method.
 *
 * @param str {string} - Json string to parse.
 *
 * @returns {Object} - json-parsed object.
 */
JsonParser.prototype.json = function(str) {
  var data = {};
  try{
    // Parse json string
    data = JSON.parse(str);
  }
  catch(e){
    // Catch exception and return empty object.
    return {};
  }
  return data;
};


module.exports.json = function() {
  return new JsonParser();
};
