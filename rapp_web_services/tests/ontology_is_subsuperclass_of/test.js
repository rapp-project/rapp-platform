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

var testParams = require( path.join(__dirname, 'params.json') );

exports.TEST = function (){
  import service ontology_is_subsuperclass_of();

  var args = {
    parent_class: testParams.request.args.parent_class,
    child_class: testParams.request.args.child_class,
    recursive: testParams.request.args.recursive,
  };
  var success = true;
  var response = ontology_is_subsuperclass_of(args).postSync();
  //console.log(response);

  var valid_results = testParams.response.results;

  for(var i in response.results){
    if( valid_results.indexOf(response.results[i]) > -1 ) {
      continue;
    }
    else{
      success = false;
      break;
    }
  }
  return {success: success, output: response, input: args};
};

