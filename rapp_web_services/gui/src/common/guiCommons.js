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

var META = function( attrs ){
  attrs = attrs || {};
  var _name = attrs.name || 'viewpoint';
  var _content = attrs.content || 'width=device-width, initial-scale=1';
  var _charset = attrs.charset || 'utf-8';

  return <META>{
    name: _name,
    content: _content,
    charset: _charset
  }
};


var HEADER = function( attrs ){
  attrs = attrs || {};
  var _title = attrs.title || 'RAPP Platform Status Web Page';
  var _css = attrs.css || [];
  var _js = attrs.js || [];
  var _require = attrs.require || [];
  var _meta = attrs.meta || undefined;
  var _plainScripts = attrs.plain_scripts || (function(){return ~{};})();

  return <HEAD>{
    title: _title,
    css: _css,
    jscript: _js,
    require: _require,
    _meta,
    _plainScripts
  }
};


var FORM = function( attrs ){
  attrs = attrs || {};
  var _labelId = attrs.labelId.toString() || "";
  var _text = attrs.text.toString() || "";
  var _formId = attrs.formId || "";
  var _class = attrs.class || "";
  var _value = attrs.value || "";

  return <DIV>{
    class: "form-group",
    <LABEL>{
      class: _class,
      id: _labelId,
      <SELECT>{
        class: "selectpicker form-control",
        id: _formId,
        <OPTION>{
          value: _value,
          _text
        }
      }
    }
  }
};

exports.HEADER = HEADER;
exports.META = META;
exports.FORM = FORM;
