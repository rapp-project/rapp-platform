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


/**
 * @module
 * @description Random String Generator. Generates and cache unique id
 * strings.
 *
 * @author Konstantinos Panayiotou
 * @copyright Rapp Project EU 2015
 *
 */



/**
 * @class RandStrGen
 * @description Random String Generator class. Generate cached unique strings.
 * @param {Number} [_length=5] - Generated strings length.
 * @param {Array} [_charsArray=] - Characters Array to use on string
 * generation
 *
 * @property {Method} setCharsArray(_charsArray) - Set/Re-Set characters array.
 * @property {Method} setLength(_length) - Set/Re-Set generated strings length.
 * @property {Method} addCached(_key) - Append input identity key into the
 * cache.
 * @property {Method} removeCached(_key) - Remove input identity key from cache
 * @property {Method} create() - Create a random identity key (NOT CACHED).
 * @property {Method} createUnique() - Create and return a unique identity key.
 */
function RandStrGen ( _length, _charsArray ) {
  var charsArray_ = _charsArray ||
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXTZabcdefghiklmnopqrstuvwxyz";
  var stringLength_ = _length || 5;
  var cache_ = {};

  this.setCharsArray = function ( _charsArray ){
    charsArray_ = _charsArray || charsArray_;
  };

  this.setLength = function ( _length ){
    stringLength_ = _length || stringLength_;
  };

  this.getCharsArray = function ( ){
    return charsArray_;
  };

  this.getLength = function ( ){
    return stringLength_;
  };

  this.addCached = function ( _key ){
    cache_[ _key.toString() ] = true; //true means current string key exists.
    //console.log( 'String [%s] added into cached unique strings', _key);
  };

  this.removeCached = function ( _key )
  {
    if( cache_[ _key.toString() ] !== undefined ){
      //remove string "_key" from cached random strings.
      delete cache_[ _key.toString() ];
      //console.log( 'String [%s] removed from cached unique strings', _key);
    }
  };

  this.getCache = function ( ){
    return cache_;
  };

  this.isUnique = function ( _key ){
    if ( cache_[ _key.toString() ] === undefined ){
      return true;
    }
    else{
      return false;
    }
  };

}


/**
 * @description Create a new random identity key (string)
 * @returns {String} - The generated identity key.
 */
RandStrGen.prototype.create = function ( ){

  var randStr = '';
  for (var i=0; i<this.getLength(); i++) {
    var rnum = Math.floor(Math.random() * this.getCharsArray().length);
    randStr += this.getCharsArray().substring(rnum,rnum+1);
  }
  return randStr;
};


/**
 * @description Create a new UNIQUE identity key (string).
 * @returns {String} - The generated identity key.
 */
RandStrGen.prototype.createUnique = function ( ){

  var randStr = '';
  do{
    randStr = this.create();

  }
  while ( this.isUnique( randStr.toString() ) === false );
  this.addCached( randStr );
  return randStr;
};



module.exports = RandStrGen;
