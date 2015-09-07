/*!
 * @file randStringGen.js
 * @brief Write to files through WriteStreams.
 * @TODO Read through read streams
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



function randStringGen ( _length, _charsArray ) {
  var charsArray_ = _charsArray || "0123456789ABCDEFGHIJKLMNOPQRSTUVWXTZabcdefghiklmnopqrstuvwxyz";
  var stringLength_ = _length || 5;
  var cache_ = new Object();

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
      delete cache_[ _key.toString() ]; //remove string "_key" from cached random strings.
      //console.log( 'String [%s] removed from cached unique strings', _key);
    }
  };

  this.getCache = function ( ){
    return cache_;
  };

  this.isUnique = function ( _key ){
    if ( cache_[ _key.toString() ] == undefined ){
      return true;
    }
    else{
      return false;
    }
  };

};


randStringGen.prototype.create = function ( ){

  var randStr = '';
  for (var i=0; i<this.getLength(); i++) {
    var rnum = Math.floor(Math.random() * this.getCharsArray().length);
    randStr += this.getCharsArray().substring(rnum,rnum+1);
  }
  return randStr;
};


randStringGen.prototype.createUnique = function ( ){

  var randStr = '';
  do{
    randStr = this.create();

  }
  while ( this.isUnique( randStr.toString() ) == false );
  this.addCached( randStr );
  return randStr;
};



/*----<Export the randomStringGen module>----*/
module.exports = randStringGen;
