/*!
 * @file fileStreams.js
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



var fs = require( 'fs' );
var Fs = require( './fileUtils.js' );


function WriteStream( filePath )
{
  this.wstream_ = undefined;

  this.__create = function( )
  {
    var absFilePath = Fs.resolve_path( filePath );
    try{
      this.wstream_ = fs.createWriteStream( absFilePath );
    }
    catch(e){
      console.log(e);
      this.wstream_ = undefined;
      return false;
    }

    console.log('Created writeStream [%s]', absFilePath);
    return true;
  }

  this.__close = function( )
  {
    try {if(this.wstream_) {this.wstream_.end();}}
    catch(e)
    {
      if(__DEBUG)
      {
        console.log('Thrown an exception on line 64' +
        ' [fileStreams.js] while trying to close stream');
          throw(e);
      }
    }
  }

  if( filePath )  {this.__create();}

};


WriteStream.prototype.create = this.__create;
WriteStream.prototype.close = this.__close;


/*!
 * @brief Flushes data into the write stream.
 * @param data Data to flush into the write stream. Can be Buffer or strings.
 * Buffers are used to write binary data and strings for utf-8 formatted file.
 */
WriteStream.prototype.write = function( data ){
  this.wstream_.write( data );
}


WriteStream.prototype.close = function( ){
  this.wstream_.end();
}


/**
 * Export this module
 */
module.exports = {
  WriteStream: WriteStream
}
