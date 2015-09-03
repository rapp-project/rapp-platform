/*!
 * @file fileStreams.js
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

  if( filePath )  {this.__create();}

};


WriteStream.prototype.create = this.__create;


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
