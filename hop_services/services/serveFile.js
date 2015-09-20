/*!
 * @file serveFile.service.js
 * @brief Http-Response-File hop front-end service.
 * @TODO Define authorized directories for users.
 *
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

var user = process.env.LOGNAME;
var module_path = '../module/'

var hop = require ( 'hop' );
var Fs = require( module_path + 'fileUtils.js' );


/*!
 * @brief Service under file requests.
 * @param _filePath Requested file name provided by path.
 *
 * Requested file name might be provided without path in the future.
 * This means that the user might me authenticated to have access to specific
 * directories.
 *
 * NOT OPERATIONAL!!!
 */
service serveFile( {file: ''} )
{
  var filePath = Fs.resolvePath( file );

  /**
   *
   * TODO Check if is authorized user space and then return.
   */

  //var file = serveFile.resource (_filePath);
  console.log ( '[ServeFile Service]: Serving file [%s]', filePath );
  /* <Returns the requested file (data)> */
  return hop.HTTPResponseFile ( filePath );
}

