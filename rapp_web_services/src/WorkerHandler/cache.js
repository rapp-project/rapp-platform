/*!
 * @file cache.js
 * @brief Functionalities used for file caching on RAPP Platform.
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

var path = require('path');

var Fs_ = require( path.join(__dirname, '..', 'common', 'fileUtils.js') );

// Default cache directory path
var __cacheDir = '~/.hop/cache/services';

var __cacheDirCreated = false;


/*!
 * @brief Sets the direcory where cache files are stored.
 */
function setCacheDir(cacheDir)
{
  __cacheDir = cacheDir;
}


/*!
 * @brief Creates directory to store cache files.
 */
function createCacheDir(cacheDir)
{
  if( cacheDir === undefined || cacheDir === '' )
  {
    Fs_.createDirRecur(__cacheDir);
  }
  else
  {
    Fs_.createDirRecur(cacheDir);
    setCacheDir(cacheDir);
  }
  __cacheDirCreated = true;
  console.log('...file cache directory is set at [%s]\r\n', __cacheDir);
  return;
}


/* ------< Export Module >-------- */
module.exports = {
  setCacheDir: setCacheDir,
  createCacheDir: createCacheDir
};
