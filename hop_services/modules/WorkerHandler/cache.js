/*!
 * @file cache.js
 */

var Fs_ = require('../fileUtils');

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
 * @brief Creates directory where we store cache files.
 */
function createCacheDir(cacheDir)
{
  if( cacheDir == undefined || cacheDir == '' )
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
}
