var hop = require('hop');
var Fs = require('./fileUtils.js');


service testService({sourcePath: '', destPath: ''})
{
  if ( ! Fs.renameFile(sourcePath, destPath) )
  {
    var logMsg = 'Failed to rename file: [' + sourcePath + '] --> [' +
      destPath + ']';
    console.log(logMsg);

    Fs.rmFile(sourcePath);
    return hop.HTTPResponseString("Did not rename File");
  }
  return hop.HTTPResponseString("Renamed File");
}


var args = {
  sourcePath: '',
  destPath: 'testFile'
}


setTimeout(function(){
  testService(args).post(function(response){
    console.log(response)
    if(response.indexOf('not') > -1)
      {console.log(colors.success + '[Success]' + colors.none)}
    else
      {console.log(colors.error + '[Failed]' + colors.none)}
  });
}, 2000)


var colors = {
  success: '\033[1;32m',
  error: '\033[1;31m',
  none: '\033[0m'
}
