var fs = require('fs');

function readFile(_file)
{
  if(fs.existsSync(_file)){
    console.log("\033[01;33mReading requested file: %s", _file);
    return fs.readFileSync(_file);
  }
  else{
    console.log("\033[01;31mCannot access the requested file. File does not exist.");
    return 0;
  }
}

exports.readFile = readFile;
