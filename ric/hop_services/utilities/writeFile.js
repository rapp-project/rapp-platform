var fs = require('fs');

function writeFile(_destPath, _data)
{
  if(fs.existsSync(_destPath)){
    console.log("\033[01;34mFile [%s] allready exists. Overwriting...", _destPath);
  }
  else{
    console.log("\033[01;34mWriting requested data @ [%s]", _destPath);
  }
  fs.writeFileSync(_destPath, _data);
}

exports.writeFile = writeFile;
