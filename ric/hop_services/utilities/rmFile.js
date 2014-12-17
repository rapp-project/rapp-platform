
var fs = require('fs');

function rmFile(_filePath)
{
  var _msg = new String("Successfully deleted file: ");
  if(fs.existsSync(_filePath)){
    fs.unlinkSync(_filePath);
    return true;
  }
  else{
    console.log("File [%s] does not exist!", _filePath);
    return false;
  }
}

exports.rmFile = rmFile;
