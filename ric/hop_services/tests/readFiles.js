
var myFs = require('../utilities/./fileUtils');
var fileUrl = '../../test_auxiliary_files/qr_code_rapp.jpg';
/*--<Works with both relative and absolute paths>--*/
var dest = 'copy.jpg';

testRead( fileUrl );
//testCopy( fileUrl, dest );

function testRead( fileUrl ){
  var file = myFs.readFileSync( fileUrl );
  console.log(file);
}

function testCopy( fileUrl, destUrl ){
  var file = myFs.readFileSync( fileUrl );
  myFs.writeFileSync( destUrl, file.data );
}

