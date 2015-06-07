var fs = require('fs');
var http = require('http');
var FormData = require('form-data');
var assert = require('assert');


var form = new FormData();
form.append('fileUrl', fs.createReadStream(__dirname + '/qr_code_rapp.jpg'));


var request = http.request({
  method: 'post',
  host: '155.207.19.37',
  port: 9001,
  path: '/hop/qr_detection',
  auth: 'rappdev:rappdev',
  headers: form.getHeaders()
}); 

form.pipe(request);

request.on('response', function(res){
  console.log("return status: ", res.statusCode);
  res.on('data', function(chunk){
    console.log('Body: ' + chunk); 
    assert.ok(chunk.toString() == "OK");
  });
});
 

//setTimeout(function(){
  //assert.ok(res == 2);
  //process.exit(res == 2 ? 0: 1);
//}, 3000)

