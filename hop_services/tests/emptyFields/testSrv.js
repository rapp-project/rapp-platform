var hop = require('hop');

service testSrv({param1: '', param2: '', param3: ''})
{
  console.log("param1-val: %s, param2-val: %s, param3-val: %s", param1,
    param2, param3);
  var response = {
    param1: param1,
    param2: param2,
    param3: param3
  }
  return hop.HTTPResponseJson(response);
}



setTimeout(function(){
  var args = {
    param1: 'klpanagi',
    param2: '',
    param3: ''
  }
  testSrv(args).post(function(response){
    console.log(response);
    var success = true;
    for(param in response) {
      if(response[param] === args[param]) {continue;}
      else {success = false;}
    }
    console.log('\n----- Test Result:')
    if( success )
      {console.log(colors.success + '[Success]' + colors.none)}
    else
      {console.log(colors.error + '[Failed]' + colors.none)}
  });

}, 1000)


var colors = {
  success: '\033[1;32m',
  error: '\033[1;31m',
  none: '\033[0m'
};
