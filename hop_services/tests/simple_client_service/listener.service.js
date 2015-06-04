var hop = require('hop');

console.log( hop.hostname );
console.log( hop.port );

service listener({file_uri:'', arg1: '', fileData: ''})
{
  console.log('Arguments: ', file_uri, arg1);
  console.log(typeof fileData);
  return 'Bitch'
};
