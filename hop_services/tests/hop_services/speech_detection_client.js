import service speech_detection_sphinx4();

Fs = require('../../utilities/./fileUtils');
var fileUri = '/home/klpanagi/Desktop/hop_tests/speech2Text/nai-oxi-test.wav';


var dataBuffer = Fs.readFileSync( fileUri ); 

var params = {
  'asynchronous': true,
  'port': 9001,
  'user': '',
  'password': '',
  'fail': function(err){
    console.log('Connection Error: ', err);
  }
};

var serviceLiteral = {
  'fileData': dataBuffer.data,
  'audio_source': 'nao_wav_1_ch',
  'words': ['όχι', 'ναι'],
  'sentences':['όχι', 'ναι'],
  'grammar': [],
  'language': 'gr'
};
 

var resp = speech_detection_sphinx4(serviceLiteral).post(
  function(response){
    parsedMessage = JSON.parse(response);
    console.log(parsedMessage);
  }, params);
