
import service listener();

serverIP = 'localhost'
hopService_listeningPort = '9001'
hopUser_username = ''
hopUser_password = ''

serverParams = {host:serverIP, 
  asynchronous:false, port:hopService_listeningPort, user:hopUser_username, 
  password:hopUser_password };

response = listener({say:'hello'}).post(
  function(data){
    console.log(data);
  }, serverParams);


