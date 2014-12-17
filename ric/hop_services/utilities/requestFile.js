import service serveFile_relative (name);
import service serveFile_absolute (name);

/*!
 * @brief   This function requests a file transmission from a hop service. 
 * @usage   This method can be called at any time.
 * @param   _file   Requested file name. 
 * @param   _absolutePath True means file is given by absolute Path. False otherwise.
 * @param   _serverParams   Server parameters (look at serverParams Prototype)
 */
function requestFile(_file, _absolutePath, _serverParams) 
{
  if(_absolutePath == false)
  {
    var file = serveFile_relative (_file).post(
      function(data)
      {
        console.log("Transfering requested file...");
        return data;
      },
      {
        asynchronous: _serverParams.asynchronous,
        host: _serverParams.host,
        port: _serverParams.port,
        user: _serverParams.user,
        password: _serverParams.password,
        fail: function(err) {console.log("Connection refused: ", err)}
      }
    );
    return file;
  }
  else
  {
    var file = serveFile_absolute (_file).post(
      function(data)
      {
        console.log("Transfering requested file...");
        return data;
      },
      {
        asynchronous: _serverParams.asynchronous,
        host: _serverParams.host,
        port: _serverParams.port,
        user: _serverParams.user,
        password: _serverParams.password,
        fail: function(err) {console.log("Connection refused: ", err)}
      }
    );
    return file;
  }
}


exports.requestFile = requestFile;
