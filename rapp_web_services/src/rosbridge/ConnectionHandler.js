
var util = require('util');

function Timer(clb, tickT) {
  this.tickT = tickT;
  this.clb = clb;
}

Timer.prototype.once = function() {
  setTimeout(this.clb, this.rickT);
};


Timer.prototype.start = function() {
  setInterval(this.clb, this.tickT);
};


function ConnectionHandler(ros, opts) {
  this.rosH = ros;

  opts = opts || {};
  this.tickT = opts.reconnectTime || 2000;

  var that = this;

  this.rosH.ros.on('connection', function() {
    var msg = util.format(
      "Connection to rosbridge websocket server established: %s ",
      that.rosH.url);
    that.rosH.logger.log(msg);
    that.rosH.isConnected = true;
  });

  this.rosH.ros.on('error', function(error) {
    //that.rosH.logger.log('Error connecting to websocket server: ', error);
    that.rosH.isConnected = false;
    if (that.rosH.reconnect) {
      setTimeout(function() {
        that.rosH.connect();
      }, that.tickT);
    }
  });

  this.rosH.ros.on('close', function() {
    var msg = util.format(
      "Connection to rosbridge websocket server closed: %s",
      that.rosH.url);
    that.rosH.logger.log(msg);
    that.rosH.isConnected = false;
    if (that.rosH.reconnect) {
      setTimeout(function() {
        that.rosH.connect();
      }, that.tickT);
    }
  });

}


module.exports = ConnectionHandler;
