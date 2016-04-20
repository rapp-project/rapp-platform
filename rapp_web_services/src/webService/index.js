
var parsers = {
  body: require('./bodyParser'),
  file: require('./fileParser')
};


module.exports = {
  Parsers: parsers,
  WebService: require('./webService')
};
