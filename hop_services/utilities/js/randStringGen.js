

function randStringGen ( _length, _charsArray ) {
  var charsArray_ = _charsArray || "0123456789ABCDEFGHIJKLMNOPQRSTUVWXTZabcdefghiklmnopqrstuvwxyz";
  var stringLength_ = _length || 5;
  var cache_ = new Object();

  this.setCharsArray = function ( _charsArray ){
    charsArray_ = _charsArray || charsArray_;
  };

  this.setLength = function ( _length ){
    stringLength_ = _length || stringLength_;
  };

  this.getCharsArray = function ( ){
    return charsArray_;
  };

  this.getLength = function ( ){
    return stringLength_;
  };

  this.addCached = function ( _key ){
    cache_[ _key.toString() ] = true; //true means current string key exists.
    console.log( 'String [%s] added into cached unique strings', _key);
  };

  this.removeCached = function ( _key )
  {
    if( cache_[ _key.toString() ] !== undefined ){
      delete cache_[ _key.toString() ]; //remove string "_key" from cached random strings.
      console.log( 'String [%s] removed from cached unique strings', _key);
    }
  };

  this.getCache = function ( ){
    return cache_;
  };

  this.isUnique = function ( _key ){
    if ( cache_[ _key.toString() ] == undefined ){
      return true;
    }
    else{
      return false;
    }
  };

};


randStringGen.prototype.create = function ( ){

  var randStr = '';
  for (var i=0; i<this.getLength(); i++) {
    var rnum = Math.floor(Math.random() * this.getCharsArray().length);
    randStr += this.getCharsArray().substring(rnum,rnum+1);
  }
  return randStr;
};


randStringGen.prototype.createUnique = function ( ){

  var randStr = '';
  do{
    randStr = this.create();

  }
  while ( this.isUnique( randStr.toString() ) == false );
  this.addCached( randStr );
  return randStr;
};



/*----<Export the randomStringGen module>----*/
module.exports = randStringGen;
