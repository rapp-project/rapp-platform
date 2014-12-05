function writeFileSync( path, data ) {
    console.log( "in write file path", path );
    var p = #:open-output-file( #:js-tostring( path, #:%this ) );
    #:display( #:js-tostring( data, #:%this ), p );
    #:close-output-port( p );
}

exports.writeFileSync = writeFileSync;
