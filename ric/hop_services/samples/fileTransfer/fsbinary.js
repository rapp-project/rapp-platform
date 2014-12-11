function writeFileSync( path, data ) {
    console.log( "File Transfer destination path: ", path );
    var p = #:open-output-file( #:js-tostring( path, #:%this ) );
    #:display( #:js-tostring( data, #:%this ), p );
    #:close-output-port( p );
}

exports.writeFileSync = writeFileSync;
