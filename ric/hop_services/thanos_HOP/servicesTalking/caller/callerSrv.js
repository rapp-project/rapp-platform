
var hop = require( "hop" );
var fs = require ('fs');
import service svc1();


 var file = svc1.resource( "pic.jpg" );
 console.log(file);
 var result1= svc1 ({path:file}).post(function(result){return result},{ host: "localhost", asynchronous: false, port: "9001" }); 
 //console.log(result1);
 
 //fs.readFile(file, function(err, original_data){
  //  fs.writeFile('image_orig.jpg', result1, function(err) {});
    //var base64Image = original_data.toString('base64');
    //var decodedImage = new Buffer(base64Image, 'base64');
    //fs.writeFile('image_decoded.jpg', decodedImage, function(err) {});
//});

//var bo = true;

//fs.writeFile('helloworld.jpg', result1, function (err) {
  //if (err) return console.log(err);
  //console.log('Hello World > helloworld.txt');
  ////bo=true;
//});
var bitmap = new Buffer(result1).toString('base64');
fs.writeFile('helloworld.jpg', result1, "base64", [callback]);
//var f=fs.createWriteStream(('name.jpeg'),{ flags: 'w', encoding: null, mode: 0666 });
//f.pipe(result1);
//f.pipe(result1);
//f.end();
//while(bo==false)
//{
  
//}
  console.log("end");
  
//});

//while(bo==false)
//{
 //// console.log("wait");
//}

 //console.log("end");
 
 
 //console.log(takis);

//service svc() {
  
  //var local_config ={c: "c", b: "b", a: "a"};
  //console.log ("Retrieving local configuration file");
  //var takis=console.log(takis);
  
  //takis=svc1(local_config);
  //console.log(takis.a);
  
  ////{ withHOP( ${svc1}(),function( r ) { document.body.appendChild( r ) } ) }
  
  
//}

