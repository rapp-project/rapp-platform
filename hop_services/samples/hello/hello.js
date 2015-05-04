/* This simple service returns Hello
to run
hop -v -p 9000 hello.js   (9000 is the port, change it if you wish)
to view in browser enter address http://localhost:9000/hop/helloSvc
*/

service helloSvc () {
console.log("Hello");
return "Hello";
}
