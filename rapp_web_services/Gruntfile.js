/*global module:false*/
module.exports = function(grunt) {

  // Project configuration.
  grunt.initConfig({
    pkg: grunt.file.readJSON('package.json'),
    // Task configuration.
    jsdoc: {
      services: {
        src: ['services/*.js', 'services/README.md'],
        options: {
          destination: 'doc/services',
          template: "node_modules/ink-docstrap/template",
          configure: "config/jsdoc-services.conf.json"
        }
      },
      commons: {
        src: ['modules/common/*.js', 'modules/common/README.md'],
        options: {
          destination: 'doc/commons',
          template: "node_modules/ink-docstrap/template",
          configure: "config/jsdoc-commons.conf.json"
        }
      },
      templates: {
        src: ['services/templates/*.js', 'services/templates/README.md'],
        options: {
          destination: 'doc/templates',
          template: "node_modules/ink-docstrap/template",
          configure: "config/jsdoc-templates.conf.json"
        }
      }
    }
  });

  // These plugins provide necessary tasks.
  grunt.loadNpmTasks('grunt-jsdoc');

  // Default task.
  grunt.registerTask('default', ['jsdoc']);

};
