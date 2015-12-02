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
    },
    shell: {
      options: {
        stderr: true,
        stdout: true
      },
      init_hop: {
        command: './run.sh'
      }
    }
  });

  // Load jsdoc grunt task.
  grunt.loadNpmTasks('grunt-jsdoc');
  // Load shell grunt task
  grunt.loadNpmTasks('grunt-shell');

  // Default task.
  grunt.registerTask('default', ['jsdoc']);

  // Initiate HOP
  grunt.registerTask('init-hop', ['shell:init_hop']);

};
