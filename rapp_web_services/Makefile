REPORTER = spec

all:  doc deps

jshint:
	jshint services

doc: doc-services doc-templates doc-modules

doc-services:  deps
	rm -rf doc/services
	grunt jsdoc:services

doc-templates: deps
	rm -rf doc/templates
	grunt jsdoc:templates

doc-modules: deps
	rm -rf doc/commons
	grunt jsdoc:commons

deps:
	npm install

clean-doc:
	rm -rf doc/

clean-deps:
	rm -rf node_modules

clean: clean-doc clean-deps


.PHONY: jshint
