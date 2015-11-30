REPORTER = spec

all:  doc deps

jshint:
	jshint services

doc: doc-services doc-templates

doc-services:  deps
	rm -rf doc/services
	npm run doc-gen-services

doc-templates: deps
	rm -rf doc/service_templates
	npm run doc-gen-templates

deps:
	npm install

clean-doc:
	rm -rf doc/

clean-deps:
	rm -rf node_modules

clean: clean-doc clean-deps


.PHONY: jshint
