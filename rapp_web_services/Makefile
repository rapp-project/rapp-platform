REPORTER = spec

all:  doc deps

jshint:
	jshint services

doc: doc-services doc-templates doc-modules

doc-travis:
	sudo npm install
	grunt doc-gen-services
	grunt doc-gen-templates
	grunt doc-gen-commons

doc-services:  deps
	rm -rf doc/services
	grunt doc-gen-services

doc-templates: deps
	rm -rf doc/templates
	grunt doc-gen-templates

doc-modules: deps
	rm -rf doc/commons
	grunt doc-gen-commons

deps:
	npm install

clean-doc:
	rm -rf doc/

clean-deps:
	rm -rf node_modules

clean: clean-doc clean-deps


.PHONY: jshint
