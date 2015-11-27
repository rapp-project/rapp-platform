REPORTER = spec

all:  doc deps

jshint:
	jshint services

doc:  deps
	rm -rf doc/
	npm run doc-gen

deps:
	npm install

clean:
	rm -rf doc/
	rm -rf node_modules/


.PHONY: jshint
