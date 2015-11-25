REPORTER = spec

all: jshint doc

jshint:
	jshint services

doc:
	npm install
	npm run doc-gen

clean:
	rm -rf doc/
	rm -rf node_modules/


.PHONY: jshint
