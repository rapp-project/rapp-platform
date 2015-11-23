REPORTER = spec

all: jshint doc

jshint:
	jshint services

doc:
	mkdir doc/
	jsdoc -c config/doc_conf.json

clean:
	rm -rf doc/


.PHONY: jshint
