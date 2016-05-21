Documentation about the RAPP Knowrob Wrapper: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Knowrob-wrapper)

#Methodology

The RAPP Knowrob wrapper ROS node is needed to expose
specific services to the other ROS nodes and RPSs (similarly to the MySQL wrapper concept).
In our implementation, the pure KnowRob ontology was
enhanced after the insertion of extra classes derived from the
OpenAAL ontology, as well as others needed to implement the desired RApps.

The services of the RAPP Knowrob wrapper are detailed below.

#ROS Services
Each service is analyzed below.

##Subclasses of
This service was created in order to return the subclasses of a specific ontology class. Apart from the basic functionality, one can perform recursive search in the ontology and not just in the classes’ immediate lower level connections.

Service URL: ```/rapp/rapp_knowrob_wrapper/subclasses_of```

Service type:
```bash
# Contains info about time and reference
Header header
# The class whose subclasses we want
string ontology_class
# True if the query is recursive
bool recursive
---
# Container for the subclasses
string[] results
# Possible error
string error
# true if successful
bool success
``` 

##Superclasses of

This service was created in order to return the superclasses of a specific ontology class. Apart from the basic functionality, one can perform recursive search in the ontology and not just in the classes’ immediate higher level connections.

Service URL: ```/rapp/rapp_knowrob_wrapper/superclasses_of```

Service type:
```bash
# Contains info about time and reference
Header header
# The class whose superclasses we want
string ontology_class
# True if the query is recursive
bool recursive
---
# Container for the superclasses
string[] results
# Possible error
string error
# true if successful
bool success
``` 

##Is sub-super-class of

This service was created in order to investigate two classes’ semantic relations. Apart from the basic functionality, one can perform recursive search in the ontology and not just in the classes’ immediate higher or lower level connections.

Service URL: ```/rapp/rapp_knowrob_wrapper/is_subsuperclass_of```

Service type:
```bash
# Contains info about time and reference
Header header
# The parent class
string parent_class
# The child class
string child_class
# True if the query is recursive
bool recursive
---
# True if the semantic condition applies
bool result
# Possible error
string error
# true if successful
bool success
``` 

##Create instance

This service was created in order to create an instance related to a specific ontology class. One can also store a file and comments. This service is not public but it is employed from the RIC nodes. Ownership of the new instance is assigned to the provided user which is an instance of the class Person within the ontology. The assignment involves setting an ontology attribute. The username provided in the input is the username of the user in the MySQL database and not the name of the user’s instance in the ontology. The second is also stored within the MySQL database the service acquires it by querying the MySQL database (with the username as input) through the MySQL wrapper. In case no ontology instance name (alias) is defined a new one is created utilizing the Create ontology alias service described below and the Ontology and MySQL database are updated accordingly.

Service URL: ```/rapp/rapp_knowrob_wrapper/create_instance```

Service type:
```bash
# Contains info about time and reference
Header header
# The user to whom the instance belongs
string username
# The instance’s ontology class
string ontology_class
# A file url (if needed)
string file_url
# Comments (if needed)
string comments
---
# A unique id 
string instance_name
# Possible error
string error
# true if successful
bool success
``` 

##Create ontology alias

This service was created in order to create an alias for a user within the ontology. A user ontology alias is basically an instance of the class Person that exists within the ontology. The user’s ontology alias is stored within the MySQL database in the respective column of the table User. This service accepts the MySQL username of the user and performs a check by querying the MySQL database in order to identify if an ontology alias has already been defined. If that is the case it simply returns that ontology alias. If not, it creates the ontology alias instance within the ontology, stores this information in the MySQL database and finally it returns the newly created user ontology alias. In the case that a new ontology alias is created both the ontology and the MySQL need to be updated. The service ensures that either both are updated in tandem or in case one fails no modifications take place in the other. This is critical to preserving proper synchronization between the MySQL database and the ontology.

Service URL: ```/rapp/rapp_knowrob_wrapper/create_ontology_alias```

Service type:
```bash
# Contains info about time and reference
Header header
# The user to whom the instance belongs
string username
---
# A unique id 
string ontology_alias
# Possible error
string error
# true if successful
bool success
```

##User instances of class

This service allows an ML algorithm to retrieve user-specific information in order to compute and extract personalized data. This service is not exposed via a HOP service, but is employed internally by other RIC nodes. It returns all the ontology instances that are assigned to the ontology alias of the provided user. The ontology alias is acquired again by querying the MySQL database.

Service URL: ```/rapp/rapp_knowrob_wrapper/user_instances_of_class```

Service type:
```bash
# Contains info about time and reference
Header header
# The user whose instances must be returned
string username
# The ontology class whose instances we desire
string ontology_class
---
# The instances
String[] results
# Possible error
string error
# true if successful
bool success
```  

##Load / Dump ontology

Since the KnowRob ontology framework does not provide online storage functionality, the ontology (along with the new information) must be stored in predefined time slots. This way, if a system crash occurs, the stored data won’t be lost but can be retrieved using the Load ontology ROS service. Both of these services have a common representation.

Service URL: ```/rapp/rapp_knowrob_wrapper/load_ontology```
Service URL: ```/rapp/rapp_knowrob_wrapper/dump_ontology```

Service type:
```bash
# Contains info about time and reference
Header header
# The file intended for loading or dumping the ontology
string file_url
---
# Possible error
string error
# true if successful
bool success
``` 

##Create cognitive test

This service creates a new cognitive exercise test in the ontology as an instance. This service is not exposed via a HOP service, but is employed internally by the RAPP Cognitive exercise system node. It accepts as input that parameters of the test which include its type, subtype, difficulty, variation and file path and returns the name of the ontology instance created.

Service URL: ```/rapp/rapp_knowrob_wrapper/create_cognitve_tests```

Service type:
```bash
# Contains info about time and reference
Header header
# The type of the test
string test_type
# The sub type of the test
string test_subtype
# The difficulty of the test
int32 test_difficulty
# The variation id of the test
int32 test_variation
# The file path where the test is located
string test_path
---
# The created test name
string test_name
# Possible error
string error
# true if successful
bool success
``` 

##Return cognitive tests of type

This service returns all cognitive tests of the given type that exist within the ontology. This service is not exposed via a HOP service, but is employed internally by the RAPP Cognitive exercise system node. It accepts as input the test type and returns the names of the tests and their parameters which include their subtypes, file paths, difficulties and variation ids.

Service URL: ```/rapp/rapp_knowrob_wrapper/cognitive_tests_of_type```

Service type:
```bash
# Contains info about time and reference
Header header
# The type of the tests to be returned
string test_type
---
# The names of the tests
string[] tests
# The subtype of the tests
string[] subtype
# The file paths of the tests
string[] file_paths
# The difficulty of the tests
string[] difficulty
# The variation of the tests
string[] variation
# Possible error
string error
# true if successful
bool success
``` 

##Record user cognitive test performance

This service records the user’s (patient’s) performance on a given test at a given time. Performance is measured as integer value in the 0-100 range. This service is not exposed via a HOP service, but is employed internally by the RAPP Cognitive exercise system node.

Service URL: ```/rapp/rapp_knowrob_wrapper/record_user_cognitive_tests_performance```

Service type:
```bash
# Contains info about time and reference
Header header
# The name of the test
string test
# The type of the test
string test_type
# The ontology alias of the patient who is taking the test
string patient_ontology_alias
# The score the patient achieved in the test
int32 score
# The timestamp at which the test was performed
int32 timestamp
---
# The name of the cognitive test performance entry
string cognitive_test_performance_entry
# Possible error
string error
# true if successful
bool success
``` 

##Return user cognitive test performance

This service returns all the tests of the requested type that a specific user (patient) has undertaken along with the scores achieved, the time at which they were performed and the difficulty and variation ids of the tests. This service is not exposed via a HOP service, but is employed internally by the RAPP Cognitive exercise system node.

Service URL: ```/rapp/rapp_knowrob_wrapper/user_performance_cognitve_tests```

Service type:
```bash
# Contains info about time and reference
Header header
# The ontology alias of the patient
string ontology_alias
# The type of the tests of interest
string test_type
---
# The names of the tests
string[] tests
# The scores of the tests
string[] scores
# The difficulty of the tests
string[] difficulty
# The variation ids of the tests
string[] variation
# The timestamps at which the tests were performed
string[] timestamps
# Possible error
string error
# true if successful
bool success
``` 

##Register image object to ontology
This service will register an image, annotated by the rapp_caffe_wrapper to the ontology.

Service URL: ```/rapp/rapp_knowrob_wrapper/register_image_object_to_ontology```

Service type:
```bash
# Contains info about time and reference
Header header
#the user's ontology alias
string user_ontology_alias
#the ontology class of the to be registered object
string object_ontology_class
#the caffe class of the to be registered object
string caffe_class
#the path to the image representing the object
string image_path
#timestamp of the time of registration
int32 timestamp
---
# The results of the query
#the name of the registered object entry
string object_entry
#possible error
string error
#trace information
string[] trace
#true if service call was successful
bool success 
``` 

##Retract user ontology alias
This service will remove the ontology alias of a user from the ontology.

Service URL: ```/rapp/rapp_knowrob_wrapper/retract_user_ontology_alias```

Service type:
```bash
# Contains info about time and reference
Header header
#the user ontology alias to be retracted
string ontology_alias
---
# The results of the query
#possible error
string error
#trace information
string[] trace
#true if service call was successful
bool success 
``` 

##Clear user cognitive test performance records
This service will remove all cognitive test performance records of a user from the ontology.

Service URL: ```/rapp/rapp_knowrob_wrapper/clear_user_cognitive_tests_performance_records```

Service type:
```bash
# Contains info about time and reference
Header header
#the username of the user whose records will be cleared
string username
#specify clearing the records of this test type onle
string test_type
---
# The results of the query
#possible error
string error
#trace information
string[] trace
#true if service call was successful
bool success 
``` 

#Launchers

##Standard launcher

Launches the **rapp_knowrob_wrapper** node and can be launched using
```
roslaunch rapp_knowrob_wrapper knowrob_wrapper.launch
```

#HOP services


## Ontology-SubClasses-Of

### Service-Url

```
/hop/ontology_subclasses_of
```

### Service request arguments

```js
{ query: '' }
```
- **query** (String): The query to the ontology database.


### Service response

application/json response.

```javascript
{ results: [], error: '' }
```

- **results** (Array): Query results.
- **error** (String): Error message, if one occures.


```javascript
 { results: [ 'http://knowrob.org/kb/knowrob.owl#Oven',
   'http://knowrob.org/kb/knowrob.owl#MicrowaveOven',
   'http://knowrob.org/kb/knowrob.owl#RegularOven',
   'http://knowrob.org/kb/knowrob.owl#ToasterOven'],
   error: ''
}
```


## Ontology-SuperClasses-Of

### Service-Url

```
/hop/ontology_superclasses_of
```

### Service request arguments

```js
{ query: '' }
```

- **query** (String): The query to the ontology database.


### Service response

application/json response.

```javascript
{ results: [], error: '' }
```

- **results** (Array): Query results returned from ontology database.
- **error** (String): Error message, if one occures.


## Ontology-Is-SubSuperClass-Of

### Service-Url

```
/hop/ontology_is_subsuperclass_of
```

### Service request arguments

```js
{ parent_class: '', child_class: '', recursive: false }
```

- **parent_class** (String): The parent class name.
- **child_class** (String) The child class name.
- **recursive** (Boolean): Defines if a recursive procedure will be used (true/false).


### Service response

application/json response.

```javascript
{ result: true, error: '' }
```

- **result** (Boolena): Success index on ontology-is-subsuperclass-of query.
- **error** (String): Error message, if one occures.

