RAPP Platform Front-End Web Services
-------------------------------------

## Synopsis

**RAPP Platform Web Services** are developed under the  [HOP web broker](https://github.com/manuel-serrano/hop).
These services are used in order to communicate with the RAPP Platform ecosystem and access RIC(RAPP Improvement Center) AI modules.

| Platform Services                 | Description   |
| :-------------------------------: | :-----------: |
| face_detection                    |  Performs face detection on given input image frame                        |
| qr_detection                      |  Performs face detection on given input image frame                        |
| text_to_speech                    |  Performs text-to-speech on given input plain text                         |
| denoise_profile                   |  Used in order to perform given user's audio profile                       |
| speech_detection_sphix4           |  Performs speech-detection using the Platform integrated Sphinx4 engine    |
| speech_detection_google           |  Performs speech-detection using the Platform integrated Google engine     |
| available_services                |  Returns a list of the Platform available services (up-to-date)            |
| ontology_subclasses_of            |  Perform Ontology, subclasses-of, query                                    |
| ontology_superclasses_of          |  Perform Ontology, superclasses-of, query                                  |
| ontology_is_supsuperclass_of      |  Perform Ontology, is-subsuperclass-of, query                              |
| cognitive_test_chooser            |  Returns a Cognitive Exercise literal that describes the test              |
| record_cognitive_test_performance |  Record user's performance on given Cognitive Exercise                     |


## Services


#### QR-Detection

```javascript
qr_detection ( {file_uri: ''} )
```

##### Service request

- file_uri: Destination where the posted file data (**image file**) are saved by hop-server. Data are posted using a multipart/form-data post request using this field. e.g.


```python
file = {'file_uri': open(<to-send-file-path>, 'rb')}
```

##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode the received data.

```javascript
{ qr_centers: [], qr_messages: [], error: '' }
```

- qr_centers: Vector that containes points (x,y) of found QR in an image frame.
- qr_messages: Vector that containes message descriptions of found QR in an image frame.
- error: Error message.

Response Sample:

```javascript
{
  qr_centers: [ { y: 165, x: 165 } ],
  qr_messages: ['rapp project qr sample'],
  error: ''
}
```


#### Face-Detection

```javascript
face_detection ( {file_uri: ''} )
```

##### Service request

- file_uri: Destination where the posted file data (**image file**) are saved by hop-server. Data are posted using a multipart/form-data post request using this field. e.g.


```python
file = {'file_uri': open(<to-send-file-path>, 'rb')}
```

##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode the received data.

```javascript
{ faces: [{ up_left_point: {}, down_right_point: {} }], error: '' }
```

Point coordinates are presented in Cartesian Coordinate System as:

```javascript
{x: <value_int>, y: <value_int>}
```

- faces: Dynamic vector that contains recognized faces in an image frame.
- up_left_point: This Object literal contains the up-left point coordinates of detected face.
- up_left_point: This Object literal contains the down-right point coordinates of detected face.
- error: Error message.

Response Sample:

```javascript
 {
   faces: [{
     up_left_point: { y: 200, x: 212 },
     down_right_point: { y: 379, x: 391 }
   }],
   error: ''
 }
```


### Speech Detection related services.
---------------------------------------

#### Denoise Audio Profile

```javascript
set_denoise_profile ( {file_uri: '', audio_source: '', user: ''} )
```

##### Service request

- **'file_uri'**: Destination where the posted file data (**audio data file**) are saved by hop-server. Data are posted using a multipart/form-data post request using this field. e.g.


```python
file = {'file_uri': open(<to-send-file-path>, 'rb')}
```

- **'audio_source'**: A value that presents the <robot>_<encode>_<channels> information for the audio source data. e.g "nao_wav_1_ch".
- **'user'**: User’s name. Used for per-user profile denoise configurations.


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode the received data.

```javascript
{ error: '<error_message>' }
```

- error: Error message.


#### Speech-Detection-Sphinx4

```javascript
speech_detection_sphinx4 ( { file_uri: '', language: '', audio_source: '', words: [], sentences: [], grammar: [], user: ''})
```

##### Service request

- **'file_uri'**: Destination where the posted file data (**audio data file**) are saved by hop-server. Data are posted using a multipart/form-data post request using this field. e.g.

```python
file = {'file_uri': open(<to-send-file-path>, 'rb')}
```

- **'language'**: Language to be used by the speech_detection_sphinx4 module. Currently valid language values are ‘gr’ for Greek and ‘en’ for English.
- **'audio_source'**: A value that presents the <robot>_<encode>_<channels> information for the audio source data. e.g "nao_wav_1_ch".
- **'words[]'**: A vector that carries the words to search for into the voice-audio-source.
- **'sentences[]'**: The under consideration sentences.
- **'grammar[]'**: Grammars to use in speech recognition.
- **'user'**: User’s name. Used for per-user profile denoise configurations.


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode the received data.

```javascript
{ words: [], error: '<error_message>' }
```

- **'error'**: Error message.
- **'words[]'**: A vector that contains the "words-found"


#### Speech-Detection-Google

```javascript
speech_detection_sphinx4 ( { file_uri: '', audio_source: '',  user: '', language: ''} )
```

##### Service request

- **'file_uri'**: Destination where the posted file data (**audio data file**) are saved by hop-server. Data are posted using a multipart/form-data post request using this field. e.g.

```python
file = {'file_uri': open(<to-send-file-path>, 'rb')}
```

- **'language'**: Language to be used by the speech_detection_sphinx4 module. Currently valid language values are ‘gr’ for Greek and ‘en’ for English.
- **'audio_source'**: A value that presents the {robot}_{encode}_{channels} information for the audio source data. e.g "nao_wav_1_ch".
- **'user'**: User’s name. Used for per-user profile denoise configurations.


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
the received data.

```javascript
{ words: [], alternatives: [] error: '<error_message>' }
```

- **'error'**: Error message.
- **'words[]'**: A vector that contains the "words-found" with highest confidence.
- **'alternatives[[]]'**: Alternative sentences. e.g. [['send', 'mail'], ['send', 'email'], ['set', 'mail']...]



### Ontology related services.
------------------------------

The following Platform services give access to the Platform integrated Ontology system.

#### Ontology-SubClasses-Of

```javascript
ontology_subclasses_of ( { query: ''} )
```

##### Service request

- **'query'**: The query to the ontology database.


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
the received data.

```javascript
{ results: [], error: '<error_message>' }
```

- **'results'**: Query results returned from ontology database.
- **'error'**: Error message.


```javascript
 { results: [ 'http://knowrob.org/kb/knowrob.owl#Oven',
   'http://knowrob.org/kb/knowrob.owl#MicrowaveOven',
   'http://knowrob.org/kb/knowrob.owl#RegularOven',
   'http://knowrob.org/kb/knowrob.owl#ToasterOven'],
   error: ''
}
```


#### Ontology-SuperClasses-Of


```javascript
ontology_superclasses_of ( { query: ''} )
```

##### Service request

- **'query'**: The query to the ontology database.


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode the received data.

```javascript
{ results: [], error: '<error_message>' }
```

- **'results'**: Query results returned from ontology database.
- **'error'**: Error message.


#### Ontology-Is-SubSuperClass-Of

```javascript
ontology_is_subsuperclass_of ( { parent_class: '', child_class: '', recursive: false } )
```

##### Service request

- **'parent_class'**: The parent class name.
- **'child_class'**: The child class name.
- **'recursive'**: Defines if a recursive procedure will be used (true/false).


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
the received data.

```javascript
{ result: Bool, error: '<error_message>' }
```

- **'result'**: Success index on ontology-is-subsuperclass-of query.
- **'error'**: Error message.



### Text-To-Speech (tts) related services
-----------------------------------------

#### Text-To-Speech

```javascript
text_to_speech( { text: '', language: ''} )
```

##### Service request

- **'text'**: Input text to translate to audio data.
- **'language'**: Language to be used for the TTS module. Valid values are currently **el** and **en**


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
the received data.

```javascript
{ payload: <audio_data>, basename: <audio_file_basename>, encoding: <payload_encoding>, error: <error_message> }
```

- **'payload'**: The audio data payload. Payload encoding is defined by the 'encoding' json field. Decode the payload audio data (client-side) using the codec value from the 'encoding' field.
- **'encoding**: Codec used to encode the audio data payload. Currently encoding of binary data is done using base64 codec. Ignore this field. May be used in future implementations.
- **'basename'**: A static basename for the audio data file, returned by the platform service. Ignore this field. May be usefull in future implementations.
- **'error'**: If error was encountered, an error message is pushed in this field.



### Cognitive Exercises related services
-----------------------------------------

#### Cognitive-Test-Selector

```javascript
cognitive_test_chooser( { user: '', test_type: '' } )
```


##### Service request

- **'user'**: Username of client used to retrieve information from database. e.g "klpanagi"
- **'test_type'**: Cognitive Exercise test type. Can be one of ['ArithmeticCts', 'AwarenessCts', 'ReasoningCts']


##### Service response

 The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
 the received data.

```javascript
 { questions: [], possib_ans: [], correct_ans: [], test_instance: '', test_type: '', test_subtype: '', error: '' }
```

- **'questions'**: The exercise set of questions.
- **'possib_ans'**:  The set of answers for each question. vector<vector<string>>
- **'correct_ans'**: The set of correct answers for each question. vector<string>
- **'test_instance'**: Returned test name. For example, 'ArithmeticCts_askw0Snwk'
- **'test_type'**: Cognitive exercise class/type. Documentation on Cognitive Exercise classes can be found [here](https://github.com/rapp-project/rapp-platform/tree/CognitiveSystem/rapp_cognitive_exercise)
- **'test_subtype'**: Cognitive exercise sub-type. Documentation on Subtypes can be found [here](https://github.com/rapp-project/rapp-platform/tree/CognitiveSystem/rapp_cognitive_exercise)
- **'error'**: If error was encountered, an error message is pushed in this field.


#### Cognitive-Test-Selector

```javascript
record_cognitive_test_performance( { user: '', test_instance: '', score: 0 } )
```

##### Service request

- **'user'**: Username of client used to retrieve information from database. e.g "klpanagi"
- **'test_instance'**: Cognitive Exercise test instance. The full cognitive test entry name as reported by the **cognitive_test_chooser()**.
- **'score'**: User's performance score on given test entry.


##### Service response

The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode the received data.

```javascript
{ performance_entry: '', error: '' }
```

- **'performace_entry'**: User's cognitive test performance entry in ontology.
- **'error'**: If error was encountered, an error message is pushed in this field.


#### Available-Services

```javascript
available_services()
```

##### Service request

None.

##### Service response

```javascript
{ services: '', error: '' }
```

- **'services'**: An array of available RAPP Platform Services.
- **'error'**: Error message.


### Health - RAPP Platform Status
---------------------------------

For RAPP developers.

#### Rapp-Platform-Status

```javascript
rapp_platform_status()
```

Invoke this service from your favourite web browser:

```javascript
<rapp_platform_pub_ipaddr>/9001/hop/rapp_platform_status
```


## Tests

Developed tests and testing tools are currently located under the [rapp_testing_tools](https://github.com/rapp-project/rapp-platform/tree/master/rapp_testing_tools) package:

```shell
$ <path_to_rapp_platform_repo>/rapp_testing_tools/
```

## Contributors

- Konstaninos Panayiotou, [klpanagi@gmail.com]
- Manos Tsardoulias, [etsardou@gmail.com]
- Vincent Prunet, [vincent.prunet@inria.fr]
