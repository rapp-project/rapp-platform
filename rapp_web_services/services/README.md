RAPP Platform Web Services
-------------------------------------

## Synopsis

**RAPP Platform Web Services** are developed under the  [HOP Web Server](https://github.com/manuel-serrano/hop).
These services are used in order to communicate with the RAPP Platform ecosystem and access RIC(RAPP Improvement Center) AI modules.

| Platform Services             | Description   								|
| :---------------------------:	| :---------------------------------------------------------------------------:	|
| face_detection                |  Detect faces on an image frame                        			|
| qr_detection                  |  Detect and recognize Qr-Codes on an image frame                        	|
| text_to_speech                |  Text-to-speech translation on given input plain text                         |
| set_noise_profile             |  Set user's noise profile. Used to apply denoising on speech-recognition 	|
| speech_detection_sphix4       |  Performs speech-detection using the Platform integrated Sphinx4 engine    	|
| speech_detection_google       |  Performs speech-detection using the Platform integrated Google engine     	|
| available_services            |  Returns a list of the Platform available services (up-to-date)            	|
| ontology_subclasses_of        |  Perform Ontology, subclasses-of, query                                    	|
| ontology_superclasses_of      |  Perform Ontology, superclasses-of, query                                  	|
| ontology_is_supsuperclass_of  |  Perform Ontology, is-subsuperclass-of, query                              	|
| cognitive_test_chooser        |  Returns a Cognitive Exercise literal that describes the test              	|
| cognitive_record_performance  |  Record user's performance on a Cognitive Exercise                     	|
| cognitive_get_history	        |  Returns user's history on Cognitive Exercise(s)				|
| cognitive_get_score		|  Returns user's performance scores on Cognitive Exercise(s)			|
| email_fetch			|  Fetch received user's emails							|
| email_send			|  Send an email, using user's account					        |
| geolocation			|  Get information about client's location				        |
| weather_report_forecast	|  Get detailed information about future weather conditions			|
| weather_report_current	|  Get detailed information about current weather conditions			|
| news_explore			|  Search for news articles							|
| hazard_detection_door_check	|  Detect open-doors (hazard) on an image frame					|
| hazard_detection_light_check	|  Detect lights-on (hazard) on an image frame					|
| human_detection		|  Detect human existance on on an image frame 					|
| login_user			|  Login existing user								|
| register_user_from_platform	|  Add new platform user using platform credentials				|
| register_user_from_store	|  Add new platform user using rapp_store credentials				|


## Service specifications - Request arguments and response objects


The Web Services listen to **POST** requests and can parse contents of the following type (Content-Type):

- application/x-www-form-urlencoded
- multipart/form-data

A lot of services require from the client to upload a file to the server for processing, like Computer-VIsion, Speech-Recognition and more.
The *multipart/form-data* content-type is used to call a service that requresres file upload to the server.
Otherwise, use *application/x-www-form-urlencoded*.

All data, except files, have to be  be send under a field named **json**.
In case of using *application/x-www-form-urlencoded* type this will look like:

```
POST /hop/ontology_subclasses_of HTTP/1.1
Connection: keep-alive
...
Content-Type: application/x-www-form-urlencoded

json=%7B%22query%22%3A+%22Oven%22%7D
```

and in case of multipart/form-data:

```
POST /hop/face_detection HTTP/1.1
Connection: keep-alive
...
Content-Type: multipart/form-data; boundary=993ac36c568042aa86582023b7422092

--993ac36c568042aa86582023b7422092
Content-Disposition: form-data; name="json"

{"fast": false}
--993ac36c568042aa86582023b7422092
Content-Disposition: form-data; name="file"; filename="Lenna.jpg"

<file-data-here>
...

```

-------------------------------
### Computer Vision
-------------------------------

#### Face-Detection

##### Service-Url

```
/hop/face_detection
```

##### Service request arguments

```js
{file: '', fast: false}
```

- **file**: Path to the uploaded file (**image file**), stored by hop-server. This is the form-data name to attach the file to.
- **fast** (Boolean): If true, detection will take less time but it will be less accurate.

##### Service response

application/json response.

```js
{ faces: [{<face_1>}, ..., {<face_n>}], error: '' }
```

- **faces** (Array):  Vector with the recognized faces in an image frame.
- **error** (String): Error message, if one occures.

where **face_x** is an object of the following structure:

```js
face: { up_left_point: {<point>}, down_right_point: {<point>} }
```

- **up_left_point** (Object): The up-left point coordinates of the detected face.
- **up_left_point** (Object): The down-right point coordinates of the detected face.

and **point** coordinates are presented in Cartesian Coordinate System as:

```js
point: {x: <value_int>, y: <value_int>}
```

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


#### QR-Detection

##### Service-Url

```
/hop/qr_detection
```

##### Service request arguments

```js
{file: ''}
```

- **file**: Path to the uploaded file (**image file**), stored by hop-server. This is the form-data name to attach the file to.

##### Service response

application/json response.

```javascript
{ qr_centers: [{<point_1>}, ..., {<point_n>}], qr_messages: ["<qr_msg_1>", ..., "<qr_msg_n>"], error: '' }
```

where **point_n** coordinates are presented in Cartesian Coordinate System as:

```js
point: {x: <value_int>, y: <value_int>}
```

- **qr_centers** (Array): Vector of points (x,y) of found QR in the image frame.
- **qr_messages** (Array): Vector that containes message descriptions of found QR in an image frame (Array of Strings).
- **error** (String): Error message, if one occures.

Response Sample:

```javascript
{
  qr_centers: [ { y: 165, x: 165 } ],
  qr_messages: ['rapp project qr sample'],
  error: ''
}
```

#### Hazard-Detection-Door-Check

##### Service-Url

```
/hop/hazard_detection_door_check
```

##### Service request arguments

```js
{file: ''}
```

- **file**: Path to the uploaded file (**image file**), stored by hop-server. This is the form-data name to attach the file to.


##### Service response

application/json response.

```js
{door_angle: 0, error: ""}
```


- **door_angle** (Integer): The angle of the detected door, on the image frame. (Integer)
- **error** (String): Error message, if one occures.


#### Hazard-Detection-Light-Check

##### Service-Url

```
/hop/hazard_detection_light_check
```

##### Service request arguments

```js
{file: ''}
```

- **file**: Path to the uploaded file (**image file**), stored by hop-server. This is the form-data name to attach the file to.


##### Service response

application/json response.

```js
{light_level: 0, error: ""}
```


- **light_level** (Integer): The, detected on the iimage frame, light level. (Integer)
- **error** (String): Error message, if one occures. (String)


#### Human-Detection

##### Service-Url

```
/hop/human_detection
```

##### Service request arguments

```js
{file: ''}
```

- **file**: Path to the uploaded file (**image file**), stored by hop-server. This is the form-data name to attach the file to.


##### Service response

application/json response.

```js
{humans: [{<human_1>}, ..., {<human_n>}], error: ""}
```

- **humans** (Array): Array of detected humans.
- **error** (String): Error message, if one occures. (String)


where **human_x** is an object of the following structure:

```js
human: { up_left_point: {x: 0, y: 0}, down_right_point: {x: 0, y: 0} }
```

Each point (x, y) is presented in Cartesian Coordinate System as:

```js
point2D: {x: <val, y: <val>}
```

-----------------------------------------------------------
### Speech Recognition
-----------------------------------------------------------

#### Set-Noise-Profile

##### Service-Url

```
/hop/set_noise_profile
```

##### Service request arguments

```js
{file: '', audio_source: ''} 
```

- **file**: Path to the uploaded file (**audio file**), stored by hop-server. This is the form-data name to attach the file to.
- **audio_source** (String): A value that presents the <robot>_<encode>_<channels> information for the audio source data. e.g "nao_wav_1_ch".


##### Service response

application/json response.

```javascript
{ error: '' }
```

- **error** (String): Error message, if one occures.


#### Speech-Detection-Sphinx4

##### Service-Url

```
/hop/speech_detection_sphinx4
```

##### Service request arguments

```js
{ file: '', language: '', audio_source: '', words: [], sentences: [], grammar: []}
```

- **file**: Path to the uploaded file (**audio file**), stored by hop-server. This is the form-data name to attach the file to.
- **language** (String): Language to be used by the speech_detection_sphinx4 module. Currently valid language values are ‘gr’ for Greek and ‘en’ for English.
- **audio_source** (String): A value that presents the <robot>_<encode>_<channels> information for the audio source data. e.g "nao_wav_1_ch".
- **words** (Array): A vector that carries the words to search for into the voice-audio-source.
- **sentences** (Array): The under consideration sentences.
- **grammar** (Array): Grammars to use in speech recognition.


##### Service response

application/json response.

```javascript
{ words: [], error: '' }
```

- **words** (Array): A vector with the "words-found"
- **error** (String): Error message, if one occures.


#### Speech-Detection-Google

##### Service-Url

```
/hop/speech_detection_google
```

##### Service request arguments

```js
{ file: '', audio_source: '', language: ''}
```

- **file**: Path to the uploaded file (**audio file**), stored by hop-server. This is the form-data name to attach the file to.
- **language** (String): Language to be used by the speech_detection_sphinx4 module. Currently valid language values are ‘gr’ for Greek and ‘en’ for English.
- **audio_source** (String): A value that presents the {robot}_{encode}_{channels} information for the audio source data. e.g "nao_wav_1_ch".


##### Service response

application/json response.

```javascript
{ words: [], alternatives: [] error: '' }
```

- **'words'** (Array): A vector that contains the "words-found" with highest confidence.
- **alternatives** (Array): Alternative sentences. e.g. [['send', 'mail'], ['send', 'email'], ['set', 'mail']...]
- **error** (String): Error message, if one occures.

------------------------------
### Ontology Queries
------------------------------

The following Platform services give access to the Platform integrated Ontology system.

#### Ontology-SubClasses-Of

##### Service-Url

```
/hop/ontology_subclasses_of
```

##### Service request arguments

```js
{query: ''}
```
- **query** (String): The query to the ontology database.


##### Service response

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


#### Ontology-SuperClasses-Of

##### Service-Url

```
/hop/ontology_superclasses_of
```

##### Service request arguments

```js
{query: ''}
```

- **query** (String): The query to the ontology database.


##### Service response

application/json response.

```javascript
{ results: [], error: '' }
```

- **results** (Array): Query results returned from ontology database.
- **error** (String): Error message, if one occures.


#### Ontology-Is-SubSuperClass-Of

##### Service-Url

```
/hop/ontology_is_subsuperclass_of
```

##### Service request arguments

```js
{ parent_class: '', child_class: '', recursive: false }
```

- **parent_class** (String): The parent class name.
- **child_class** (String) The child class name.
- **recursive** (Boolean): Defines if a recursive procedure will be used (true/false).


##### Service response

application/json response.

```javascript
{ result: true, error: '' }
```

- **result** (Boolena): Success index on ontology-is-subsuperclass-of query.
- **error** (String): Error message, if one occures.



-----------------------------------------
### Text-To-Speech (tts) related services
-----------------------------------------

#### Text-To-Speech

##### Service-Url

```
/hop/text_to_speech
```

##### Service request arguments

```js
{text: '', language: ''}
```

- **text** (String): Input text to translate to audio data.
- **language** (String): Language to be used for the TTS module. Valid values are currently **el** and **en**


##### Service response

application/json response.

```javascript
{ payload: <audio_data>, basename: <audio_file_basename>, encoding: <payload_encoding>, error: <error_message> }
```

- **payload** (String/base64): The audio data payload. Payload encoding is defined by the 'encoding' json field. Decode the payload audio data (client-side) using the codec value from the 'encoding' field.
- **encoding** (String): Codec used to encode the audio data payload. Currently encoding of binary data is done using base64 codec. Ignore this field. May be used in future implementations.
- **basename** (String): A static basename for the audio data file, returned by the platform service. Ignore this field. May be usefull in future implementations.
- **error** (String): Error message, if one occures.


----------------------------------
### Cognitive Exercises
----------------------------------

#### Cognitive-Test-Selector

##### Service-Url

```
/hop/cognitive_test_chooser
```

##### Service request arguments

```js
{test_type: ''}
```

- **test_type** (String): Cognitive Exercise test type. Can be one of ['ArithmeticCts', 'AwarenessCts', 'ReasoningCts']


##### Service response

application/json response.

```javascript
 { questions: [], possib_ans: [], correct_ans: [], test_instance: '', test_type: '', test_subtype: '', error: '' }
```

- **questions** (Array): The exercise set of questions.
- **possib_ans**(Array):  The set of answers for each question. vector<vector<string>>
- **correct_ans**(Array): The set of correct answers for each question. vector<string>
- **test_instance** (String): Returned test name. For example, 'ArithmeticCts_askw0Snwk'
- **test_type** (String): Cognitive exercise class/type. Documentation on Cognitive Exercise classes can be found [here](https://github.com/rapp-project/rapp-platform/tree/CognitiveSystem/rapp_cognitive_exercise)
- **test_subtype** (String): Cognitive exercise sub-type. Documentation on Subtypes can be found [here](https://github.com/rapp-project/rapp-platform/tree/CognitiveSystem/rapp_cognitive_exercise)
- **error** (String): Error message, if one occures.


#### Cognitive-Record-Performance

##### Service-Url

```
/hop/cognitive_record_performance
```

##### Service request arguments

```js
{ test_instance: '', score: 0 }
```

- **test_instance** (String): Cognitive Exercise test instance. The full cognitive test entry name as returned by the **cognitive_test_chooser** web service.
- **score** (Integer): User's performance score on given test entry.


##### Service response

application/json response.

```javascript
{ performance_entry: '', error: '' }
```

- **performace_entry** (String): User's cognitive test performance entry in ontology.
- **error** (String): Error message, if one occures.


#### Cognitive-Get-History

##### Service-Url

```
/hop/cognitive_get_history
```

##### Service request arguments

```js
{ from_time: '', to_time: 0, test_type: '' }
```

- **test_type** (String): Cognitive Exercise test type. Can be one of ['ArithmeticCts', 'AwarenessCts', 'ReasoningCts'] or leave empty ("") for all.
- **from_time** (Integer): Unix timestamp.
- **to_time** (Integer):  Unix timestamp.


##### Service response

application/json response.

```javascript
{ records: {}, error: '' }
```

- **records** (Object): Users history records on Cognitive Exerises
- **error** (String): Error message, if one occures.


#### Cognitive-Get-Scores

##### Service-Url

```
/hop/cognitive_get_history
```

##### Service request arguments

```js
{ up_to_time: 0, test_type: '' }
```

- **test_type** (String): Cognitive Exercise test type. Can be one of ['ArithmeticCts', 'AwarenessCts', 'ReasoningCts'] or leave empty ("") for all.
- **up_to_time** (Integer):  Unix timestamp. Return scores that have been recorder up to this time value.


##### Service response

application/json response.

```javascript
{ test_classes: [], scores: [], error: '' }
```

- **test_classes** (Array): An array of the test classes indexes.
- **scores** (Array): Array of scores. Each array index corresponds to the test class of the **test_classes** property.
- **error** (String): Error message, if one occures.


----------------------------------
### Email Support
----------------------------------


#### Email-Fetch

##### Service-Url

```
/hop/email_fetch
```

##### Service request arguments

```js
{ email: '', passwd: '', server: '', port: '', email_status: '', from_date: 0, to_date: 0, num_emails: 0 } 
```

- **email** (String): The user's email username
- **passwd** (String): The user's email password
- **server** (String): The email server's imap address, i.e. 'imap.gmail.com'
- **port** (String): The email server imap port
- **email_status** (String): Define which mails the users requests. Values: ALL, UNSEEN(DEFAULT)
- **from_date** (Integer): Emails since date. Unix timestamp.
- **to_date** (Integer): Emails until date. Unix timestamp.
- **num_emails** (Integer): Number of requested emails

##### Service response

application/json response.

```javascript
{ emails: [{<emailEntry_1>}, ..., {<emailEntry_n>}], error: '' }
```

where emailEntry is an object of structure:

```js
{sender: '', receivers: [], body: '', date: '', body: '', attachments: []}
```

- **emails** (Array): An array of emailEntry objects. 
- **error** (String): Error message, if one occures.


#### Email-Send

##### Service-Url

```
/hop/email_send
```

##### Service request arguments

```js
{ email: '', passwd: '', server: '', port: '', recipients: [], body: '', subject: '', file: '' } 
```

- **email** (String): The user's email username
- **passwd** (String): The user's email password
- **server** (String): The email server's smtp address, i.e. 'smtp.gmail.com'
- **port** (String): The email server imap port
- **recipients** (Array):  Email addresses of the recipients
- **body** (String): The email body
- **subject** (String): The email subject
- **file**: File attachment. Single file. **In case of multiple attachments a zip file must be send to this field name.**


##### Service response

application/json response.

```javascript
{ error: '' }
```

- **error** (String): Error message, if one occures.


---------------------------------
### Weather Report
---------------------------------

#### Weather-Report-Current

##### Service-Url

```
/hop/weather_report_current
```

##### Service request arguments

```js
{ city: '', weather_reporter: '', metric: 0 }
```

- **city** (String): The desirec city
- **weather_reporter** (String): The weather API to use. Defaults to "yweather" .
- **metric** (Integer): The return value units.


##### Service response

application/json response.

```javascript
{ 
  weather_current: {date: '', temperature: '', weather_description: '', humidity: '',
                 		visibility: '', pressure: '', wind_speed: '', wind_temperature: '', wind_direction: ''},
  error: ''
}
```

- **date** (String): The date.
- **temperature** (String): The current temerature.
- **weather_description** (String): A brief desctiption of the current weather
- **humidity** (String): The current humidity
- **visibility** (String): The current visibility.
- **pressure** (String): The current pressure.
- **wind_speed** (String): The current speed of the wind.
- **wind_temperature** (String): The current temperature of the wind.
- **wind_direction** (String): The current direction of the wind.


#### Weather-Report-Forecast

##### Service-Url

```
/hop/weather_report_forecast
```

##### Service request arguments

```js
{ city: '', weather_reporter: '', metric: 0 } 
```

- **city** (String): The desirec city
- **weather_reporter** (String): The weather API to use. Defaults to "yweather" .
- **metric** (Integer): The return value units.


##### Service response

application/json response.

```javascript
{ forecast: [{<forecastEntry_1>}, ...,  {<forecastEntry_n>}], error: '' }
```

- **forecast** (Array): Array of **forecastEntry** objects.
- **error** (String): Error message, if one occures.

where forecast entries are forecastEntry objects:

{high_temp: '', low_temp: '', description: '', date: ''}



---------------------------------
### Other
---------------------------------

#### Available-Services

```javascript
available_services()
```

##### Service request arguments

None.

##### Service response

```javascript
{ services: [], error: '' }
```

- **services** (Array): An array of available RAPP Platform Services.
- **error** (String): Error message, if one occures.


#### Geolocation

##### Service-Url

```
/hop/geolocation
```

##### Service request arguments

```js
{ipaddr: '', engine: ''} 
```

- **ipaddr**: The machine's ipaddr
- **engine**: Engine to use. Defaults to 'ip-api' (Currently the only supported).

##### Service response

application/json response.

```javascript
{ city: '', country: '', country_code: '', latitude: 0.0, longtitude: 0.0, region: '', timezone: '', zip: '', error: '' }
```

- **city**: (String): The city.
- **country** (String): The country.
- **contry_code** (String): The country code.
- **latitude**: (Float): The latitude.
- **longtitude** (Float): The longtitude.                          
- **timezone** (String): The timezone.
- **zip** (String): The zip postal code.
- **error** (String): Error message, if one occures.


#### News-Explore

##### Service-Url

```
/hop/news_explore
```

##### Service request arguments

```js
{news_engine: '', keywords: [], exclude_titles: [], region: '', topic: '', num_news: 0} 
```

- **news_engine** (String): The news search engine to use.
- **keywords** (Array): Desired keywords.
- **exclude_titles** (Array): Reject list of previously read articles, in order to avoid duplicates.
- **region** (String): Language/Region.
- **topic** (String): Main topics, i.e. sports, politics, etc.
- **num_news** (Integer): Number of news stories.



##### Service response

application/json response.

```javascript
```javascript
{ news_stories: [{<story_1>}, ...,  {<story_n>}], error: '' }
```

- **news_stories** (Array): Array of **story** objects.
- **error** (String): Error message, if one occures.

where *story* object is:

```js
{ title: '', content: '', puplisher: '', publishedDate: '', url: '' }
```

- **title** (String): Article title.
- **content** (String): Article brief content.
- **publisher** (String): Article publisher.
- **publishedDate** (String): Article publication date.
- **url** (String): Article original url.


## Tests

Developed tests and testing tools are currently located under the [rapp_testing_tools](https://github.com/rapp-project/rapp-platform/tree/master/rapp_testing_tools) package:

```shell
$ <path_to_rapp_platform_repo>/rapp_testing_tools/
```

## Contributors

- Konstaninos Panayiotou, [klpanagi@gmail.com]
- Manos Tsardoulias, [etsardou@gmail.com]
- Vincent Prunet, [vincent.prunet@inria.fr]
