Documentation about the RAPP Text to speech using Espeak & Mbrola: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Text-to-speech-using-Espeak-&-Mbrola)

In order to provide speech capabilities to the robots that do not have a Text-To-Speech system installed, but are equipped with microphones, the ```rapp_text_to_speech_espeak``` ROS node. This module utilizes the [espeak](http://espeak.sourceforge.net/) speech synthesis library, as well as the [mbrola](http://tcts.fpms.ac.be/synthesis/mbrola.html) project for altering the speech synthesis voices.

# ROS Services

## Text to speech
Service URL: ```/rapp/rapp_text_to_speech_espeak/text_to_speech_topic```

Service type:
```bash
#Contains info about time and reference
Header header
#The text to be spoken
string text
#The audio output file
string audio_output
#Language (en, gr)
string language
---
#Possible errors
string error
``` 

It should be mentioned that there are cases where Espeak fails to produce the entirety of the given text as audio (for example if the input is a long sentence in Greek, it would probably create a shorter audio than the desired).

# Launchers

## Standard launcher

Launches the **rapp_text_to_speech_espeak** node and can be launched using
```
roslaunch rapp_text_to_speech_espeak text_to_speech_espeak.launch
```

# Web services

## URL
```localhost:9001/hop/text_to_speech ```

## Input / Output

```
Input = {
  “text”: “THE_TEXT_TO_BECOME_SPEECH”
  “language”: “THE_TEXT_LANGUAGE”
}
```
```
Output = {
  "payload": THE_AUDIO_DATA,
  "basename": "test.wav",
  "encoding": "base64",
  "error": ""
}
```

The full documentation exists [here]://github.com/rapp-project/rapp-platform/tree/master/rapp_web_services/services#text-to-speech)
