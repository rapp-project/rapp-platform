Documentation about the RAPP Speech Detection using Google API: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Speech-Detection-using-Google-API)

For the initial Speech recognition implementation, a proof of concept approach was followed, employing the Google Speech API . This API was created to enable developers to provide web-based speech to text functionalities. There are two modes (one-shot speech and streaming) and the results are returned as a list of hypotheses, along with the most dominant one. Since the aforementioned API is for developing purposes some limitations exist, such as that the input stream cannot be longer than 10-15 seconds of audio and the requests per day cannot be more than 50 if a personal Speech API Key is used. In our implementation we use the one-shot speech functionality, meaning that an audio file must be locally stored.

Another limitation of the Google Speech API is that the input audio file must be of certain format. Specifically the file must be a flac file with one channel, having a sample rate of 16kHz. In order for this service to be able to be used with any audio file that arrives as input, in case where the input file has not the desired characteristics, a system call to the flac library is performed, converting the file to the correct format. Additionally, if the audio originates from the NAO robot, the file is denoised using the respective services of the Audio Processing node, according to its specific characteristics.

# ROS Services


## Speech to text service using Google API
The Google Speech Recognition node provides a single ROS service.

Service URL: ```/rapp/rapp_speech_detection_google/speech_to_text```

Service type:
```bash
#The stored audio file
string audio_file
#The audio type [nao_ogg, nao_wav_1_ch, nao_wav_4_ch]
string audio_file_type
#The user
string user
---
#The words
string[] words
#The confidence returned by Google
float64 confidence
#A list of alternative phrases returned from Google
string[][] alternatives
#Possible error
string error
``` 

## Speech detection Google RPS

The QR detection RPS is of type 4 since it contains a HOP service frontend that contacts a ROS node wrapper, which in turn invokes an external service. The speech detection RPS can be invoked using the following URL.

Service URL: ```localhost:9001/hop/speech_detection_google ```

### Input/Output
As described, the speech detection RPS takes as input the audio file in which we desire to detect the words. The file path is encoded in JSON format in a binary string representation.

The speech detection RPS returns as output an array of words determining the dominant guess, the confidence in a probability form and the suggested alternative sentences. The encoding is in JSON format.

```
Input = {
  “file”: “THE_AUDIO_FILE”
  “audio_source”: “nao_ogg, nao_wav_1_ch, nao_wav_4_ch”
  “language”: “el”
}
```
```
Output = {
  “words”: [ “WORD_1”, “WORD_2”, … , “WORD_N” ],
  “alternatives”: [[ “WORD_1”, “WORD_2”, … , “WORD_N” ],  [....  ]],
  "error": ""
}
```
### Example
An example input for the speech detection RPS was created. The actual input was a flac file, having a size of 242.6 KB, where the “I want to use the Skype” sentence was recorded. For this specific input, the result obtained was:

For this specific input, the result obtained was
```
Output = {
  “words”: [ “I”, “want”, “to” , “use” , “Skype” ],
  “alternatives”: [
    [ “I”, “want”, “to” , “use” , “the” , “Skype” ],
    [ “I”, “want”, “to” , “use” , “Skype” ],
    [ “I”, “want”, “to” , “use” , “the” , “Skype” , “app” ]
  ],
  "error": ""
}
```

The full documentation exists [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_web_services/services#speech-detection-google)
