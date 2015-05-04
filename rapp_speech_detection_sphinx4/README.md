Sample call to the configuration service:

```
rosservice call /ric/speech_detection_sphinx4_configure '{
language: en,
words: [butter, this, is, excellent],                                            
grammar: [(butter), (this), (is), (excellent)],
sentences: [butter, this, is, excellent]
}'
```
