rm sentences.arpa
rm sentences.dict
rm sentences.idngram
rm sentences.lm.dmp
text2wfreq < sentences.txt | wfreq2vocab > sentences.dict
text2idngram -vocab sentences.dict -idngram sentences.idngram < sentences.txt
idngram2lm -vocab_type 0 -idngram sentences.idngram -vocab sentences.dict -arpa sentences.arpa
sphinx_lm_convert -i sentences.arpa -o sentences.lm.dmp
sphinx_lm_convert -i sentences.lm.dmp -o sentences.lm.bin
