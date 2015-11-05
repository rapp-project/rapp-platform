rm sentences.arpa
rm sentences.dict
rm sentences.idngram
rm sentences.lm.dmp
text2wfreq < sentences.txt 2> /dev/null | wfreq2vocab > sentences.dict 2> /dev/null
text2idngram -vocab sentences.dict -idngram sentences.idngram < sentences.txt &> /dev/null
idngram2lm -vocab_type 0 -idngram sentences.idngram -vocab sentences.dict -arpa sentences.arpa &> /dev/null
sphinx_lm_convert -i sentences.arpa -o sentences.lm.dmp &> /dev/null
sphinx_lm_convert -i sentences.lm.dmp -o sentences.lm.bin &> /dev/null
