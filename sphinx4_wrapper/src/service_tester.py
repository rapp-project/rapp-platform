#!/usr/bin/env python
# -*- coding: utf-8 -*-

#MIT License (MIT)

#Copyright (c) <2014> <Rapp Project EU>

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

# Authors: Athanassios Kintsakis, Manos Tsardoulias
# contact: akintsakis@issel.ee.auth.gr, etsardou@iti.gr

import rospy
from denoiser import Denoiser

from rapp_platform_ros_communications.srv import (
  SpeechRecognitionSphinx4Srv,
  SpeechRecognitionSphinx4SrvResponse,
  SpeechRecognitionSphinx4SrvRequest,
  SpeechRecognitionSphinx4ConfigureSrv,
  SpeechRecognitionSphinx4ConfigureSrvResponse,
  SpeechRecognitionSphinx4ConfigureSrvRequest
  )

class SpeechRecognitionTester: 

  def setup_fifty_words_voc(self):
    spreq = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spreq.language = 'gr'
    spreq.words = []
    spreq.words.append(u'όχι')
    spreq.words.append(u'ναι')
    spreq.words.append(u'ίσως')
    spreq.words.append(u'δε')
    spreq.words.append(u'θυμάμαι')
    spreq.words.append(u'αρκετά')
    spreq.words.append(u'κοντά')
    spreq.words.append(u'έλα')
    spreq.words.append(u'φύγε')
    spreq.words.append(u'σήκω')
    spreq.words.append(u'κάτσε')
    spreq.words.append(u'είμαι')
    spreq.words.append(u'είσαι')
    spreq.words.append(u'σε')
    spreq.words.append(u'λένε')
    spreq.words.append(u'πώς')
    spreq.words.append(u'που')
    spreq.words.append(u'ποτήρι')
    spreq.words.append(u'φούρνος')
    spreq.words.append(u'ώρα')
    spreq.words.append(u'είναι')
    spreq.words.append(u'τι')
    spreq.words.append(u'γιατί')
    spreq.words.append(u'ρομπότ')
    spreq.words.append(u'σκάιπ')
    spreq.words.append(u'ιμέηλ')
    spreq.words.append(u'κόρη')
    spreq.words.append(u'γιός')
    spreq.words.append(u'στείλε')
    spreq.words.append(u'μη')
    spreq.words.append(u'στείλεις')
    spreq.words.append(u'θέλω')
    spreq.words.append(u'βοήθεια')
    spreq.words.append(u'γιατρός')
    spreq.words.append(u'χάπια')
    spreq.words.append(u'χάπι')
    spreq.words.append(u'το')
    spreq.words.append(u'τα')
    spreq.words.append(u'του')
    spreq.words.append(u'ξέρω')
    spreq.words.append(u'ξέρεις')
    spreq.words.append(u'ποδόσφαιρο')
    spreq.words.append(u'ποιός')
    spreq.words.append(u'είναι')
    spreq.words.append(u'τώρα')
    spreq.words.append(u'μετά')
    spreq.words.append(u'πριν')
    spreq.words.append(u'χτές')
    spreq.words.append(u'αύριο')
 
    spreq.sentences = []
    spreq.sentences.append(u'όχι δε θυμάμαι')
    spreq.sentences.append(u'ναι στείλε ιμέηλ')
    spreq.sentences.append(u'ίσως δε θυμάμαι')
    spreq.sentences.append(u'τι ώρα είναι')
    spreq.sentences.append(u'πώς σε λένε')
    spreq.sentences.append(u'που είναι το ποτήρι')
    spreq.sentences.append(u'θέλω βοήθεια')
    spreq.sentences.append(u'που είναι ο γιατρός')
    spreq.sentences.append(u'τι ώρα είναι τώρα')
    spreq.sentences.append(u'που είναι τα χάπια')
    spreq.sentences.append(u'το ξέρω')
    spreq.sentences.append(u'έλα αρκετά κοντά')
    spreq.sentences.append(u'που είναι το χάπι')
    spreq.sentences.append(u'ναι έλα')
    spreq.sentences.append(u'στείλε ιμέηλ τώρα')
    spreq.sentences.append(u'στείλε ιμέηλ αύριο')
    spreq.sentences.append(u'είσαι ρομποτ')
    spreq.sentences.append(u'που είναι ο φούρνος')
    spreq.sentences.append(u'δε θυμάμαι ποιός είσαι')
    spreq.sentences.append(u'δε θυμάμαι')
    spreq.sentences.append(u'ποιός είσαι')
    spreq.sentences.append(u'θέλω τα χάπια')

    spreq.grammar = []
    spreq.grammar.append(u'όχι')
    spreq.grammar.append(u'δε θυμάμαι')
    spreq.grammar.append(u'όχι δε θυμάμαι')
    spreq.grammar.append(u'στείλε ιμέηλ')
    spreq.grammar.append(u'τι ώρα είναι')
    spreq.grammar.append(u'τι είναι')
    spreq.grammar.append(u'που είναι')
    spreq.grammar.append(u'που είναι το ποτήρι')
    spreq.grammar.append(u'τι είσαι')
    spreq.grammar.append(u'ποιός είσαι')
    spreq.grammar.append(u'δε θυμάμαι ποιός είσαι')
    spreq.grammar.append(u'που είναι ο φούρνος')
    spreq.grammar.append(u'θέλω')
    spreq.grammar.append(u'θέλω τα χάπια')
    spreq.grammar.append(u'θέλω το χάπι')
    spreq.grammar.append(u'πώς σε λένε')
    spreq.grammar.append(u'σε λένε')
    spreq.grammar.append(u'ίσως')
    spreq.grammar.append(u'έλα κοντά')
    spreq.grammar.append(u'αρκετά κοντά')
    spreq.grammar.append(u'ξέρεις ποδόσφαιρο')
    spreq.grammar.append(u'θέλω βοήθεια')
    spreq.grammar.append(u'γιατί')
    spreq.grammar.append(u'τι είσαι')
    spreq.grammar.append(u'μη στείλεις')
    spreq.grammar.append(u'στείλε αύριο')
    spreq.grammar.append(u'είναι αύριο')
    spreq.grammar.append(u'είναι τώρα')
    spreq.grammar.append(u'το ξέρεις')
    spreq.grammar.append(u'δεν το ξέρεις')

    return spreq

  def setup_six_words_voc(self):
    spreq = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spreq.language = 'gr'
    spreq.words = []
    spreq.words.append(u'όχι')
    spreq.words.append(u'ναι')
    spreq.words.append(u'μπορεί')
    spreq.words.append(u'ίσως')
    spreq.words.append(u'δε')
    spreq.words.append(u'ξέρω')

    spreq.sentences = []
    spreq.sentences.append(u'όχι')
    spreq.sentences.append(u'ναι')
    spreq.sentences.append(u'μπορεί')
    spreq.sentences.append(u'ίσως')
    spreq.sentences.append(u'δε')
    spreq.sentences.append(u'ξέρω')
    spreq.sentences.append(u'όχι δε ξέρω')
    spreq.sentences.append(u'ναι μπορεί')
    spreq.sentences.append(u'ίσως δε ξέρω')
    spreq.sentences.append(u'δε ξέρω')
    spreq.sentences.append(u'ναι ίσως')
    spreq.sentences.append(u'μπορεί δε ξέρω')
    spreq.sentences.append(u'ίσως όχι')

    spreq.grammar = []
    spreq.grammar.append(u'(όχι)')
    spreq.grammar.append(u'(ναι)')
    spreq.grammar.append(u'(μπορεί)')
    spreq.grammar.append(u'(μπορεί δε ξέρω)')
    spreq.grammar.append(u'(δε ξέρω)')
    spreq.grammar.append(u'(ίσως όχι)')
    spreq.grammar.append(u'(ίσως ναι)')
    spreq.grammar.append(u'(ναι ίσως)')
 
    return spreq

  def setup_two_words_voc(self):
    spreq = SpeechRecognitionSphinx4ConfigureSrvRequest()
    spreq.language = 'gr'
    spreq.words = []
    spreq.words.append(u'όχι')
    spreq.words.append(u'ναι')
   
    spreq.sentences = []
    spreq.sentences.append(u'όχι')
    spreq.sentences.append(u'ναι')
  
    spreq.grammar = []
    spreq.grammar.append(u'(όχι)')
    spreq.grammar.append(u'(ναι)')
   
    return spreq

  def perform_experiment(self, file_path, int_words, grammar, experiments_number):
    reqspeak = SpeechRecognitionSphinx4SrvRequest()
    mean = 0.0

    for i in range(0, experiments_number):
      self.counter += 1
      reqspeak.path.data = self.base_path + file_path   
      res = self.conf_sp_ser(reqspeak)
      exp_res = []
      if int_words == 2:
        exp_res = self.two_words_res
      elif int_words == 6:
        exp_res = self.six_words_res
      elif int_words == 50:
        exp_res = self.fifty_words_res
      #self.overall_score[file_path + "_" + grammar + "_" + str(i)] =\
              #self.lcs_length(res.words, exp_res) / (1.0 * len(exp_res))
      mean += self.lcs_length(res.words, exp_res) / (1.0 * len(exp_res))
      self.total_mean += self.lcs_length(res.words, exp_res) / (1.0 * len(exp_res))
      print file_path + "_" + grammar
      print res
      print exp_res
      print "Score: " + str(self.lcs_length(res.words, exp_res) / (1.0 * len(exp_res)))
      print "overall up to now: " + str(self.total_mean / self.counter * 100.0) + "%"
      print "-------------------------------------------------"
    mean /= experiments_number * 1.0
    self.overall_score[file_path + "_" + grammar + "_mean"] = mean
    print ">>>>> Mean Score: " + str(self.overall_score[file_path + "_" + grammar + "_mean"])
    print "-------------------------------------------------"


  def lcs_length(self,a, b):
    table = [[0] * (len(b) + 1) for _ in xrange(len(a) + 1)]
    for i, ca in enumerate(a, 1):
      for j, cb in enumerate(b, 1):
        table[i][j] = (
          table[i - 1][j - 1] + 1 if ca == cb else
          max(table[i][j - 1], table[i - 1][j]))
    return table[-1][-1]

  def lcs(self, xstr, ystr):
    if not xstr or not ystr:
      return ""
    x, xs, y, ys = xstr[0], xstr[1:], ystr[0], ystr[1:]
    if x == y:
      return x + self.lcs(xs, ys)
    else:
      return max(self.lcs(xstr, ys), self.lcs(xs, ystr), key=len)

  def __init__(self):    
     
    configure_service = rospy.ServiceProxy(\
        '/ric/speech_detection_sphinx4_configure',\
        SpeechRecognitionSphinx4ConfigureSrv)
    print "Configuration done!\n"

    self.conf_sp_ser = rospy.ServiceProxy(\
        '/ric/speech_detection_sphinx4',\
        SpeechRecognitionSphinx4Srv)
    
    self.two_words_res = ['nai', 'oxi', 'nai']
    self.six_words_res = ['nai', 'mporei', 'de', 'ksero', 'isos', 'oxi']
    self.fifty_words_res = ['pos', 'se', 'lene', 'de', 'thumamai', 'poios', 'eisai',\
            'thelo', 'ta', 'xapia', 'pou', 'einai', 'to', 'potiri']

    self.base_path = "/home/etsardou/benchmark_recordings/"
    self.overall_score = {}

    self.total_mean = 0.0
    self.counter = 0
   
    noftests = 20

    # Fifty words, no grammar
    spreq = self.setup_fifty_words_voc()
    spreq.grammar = []
    res = configure_service(spreq)

    self.perform_experiment("denoised.wav", 50, "no_gr",noftests)
  
    # Fifty words, with grammar
    spreq = self.setup_fifty_words_voc()
    #spreq.grammar = []
    res = configure_service(spreq)

    self.perform_experiment("denoised.wav", 50, "gr",noftests)
   
    for el in self.overall_score:
      print el + " : " + str(self.overall_score[el]*100.0) + " %"
    print "Final overall: " + str(self.total_mean / self.counter * 100.0) + "%"

# Main function
if __name__ == "__main__": 
  rospy.init_node('SpeechRecognitionTester')
  SpeechRecognitionTesterNode = SpeechRecognitionTester()
  #rospy.spin()

