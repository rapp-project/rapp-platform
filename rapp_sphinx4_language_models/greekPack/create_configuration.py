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

# Authors: Manos Tsardoulias
# contact: etsardou@iti.gr


import rospy
import sys
import mmap

class GreekSupport():

  def __init__(self):
    pass

  def configureLetters(self):
    self.f_base_pre = [u'π', u'τ', u'κ', u'θ', u'χ', u'σ', u'ξ', u'ψ'] 
    self.f_base = []
    for l in self.f_base_pre:
      self.f_base.append(l.encode('utf-8'))

    self.v_base_pre = [u'δ', u'γ', u'ζ', u'λ', u'ρ', u'μ', u'ν', u'α', u'ά', u'ε',\
        u'έ', u'η', u'ή', u'ι', u'ί', u'ϊ', u'ΐ', u'ο', u'ό', u'υ', u'ύ', u'ϋ'\
        u'ΰ', u'ω', u'ώ'] 
    self.v_base = []
    for l in self.v_base_pre:
      self.v_base.append(l.encode('utf-8'))

    self.capital_letters = {}
    self.capital_letters[(u'Α').encode('utf-8')] = (u'α').encode('utf-8')
    self.capital_letters[(u'Ά').encode('utf-8')] = (u'ά').encode('utf-8')
    self.capital_letters[(u'Β').encode('utf-8')] = (u'β').encode('utf-8')
    self.capital_letters[(u'Γ').encode('utf-8')] = (u'γ').encode('utf-8')
    self.capital_letters[(u'Δ').encode('utf-8')] = (u'δ').encode('utf-8')
    self.capital_letters[(u'Ε').encode('utf-8')] = (u'ε').encode('utf-8')
    self.capital_letters[(u'Έ').encode('utf-8')] = (u'έ').encode('utf-8')
    self.capital_letters[(u'Ζ').encode('utf-8')] = (u'ζ').encode('utf-8')
    self.capital_letters[(u'Η').encode('utf-8')] = (u'η').encode('utf-8')
    self.capital_letters[(u'Ή').encode('utf-8')] = (u'ή').encode('utf-8')
    self.capital_letters[(u'Θ').encode('utf-8')] = (u'θ').encode('utf-8')
    self.capital_letters[(u'Ι').encode('utf-8')] = (u'ι').encode('utf-8')
    self.capital_letters[(u'Ί').encode('utf-8')] = (u'ί').encode('utf-8')
    self.capital_letters[(u'Ϊ').encode('utf-8')] = (u'ϊ').encode('utf-8')
    self.capital_letters[(u'Κ').encode('utf-8')] = (u'κ').encode('utf-8')
    self.capital_letters[(u'Λ').encode('utf-8')] = (u'λ').encode('utf-8')
    self.capital_letters[(u'Μ').encode('utf-8')] = (u'μ').encode('utf-8')
    self.capital_letters[(u'Ν').encode('utf-8')] = (u'ν').encode('utf-8')
    self.capital_letters[(u'Ξ').encode('utf-8')] = (u'ξ').encode('utf-8')
    self.capital_letters[(u'Ο').encode('utf-8')] = (u'ο').encode('utf-8')
    self.capital_letters[(u'Ό').encode('utf-8')] = (u'ό').encode('utf-8')
    self.capital_letters[(u'Π').encode('utf-8')] = (u'π').encode('utf-8')
    self.capital_letters[(u'Ρ').encode('utf-8')] = (u'ρ').encode('utf-8')
    self.capital_letters[(u'Σ').encode('utf-8')] = (u'σ').encode('utf-8')
    self.capital_letters[(u'Τ').encode('utf-8')] = (u'τ').encode('utf-8')
    self.capital_letters[(u'Υ').encode('utf-8')] = (u'γ').encode('utf-8')
    self.capital_letters[(u'Ύ').encode('utf-8')] = (u'ύ').encode('utf-8')
    self.capital_letters[(u'Ϋ').encode('utf-8')] = (u'ϋ').encode('utf-8')
    self.capital_letters[(u'Φ').encode('utf-8')] = (u'φ').encode('utf-8')
    self.capital_letters[(u'Χ').encode('utf-8')] = (u'χ').encode('utf-8')
    self.capital_letters[(u'Ψ').encode('utf-8')] = (u'ψ').encode('utf-8')
    self.capital_letters[(u'Ω').encode('utf-8')] = (u'ω').encode('utf-8')
    self.capital_letters[(u'Ώ').encode('utf-8')] = (u'ώ').encode('utf-8')

    self.phonems = {}
    self.phonems[(u'ου').encode('utf-8')] = 'UW '
    self.phonems[(u'ού').encode('utf-8')] = 'UW '
    self.phonems[(u'μπ').encode('utf-8')] = 'B '
    self.phonems[(u'ντ').encode('utf-8')] = 'D '
    self.phonems[(u'γκ').encode('utf-8')] = 'G ' #?
    self.phonems[(u'γγ').encode('utf-8')] = 'G ' #?
    self.phonems[(u'τσ').encode('utf-8')] = 'CH ' #?
    self.phonems[(u'τζ').encode('utf-8')] = 'JH ' #?
    self.phonems[(u'σσ').encode('utf-8')] = 'S ' #?
    self.phonems[(u'κκ').encode('utf-8')] = 'K '
    
    self.two_digit_letters = {}
    self.two_digit_letters[(u'αι').encode('utf-8')] = 'EH '
    self.two_digit_letters[(u'αί').encode('utf-8')] = 'EH '
    self.two_digit_letters[(u'ει').encode('utf-8')] = 'IH '
    self.two_digit_letters[(u'εί').encode('utf-8')] = 'IH '
    self.two_digit_letters[(u'οι').encode('utf-8')] = 'IH '
    self.two_digit_letters[(u'οί').encode('utf-8')] = 'IH '
    self.two_digit_letters[(u'υι').encode('utf-8')] = 'IH '
    self.two_digit_letters[(u'υί').encode('utf-8')] = 'IH '

    self.special_two_digit_letters = []
    self.special_two_digit_letters.append((u'αυ').encode('utf-8'))
    self.special_two_digit_letters.append((u'αύ').encode('utf-8'))
    self.special_two_digit_letters.append((u'ευ').encode('utf-8'))
    self.special_two_digit_letters.append((u'εύ').encode('utf-8'))
    self.special_two_digit_letters_v = {}
    self.special_two_digit_letters_v[(u'αυ').encode('utf-8')] = (u'αβ').encode('utf-8')
    self.special_two_digit_letters_v[(u'αύ').encode('utf-8')] = (u'άβ').encode('utf-8')
    self.special_two_digit_letters_v[(u'ευ').encode('utf-8')] = (u'εβ').encode('utf-8')
    self.special_two_digit_letters_v[(u'εύ').encode('utf-8')] = (u'έβ').encode('utf-8')
    self.special_two_digit_letters_f = {}
    self.special_two_digit_letters_f[(u'αυ').encode('utf-8')] = (u'αφ').encode('utf-8')
    self.special_two_digit_letters_f[(u'αύ').encode('utf-8')] = (u'άφ').encode('utf-8')
    self.special_two_digit_letters_f[(u'ευ').encode('utf-8')] = (u'εφ').encode('utf-8')
    self.special_two_digit_letters_f[(u'εύ').encode('utf-8')] = (u'έφ').encode('utf-8')

    self.all_special_two_digit_letters = {}
    for tdl in self.special_two_digit_letters:
      for fb in self.f_base:
        self.all_special_two_digit_letters[tdl + fb] = \
            self.special_two_digit_letters_f[tdl] + fb
    for tdl in self.special_two_digit_letters:
      for vb in self.v_base:
        self.all_special_two_digit_letters[tdl + vb] = \
            self.special_two_digit_letters_v[tdl] + vb

    self.s_specific_rules = {}
    self.s_specific_rules[(u'σγ').encode('utf-8')] = 'Z W '
    self.s_specific_rules[(u'σβ').encode('utf-8')] = 'Z V '
    self.s_specific_rules[(u'σδ').encode('utf-8')] = 'Z DH '

    self.letters = {}
    self.letters[(u'α').encode('utf-8')] = 'AA ' # when AE?
    self.letters[(u'ά').encode('utf-8')] = 'AA '
    self.letters[(u'β').encode('utf-8')] = 'V '
    self.letters[(u'γ').encode('utf-8')] = 'W '
    self.letters[(u'δ').encode('utf-8')] = 'DH '
    self.letters[(u'ε').encode('utf-8')] = 'EH '
    self.letters[(u'έ').encode('utf-8')] = 'EH '
    self.letters[(u'ζ').encode('utf-8')] = 'Z '
    self.letters[(u'η').encode('utf-8')] = 'IH '
    self.letters[(u'ή').encode('utf-8')] = 'IH '
    self.letters[(u'θ').encode('utf-8')] = 'TH '
    self.letters[(u'ι').encode('utf-8')] = 'IH '
    self.letters[(u'ί').encode('utf-8')] = 'IH '
    self.letters[(u'ϊ').encode('utf-8')] = 'IH '
    self.letters[(u'ΐ').encode('utf-8')] = 'IH '
    self.letters[(u'κ').encode('utf-8')] = 'K '
    self.letters[(u'λ').encode('utf-8')] = 'L '
    self.letters[(u'μ').encode('utf-8')] = 'M '
    self.letters[(u'ν').encode('utf-8')] = 'N '
    self.letters[(u'ξ').encode('utf-8')] = 'K S '
    self.letters[(u'ο').encode('utf-8')] = 'OW '
    self.letters[(u'ό').encode('utf-8')] = 'OW '
    self.letters[(u'π').encode('utf-8')] = 'P '
    self.letters[(u'ρ').encode('utf-8')] = 'R '
    self.letters[(u'σ').encode('utf-8')] = 'S '
    self.letters[(u'τ').encode('utf-8')] = 'T '
    self.letters[(u'υ').encode('utf-8')] = 'IH '
    self.letters[(u'ύ').encode('utf-8')] = 'IH '
    self.letters[(u'ϋ').encode('utf-8')] = 'IH ' 
    self.letters[(u'ΰ').encode('utf-8')] = 'IH '
    self.letters[(u'φ').encode('utf-8')] = 'F '
    self.letters[(u'χ').encode('utf-8')] = 'HH '
    self.letters[(u'ψ').encode('utf-8')] = 'P S '
    self.letters[(u'ω').encode('utf-8')] = 'OW '
    self.letters[(u'ώ').encode('utf-8')] = 'OW '
    self.letters[(u'ς').encode('utf-8')] = 'S '

    self.literal_letters = {}
    self.literal_letters[(u'α').encode('utf-8')] = 'a' # when AE?
    self.literal_letters[(u'ά').encode('utf-8')] = 'a\''
    self.literal_letters[(u'β').encode('utf-8')] = 'b'
    self.literal_letters[(u'γ').encode('utf-8')] = 'g'
    self.literal_letters[(u'δ').encode('utf-8')] = 'd'
    self.literal_letters[(u'ε').encode('utf-8')] = 'e'
    self.literal_letters[(u'έ').encode('utf-8')] = 'e\''
    self.literal_letters[(u'ζ').encode('utf-8')] = 'z'
    self.literal_letters[(u'η').encode('utf-8')] = 'h'
    self.literal_letters[(u'ή').encode('utf-8')] = 'h\''
    self.literal_letters[(u'θ').encode('utf-8')] = 'th'
    self.literal_letters[(u'ι').encode('utf-8')] = 'i'
    self.literal_letters[(u'ί').encode('utf-8')] = 'i\''
    self.literal_letters[(u'ϊ').encode('utf-8')] = 'i:'
    self.literal_letters[(u'ΐ').encode('utf-8')] = 'i\':'
    self.literal_letters[(u'κ').encode('utf-8')] = 'k'
    self.literal_letters[(u'λ').encode('utf-8')] = 'l'
    self.literal_letters[(u'μ').encode('utf-8')] = 'm'
    self.literal_letters[(u'ν').encode('utf-8')] = 'n'
    self.literal_letters[(u'ξ').encode('utf-8')] = 'ks'
    self.literal_letters[(u'ο').encode('utf-8')] = 'o'
    self.literal_letters[(u'ό').encode('utf-8')] = 'o\''
    self.literal_letters[(u'π').encode('utf-8')] = 'p'
    self.literal_letters[(u'ρ').encode('utf-8')] = 'r'
    self.literal_letters[(u'σ').encode('utf-8')] = 's'
    self.literal_letters[(u'ς').encode('utf-8')] = 's\''
    self.literal_letters[(u'τ').encode('utf-8')] = 't'
    self.literal_letters[(u'υ').encode('utf-8')] = 'u'
    self.literal_letters[(u'ύ').encode('utf-8')] = 'u\''
    self.literal_letters[(u'ϋ').encode('utf-8')] = 'u:' 
    self.literal_letters[(u'ΰ').encode('utf-8')] = 'u\':'
    self.literal_letters[(u'φ').encode('utf-8')] = 'f'
    self.literal_letters[(u'χ').encode('utf-8')] = 'x'
    self.literal_letters[(u'ψ').encode('utf-8')] = 'ps'
    self.literal_letters[(u'ω').encode('utf-8')] = 'w'
    self.literal_letters[(u'ώ').encode('utf-8')] = 'w\''

    self.all_greek_letters = [\
            u'Α', u'Ά', u'α', u'ά',\
            u'Β', u'β',\
            u'Γ', u'γ',\
            u'Δ', u'δ',\
            u'Ε', u'Έ', u'ε', u'έ',\
            u'Ζ', u'ζ',\
            u'Η', u'Ή', u'η', u'ή',\
            u'Θ', u'θ',\
            u'I', u'Ί', u'Ϊ', u'ι', u'ί', u'ϊ', u'ΐ',\
            u'Κ', u'κ',\
            u'Λ', u'λ',\
            u'Μ', u'μ',\
            u'Ν', u'ν',\
            u'Ξ', u'ξ',\
            u'Ο', u'Ό', u'ο', u'ό',\
            u'Π', u'π',\
            u'Ρ', u'ρ',\
            u'Σ', u'σ', u'ς',\
            u'Τ', u'τ',\
            u'Υ', u'Ύ', u'Ϋ', u'υ', u'ύ', u'ϋ', u'ΰ',\
            u'Φ', u'φ',\
            u'Χ', u'χ',\
            u'Ψ', u'ψ',\
            u'Ω', u'Ώ', u'ω', u'ώ',\
            u' '\
            ]
    #tmp = []
    #for l in self.all_greek_letters:
        #tmp.append(l.encode('utf-8'))
    #self.all_greek_letters = tmp


  def transformWords(self, words):
    enhanced_words = {}
    englified_words = {}
    for word in words:
      initial_word = word
      # transform capital letters
      for cap in self.capital_letters:
        initial_word = initial_word.replace(cap, self.capital_letters[cap])
      # fix english version of letters
      eng_w = initial_word
      for lit in self.literal_letters:
        eng_w = eng_w.replace(lit, self.literal_letters[lit])
      englified_words[eng_w] = word
      # check phonems
      for ph in self.phonems:
        initial_word = initial_word.replace(ph, self.phonems[ph])
      # check special two digit letters
      for stdl in self.all_special_two_digit_letters:
        initial_word = initial_word.replace(stdl, \
            self.all_special_two_digit_letters[stdl])
      # check two-digit letters
      for let in self.two_digit_letters:
        initial_word = initial_word.replace(let, self.two_digit_letters[let])
      # check specific rules
      for sr in self.s_specific_rules:
        initial_word = initial_word.replace(sr, self.s_specific_rules[sr])
      # check the rest of the letters
      for l in self.letters:
        initial_word = initial_word.replace(l, self.letters[l])
      
      enhanced_words[eng_w] = []
      temp = initial_word.split(' ')
      if len(temp) > 0:
        temp = temp[:-1]
      enhanced_words[eng_w] = temp
    
    return [enhanced_words, englified_words]

  def englify_words(self, words):
    englified_words = []
    for word in words:
      eng_w = word
      for lit in self.literal_letters:
        eng_w = eng_w.replace(lit, self.literal_letters[lit])
      englified_words.append(eng_w)
    return englified_words

def main():

    # The Greek support helper class
    greek = GreekSupport()
    greek.configureLetters()

    # Open corpus file
    corpus_file = open("corpus.txt")
    corpus = corpus_file.read()
    corpus_file.close()

    # Split the corpus into expressions / sentences
    split_chars = [".", ",", ")", "(", "\"", "[", "]", ":"]
    sentences = [corpus]
    for c in split_chars:
        tmp = []
        for s in sentences:
           tmp += s.split(c)
        sentences = tmp

    # Erase all other words that are not Greek valid letters
    to_be_erased = []
    for s in sentences:
        for l in s.decode('utf-8'):
            if l not in greek.all_greek_letters:
                if l not in to_be_erased:
                    to_be_erased.append(l)
    tmp = []
    for s in sentences:
        if s == '':
            continue
        tmp_s = s.decode('utf-8')
        for l in to_be_erased:
            tmp_s = tmp_s.replace(l, "")
        # Transform capital letters to lower case
        tmp_w = tmp_s.encode('utf-8')
        for cl in greek.capital_letters:
            tmp_w = tmp_w.replace(cl, greek.capital_letters[cl])
        tmp.append(tmp_w)
    sentences = tmp
    initial_sentences = tmp
    # Sentences and initial sentences are unicode now

    # Break the sentences into words
    words = []
    for s in sentences:
        tmp_words = s.split(" ")
        for w in tmp_words:
            if w == "":
                continue
            if w not in words:
                words.append(w)
    # Words in utf-8 now

    # Transform words in phonemes
    [enh, engl] = greek.transformWords(words)
    
    # Open custom.dict and write the words
    custom_dict = open('custom.dict', 'w')
    for w in enh:
        custom_dict.write(w + " ")
        for ph in enh[w]:
            custom_dict.write(ph + " ")
        custom_dict.write("\n")
    custom_dict.close()

    # Create the transliteration file
    translit_file = open('transliteration.txt', 'w')
    for i in engl:
        translit_file.write(i + " " + engl[i] + "\n")
    translit_file.close()

    # Create the sentences file
    sentences_file = open('sentences.txt', 'w')
    # Create reverse translit dictionary
    reverse_translit = {}
    for tr in engl:
        reverse_translit[engl[tr]] = tr
        #print engl[tr], tr
    # Create translit sentences
    translit_sentences = [] 
    for s in initial_sentences:
        words = s.split(" ")
        new_sentence = ""
        for w in words:
            if w == '':
                continue
            a = reverse_translit[w]
            new_sentence += reverse_translit[w] + " "
        sentences_file.write("<s>" + new_sentence + "</s>\n")
    sentences_file.close()
        

if __name__ == "__main__":
    main()
