#!/usr/bin/env python
# -*- encode: utf-8 -*-

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

# Authors: Aris Thallas
# contact: aris.thallas@gmail.com

import os
from scipy.io import wavfile

#from rapp_exceptions import RappError

class TransformAudio:


    def transform_audio(self, source_type, source_name, \
            target_type, target_name, target_channels, target_rate ):

        if not os.path.isfile( source_name ):
            error = "Error: file " + source_name + ' not found'
            return [ error, '' ]

        status = self.validateSourceType( source_type, source_name )
        if status != 'valid':
            return [ status, '' ]

        try:
            self.convertType( source_type, source_name, target_type, \
                    target_name, target_channels, target_rate )
        #except RappError as e:
        except IOError as e:
            error = e.value
            return [ error, '' ]

        return [ 'success', target_name ]



    def convertType(self, source_type, source_name, target_type, target_name, \
            target_channels, target_rate ):

        if target_type == 'flac':
            command = 'flac -f --channels=' + str( target_channels ) + \
                    ' --sample-rate=' + str( target_rate ) + " " + source_name + ' -o ' + \
                    target_name

            # This is not working
            if os.system( command ):
                #raise RappError( "Error: flac command malfunctioned. File path was"\
                raise IOError( "Error: flac command malfunctioned. File path was"\
                        + source_name )
        else:
            command = "sox " + source_name + " -c " + target_channels + " -r " + \
                    target_rate + " " + target_name

            if os.system( command ):
                #raise RappError( "Error: SoX malfunctioned. File path was" + \
                raise IOError( "Error: SoX malfunctioned. File path was" + \
                        source_name )


    def validateSourceType( self, source_type, name ):

        [ source_file_name, source_extention ] = os.path.splitext( name )

        if source_type == 'nao_ogg':
            if source_extention != 'ogg':
                return "Error: ogg type selected but file is of another type"
            return 'valid'

        elif source_type == "nao_wav_1_ch" or source_type == 'headset':
            if source_extention != ".wav":
                return "Error: wav type 1 channel selected but file is of another type"

            samp_freq, signal = wavfile.read( name )
            if len( signal.shape ) != 1:
                error = ("Error: wav 1 ch declared but the audio file has " +\
                str(signal.shape[1]) + ' channels')
                return error
            return 'valid'

        elif source_type == "nao_wav_4_ch":
            if source_extention != ".wav":
                return "Error: wav type 4 channels selected but file is of another type"

            samp_freq, signal = wavfile.read( name )
            if len(signal.shape) != 2 or signal.shape[1] != 4:
                return "Error: wav 4 ch declared but the audio file has not 4 channels"

            return "valid"

        else:
            return "Non valid noise audio type"
