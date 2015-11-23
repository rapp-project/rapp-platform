#! /usr/bin/env python
# -*- encode: utf-8 -*-

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Authors: Aris Thallas
# contact: aris.thallas@{iti.gr, gmail.com}


import sys
import os
import string
import random
import subprocess

import unittest
import roslib
import rospkg

roslib.load_manifest("rapp_audio_processing")

from rapp_audio_processing import TransformAudio

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        rospack = rospkg.RosPack()
        self.auxiliary_files_url = rospack.get_path("rapp_testing_tools") + \
                '/test_data'
        self.transform_audio_module = TransformAudio()

    def tearDown(self):
        self.transform_audio_module = None
        self.rospack = None

    def test_normalTrans2Flac(self):
        for root, dirs, files in os.walk( self.auxiliary_files_url ):
            for filename in files:
                source_name = os.path.join( self.auxiliary_files_url, root, filename )

                source_type = ''
                [ file_name, extention ] = os.path.splitext( filename )

                if extention == ".ogg":
                    source_type = 'nao_ogg'
                elif extention == ".wav":

                    # Get channel number via `soxi
                    command = "soxi -c " + source_name
                    proc = subprocess.Popen( command, stdout=subprocess.PIPE, shell=True)
                    ( out, err ) = proc.communicate()
                    channels = int( out )

                    source_type = 'nao_wav_' + str(channels) + '_ch'
                else:
                    continue

                target_name = \
                    os.path.join( self.auxiliary_files_url, root, file_name ) + \
                    ".flac"
                target_type = 'flac'
                target_rate = random.randrange( 16000, 65535 )
                target_channels = random.randrange( 1, 8 )

                #for _ in range(50):
                result, final_name = \
                        self.transform_audio_module.transform_audio( \
                        source_type, source_name, target_type, target_name, \
                        target_channels, target_rate )

                self.assertEqual( result, 'success' )
                self.assertTrue( os.path.isfile( target_name ) )
                os.remove( target_name )
            #os.remove( target_name ) # Check deletions inside and ouside loop


    def test_normalTransOgg2Wav(self):
        for root, dirs, files in os.walk( self.auxiliary_files_url ):
            for filename in files:
                source_name = os.path.join( self.auxiliary_files_url, root, filename )

                source_type = ''
                [ file_name, extention ] = os.path.splitext( filename )


                if extention == ".ogg":
                    source_type = 'nao_ogg'
                else:
                    continue

                target_name = \
                    os.path.join( self.auxiliary_files_url, root, file_name ) + \
                    ".wav"
                target_type = 'wav'
                target_rate = random.randrange( 16000, 65535 )
                target_channels = random.randrange( 1, 8 )

                #for _ in range(50):
                result, final_name = \
                        self.transform_audio_module.transform_audio( \
                        source_type, source_name, target_type, target_name, \
                        target_channels, target_rate )

                self.assertEqual( result, 'success' )
                self.assertTrue( os.path.isfile( target_name ) )
                os.remove( target_name )
            #os.remove( target_name ) # Check deletions inside and ouside loop


    def test_nonexistentFile(self):

        for _ in range( 50 ):
            size = random.randrange( 9, 20 )
            filename = self.id_generator( size ) + "." + self.id_generator( size )
            source_name = os.path.join( self.auxiliary_files_url, filename )

            types = [ 'nao_wav_1_ch', 'nao_ogg', 'nao_wav_4_ch' ]
            source_type = random.choice( types )

            target_name = \
                os.path.join( self.auxiliary_files_url, filename ) + \
                ".wav"
            target_type = 'wav'
            target_rate = random.randrange( 16000, 65535 )
            target_channels = random.randrange( 1, 8 )

            result, final_name = \
                    self.transform_audio_module.transform_audio( \
                    source_type, source_name, target_type, target_name, \
                    target_channels, target_rate )

            self.assertEqual( result, "Error: file \'" + source_name + '\' not found')
            self.assertEqual( final_name, '' )

    def test_wrongTypes(self):
        for root, dirs, files in os.walk( self.auxiliary_files_url ):
            for filename in files:
                source_name = os.path.join( self.auxiliary_files_url, root, filename )

                source_type = ''
                [ file_name, extention ] = os.path.splitext( filename )

                if extention == ".wav":

                    # Get channel number via `soxi
                    command = "soxi -c " + source_name
                    proc = subprocess.Popen( command, stdout=subprocess.PIPE, shell=True)
                    ( out, err ) = proc.communicate()
                    channels = int( out )

                    if channels == 4:
                        source_type = 'nao_wav_1_ch'
                    elif channels ==1:
                        source_type = 'nao_wav_4_ch'
                    else:
                        assertEqual( 'Undefined filetype', '' )

                else:
                    continue

                target_name = \
                    os.path.join( self.auxiliary_files_url, root, file_name ) + \
                    ".flac"
                target_type = 'flac'
                target_rate = random.randrange( 16000, 65535 )
                target_channels = random.randrange( 1, 8 )

                result, final_name = \
                        self.transform_audio_module.transform_audio( \
                        source_type, source_name, target_type, target_name, \
                        target_channels, target_rate )

                if channels == 4:
                    self.assertEqual( result, 'Error: wav 1 ch declared but the audio file has ' + str(channels) + ' channels' )
                elif channels == 1:
                    self.assertEqual( result, 'Error: wav 4 ch declared but the audio file has not 4 channels' )
                self.assertFalse( os.path.isfile( target_name ) )

    def test_missingSource(self):
        for _ in range( 50 ):
            source_name = ''
            filename = self.id_generator() + "." + self.id_generator()

            types = [ 'nao_wav_1_ch', 'nao_ogg', 'nao_wav_4_ch' ]
            source_type = random.choice( types )

            target_name = \
                os.path.join( self.auxiliary_files_url, filename ) + \
                ".wav"
            target_type = 'wav'
            target_rate = random.randrange( 16000, 65535 )
            target_channels = random.randrange( 1, 8 )

            result, final_name = \
                    self.transform_audio_module.transform_audio( \
                    source_type, source_name, target_type, target_name, \
                    target_channels, target_rate )

            self.assertEqual( result, "Error: file \'\' not found" )
            self.assertEqual( final_name, '' )

    def test_missingTarget(self):
        for root, dirs, files in os.walk( self.auxiliary_files_url ):
            for filename in files:
                source_name = os.path.join( self.auxiliary_files_url, root, filename )
                types = [ 'nao_wav_1_ch', 'nao_ogg', 'nao_wav_4_ch' ]
                source_type = random.choice( types )


                target_name = ''
                target_type = 'flac'
                target_rate = random.randrange( 16000, 65535 )
                target_channels = random.randrange( 1, 8 )

                result, final_name = \
                        self.transform_audio_module.transform_audio( \
                        source_type, source_name, target_type, target_name, \
                        target_channels, target_rate )

                self.assertEqual( result, "Error: target filename not provided" )
                self.assertEqual( final_name, '' )

    def test_wrongValues(self):
        for root, dirs, files in os.walk( self.auxiliary_files_url ):
            for filename in files:
                source_name = os.path.join( self.auxiliary_files_url, root, filename )
                types = [ 'nao_wav_1_ch', 'nao_ogg', 'nao_wav_4_ch' ]
                source_type = random.choice( types )

                target_name = \
                    os.path.join( self.auxiliary_files_url, root, filename ) + \
                    ".wav"
                target_type = 'flac'
                target_rate = random.randrange( 16000, 65535 )

                for _ in range( 50 ):
                    target_channels = random.randrange( 9, 15000 )
                    result, final_name = \
                            self.transform_audio_module.transform_audio( \
                            source_type, source_name, target_type, target_name, \
                            target_channels, target_rate )

                    self.assertEqual( result, "Error: target_channels can not be greater than 8" )
                    self.assertEqual( final_name, '' )

                for _ in range( 50 ):
                    target_channels = random.randrange( 1, 8 )
                    target_channels = -target_channels
                    print target_channels
                    result, final_name = \
                            self.transform_audio_module.transform_audio( \
                            source_type, source_name, target_type, target_name, \
                            target_channels, target_rate )

                    self.assertEqual( result, "Error: target_channels can not be negative" )
                    self.assertEqual( final_name, '' )
                for _ in range( 50 ):
                    target_rate = random.randrange( 16000, 65535 )
                    target_rate = - target_rate
                    result, final_name = \
                            self.transform_audio_module.transform_audio( \
                            source_type, source_name, target_type, target_name, \
                            target_channels, target_rate )

                    self.assertEqual( result, "Error: target_rate can not be negative" )
                    self.assertEqual( final_name, '' )


    def id_generator( self, size = 15, chars = string.ascii_lowercase + '_' ):
        return ''.join( random.choice( chars ) for _ in range( size ) )



