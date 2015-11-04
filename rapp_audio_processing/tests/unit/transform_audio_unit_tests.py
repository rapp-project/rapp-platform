#! /usr/bin/env python

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
                '/testing_tools/test_data'
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


    def test_normalNonexistentFile(self):

        size = random.randrange( 9, 20 )
        filename = self.id_generator( size ) + "." + self.id_generator( size )
        source_name = os.path.join( self.auxiliary_files_url, filename )
        print source_name

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

        self.assertEqual( result, "Error: file " + source_name + ' not found')
        self.assertEqual( final_name, '' )

    def test_normalWrongTypes(self):
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


    def id_generator( self, size = 15, chars = string.ascii_lowercase + '_' ):
        return ''.join( random.choice( chars ) for _ in range( size ) )



