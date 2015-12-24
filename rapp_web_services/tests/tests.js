/***
 * Copyright 2015 RAPP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Konstantinos Panayiotou
 * Contact: klpanagi@gmail.com
 *
 */

var path = require('path');

exports.ONTOLOGY_SUBCLASSES = require( path.join(__dirname,
    'ontology_subclasses_of', 'test.js') ).TEST;

exports.ONTOLOGY_SUPERCLASSES = require( path.join(__dirname,
    'ontology_superclasses_of', 'test.js') ).TEST;

exports.ONTOLOGY_IS_SUBSUPERCLASS = require( path.join(__dirname,
    'ontology_superclasses_of', 'test.js') ).TEST;

exports.COGNITIVE_TEST_CHOOSER = require( path.join(__dirname,
    'cognitive_test_chooser', 'test.js') ).TEST;

exports.RECORD_COGNITIVE_TEST_PERFORMANCE = require( path.join(__dirname,
    'record_cognitive_test_performance', 'test.js') ).TEST;

exports.SET_NOISE_PROFILE= require( path.join(__dirname,
    'set_noise_profile', 'test.js') ).TEST;

exports.SPEECH_DETECTION_SPHINX4= require( path.join(__dirname,
    'speech_detection_sphinx4', 'test.js') ).TEST;

exports.SPEECH_DETECTION_GOOGLE= require( path.join(__dirname,
    'speech_detection_google', 'test.js') ).TEST;

exports.FACE_DETECTION= require( path.join(__dirname,
    'face_detection', 'test.js') ).TEST;

exports.QR_DETECTION= require( path.join(__dirname,
    'qr_detection', 'test.js') ).TEST;

exports.TEXT_TO_SPEECH= require( path.join(__dirname,
    'text_to_speech', 'test.js') ).TEST;

exports.AVAILABLE_SERVICES= require( path.join(__dirname,
    'available_services', 'test.js') ).TEST;
