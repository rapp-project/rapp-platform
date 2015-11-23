/******************************************************************************
Copyright 2015 RAPP

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

******************************************************************************/

import edu.cmu.sphinx.api.Configuration;
import edu.cmu.sphinx.api.Context;
import edu.cmu.sphinx.api.SpeechResult;
import edu.cmu.sphinx.api.StreamSpeechRecognizer;
import edu.cmu.sphinx.linguist.flat.FlatLinguist;
import edu.cmu.sphinx.recognizer.Recognizer;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.OutputStreamWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.URL;
import java.net.URLClassLoader;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;

/*
 * Fix licence
 * Authors: Thanos Kintsakis, Manos Tsardoulias
 */
public class Sphinx4 {

  public static String dictionary_path = "";
  public static String dictionary_path_prev = "";

  public static String language_path = "";
  public static String language_path_prev = "";

  public static String acoustic_model_path = "";
  public static String acoustic_model_path_prev = "";

  public static String grammar_model_file_path = "";
  public static String grammar_model_file_path_prev = "";
  public static String grammar_model_folder_path = "";
  public static String grammar_model_folder_path_prev = "";

  public static String configuration_model_path = "";
  public static String configuration_model_path_prev = "";

  public static Configuration configuration = new Configuration();
  public static ConfigurationManager cm;
  public static StreamSpeechRecognizer recognizer;

  public static BufferedReader bufferRead;
  public static PrintWriter bufferWrite;

  public static boolean grammar_enabled = false;
  public static boolean grammar_enabled_prev = false;

  public static void updateConfiguration() throws IOException{
    boolean new_info = false;
    if(dictionary_path != dictionary_path_prev){
      dictionary_path_prev = dictionary_path;
      configuration.setDictionaryPath(dictionary_path);
      new_info = true;
    }
    if(language_path != language_path_prev){
      language_path_prev = language_path;
      configuration.setLanguageModelPath(language_path);
      new_info = true;
    }
    if(acoustic_model_path != acoustic_model_path_prev){
      acoustic_model_path_prev = acoustic_model_path;
      configuration.setAcousticModelPath(acoustic_model_path);
      new_info = true;
    }
    if(grammar_model_file_path != grammar_model_file_path_prev){
      grammar_model_file_path_prev = grammar_model_file_path;
      configuration.setGrammarName(grammar_model_file_path);
      new_info = true;
    }
    if(grammar_model_folder_path != grammar_model_folder_path_prev){
      grammar_model_folder_path_prev = grammar_model_folder_path;
      configuration.setGrammarPath(grammar_model_folder_path);
      new_info = true;
    }
    if(configuration_model_path != configuration_model_path_prev){
      configuration_model_path_prev = configuration_model_path;
      cm = new ConfigurationManager(configuration_model_path);
      new_info = true;
    }
    if(grammar_enabled != grammar_enabled_prev){
      grammar_enabled_prev = grammar_enabled;
      configuration.setUseGrammar(grammar_enabled);
      new_info = true;
    }

    if(new_info == true){
      try{
        recognizer = new StreamSpeechRecognizer(configuration);
      }
      catch (IOException e) {
        e.printStackTrace();
        bufferWrite.println("CatchedException " + e);
      }
    }
  }

  public static void main(String[] args) throws IOException {

    int socketPort;
    Socket sphinxSocket;

    if (args.length == 1)
    {
      socketPort = Integer.parseInt(args[0]);
      System.err.println("Socket port: " + socketPort);
      sphinxSocket = new Socket( "127.0.0.1", socketPort );

      bufferRead = new BufferedReader(new InputStreamReader(sphinxSocket.getInputStream()));
      bufferWrite = new PrintWriter(new BufferedWriter(new OutputStreamWriter(sphinxSocket.getOutputStream())), true);
    }
    else
    {
      System.err.println("CatchedException");
      System.exit(-1);
    }


    String[] tmp;
    File test_file;
    // Waiting for input from the Python node
    while (true) {
      try {
        configuration.setUseGrammar(false);
        String s = bufferRead.readLine();
        tmp = s.split("#");
        // Dictionary file setup
        if(tmp[0].contains("dictionary")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            bufferWrite.println("Fatal error, dictionary file does not exist");
          }
          else
          {
            dictionary_path = tmp[1];
            bufferWrite.println("Dictionary set");
          }
        }
        // Language model file setup
        else if(tmp[0].contains("languageModel")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            bufferWrite.println("Fatal error, language file does not exist");
          }
          else
          {
            language_path = tmp[1];
            bufferWrite.println("Language model set");
          }
        }
        // Acoustic model folder setup
        else if(tmp[0].contains("acousticModel")){
          configuration.setAcousticModelPath(tmp[1]);
          bufferWrite.println("Acoustic model set");
        }
        // Grammar file and folder setup
        else if(tmp[0].contains("grammarName")){
          grammar_model_file_path = tmp[1];
          grammar_model_folder_path = tmp[2];
          bufferWrite.println("Grammar model set");
        }
        // Configuration file path set
        else if(tmp[0].contains("configurationPath")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            bufferWrite.println("Fatal error, configuration file does not exist");
          }
          else
          {
            configuration_model_path = tmp[1];
            bufferWrite.println("Configuration path set");
          }
        }
        // Perform forced configuration
        else if(tmp[0].contains("forceConfiguration")){
          updateConfiguration();
          bufferWrite.println("Configuration performed");
        }

        // Enable grammar
        else if(tmp[0].contains("enableGrammar")){
          grammar_enabled = true;
          bufferWrite.println("Grammar enabled");
        }
        // Disable grammar
        else if(tmp[0].contains("disableGrammar")){
          grammar_enabled = false;
          bufferWrite.println("Grammar disabled");
        }
        // Perform audio recognition
        else if (tmp[0].contains("audioInput")) {
          updateConfiguration();
          test_file = new File(tmp[1]);
          bufferWrite.println(tmp[1]); // Check why these are needed!
          bufferWrite.println(tmp[1]);
          if(!test_file.exists())
          {
            bufferWrite.println("Fatal error, audio file does not exist");
          }
          else
          {
            recognizer.startRecognition(new FileInputStream(tmp[1]));
            SpeechResult result;
            while ((result = recognizer.getResult()) != null) {
              bufferWrite.println("#" + result.getHypothesis());
            }
            recognizer.stopRecognition();
            bufferWrite.println("stopPython");
          }
        }
      }
      catch (IOException | RuntimeException e) {
      //catch (IOException e) {
        e.printStackTrace();
        //bufferWrite.println("#"+e);
        bufferWrite.println("CatchedException " + e);
        //bufferWrite.println("stopPython");
      }
    }
  }
}
