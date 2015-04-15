import edu.cmu.sphinx.api.Configuration;
import edu.cmu.sphinx.api.Context;
import edu.cmu.sphinx.api.SpeechResult;
import edu.cmu.sphinx.api.StreamSpeechRecognizer;
import edu.cmu.sphinx.linguist.flat.FlatLinguist;
import edu.cmu.sphinx.recognizer.Recognizer;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.HashMap;
import java.util.Map;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
/**
 *
 * @author thanos
 */
public class Sphinx4 {
  public static void main(String[] args) throws IOException {
    
    Configuration configuration = new Configuration();
    //String dictionaryPath;//="/home/thanos/rapp/speachRecognition/greekPack/custom.dict";
    //String languageModelPath;//="/home/thanos/rapp/speachRecognition/greekPack/sentences.lm.dmp";
    
    //File t2 = new File(languageModelPath);
    //if(!t2.exists())
    //{
      //System.out.println("Fatal error, Language model file does not exist");
    //}
    //Context ct = new Context("//",configuration);
    ConfigurationManager cm;
    //cm = new ConfigurationManager("/home/thanos/local_catkin_workspaces/catkin_ws/src/rapp-platform/rapp_ric/sphinx4_wrapper/train/default.config.xml");
    //Recognizer rec = cm.lookup("recognizer");
    //rec.allocate();

    //Map<String, String> properties = new HashMap();
    //properties=cm.getGlobalProperties();
    //System.out.println("# "+properties.get("oogProbability"));

    // Set path to acoustic model.
    //configuration.setAcousticModelPath("resource:/edu/cmu/sphinx/models/en-us/en-us");
    //configuration.setAcousticModelPath("/home/thanos/rapp/speachRecognition/cmuSphinx/sphinx4-5prealpha-src/sphinx4-data/src/main/resources/edu/cmu/sphinx/models/en-us/en-us");

    //System.out.println(configuration.getAcousticModelPath());

    //set grammar
    //configuration.setUseGrammar(true);
    //configuration.setGrammarName("hello");
    //configuration.setGrammarPath("/home/thanos/rapp/speachRecognition/greekPack/");
    // configuration.setGrammarPath("/home/thanos/local_catkin_workspaces/catkin_ws/src/rapp-platform/rapp_ric/sphinx4_wrapper/train/");
    //FlatLinguist t = new FlatLinguist();

    // Set path to dictionary.
    //configuration.setDictionaryPath("resource:/edu/cmu/sphinx/models/en-us/cmudict-en-us.dict");
    //configuration.setDictionaryPath("/home/thanos/rapp/speachRecognition/cmuSphinx/sphinx4-5prealpha-src/sphinx4-data/src/main/resources/edu/cmu/sphinx/models/en-us/cmudict-en-us.dict");
    //configuration.setDictionaryPath(dictionaryPath);
    //System.out.println(configuration.getDictionaryPath());

    // Set language model.
    //configuration.setLanguageModelPath("resource:/edu/cmu/sphinx/models/en-us/en-us.lm.dmp");
    //configuration.setLanguageModelPath("/home/thanos/rapp/speachRecognition/cmuSphinx/sphinx4-5prealpha-src/sphinx4-data/src/main/resources/edu/cmu/sphinx/models/en-us/en-us.lm.dmp");
    //configuration.setLanguageModelPath(languageModelPath);
    //System.out.println("done!");
    //System.out.println("mark01");
    /*
       LiveSpeechRecognizer recognizer = new LiveSpeechRecognizer(configuration);
       while (true) {
       recognizer.startRecognition(true);
       System.out.println("go!");
       String utterance = "######" + recognizer.getResult().getHypothesis();
       System.out.println(utterance);
       recognizer.stopRecognition();
       System.out.println("done");
       }
       */
    //try {
    BufferedReader bufferRead = new BufferedReader(new InputStreamReader(System.in));
    
    String[] tmp;
    File test_file;
    while (true) {
      try {
        configuration.setUseGrammar(false);
        String s = bufferRead.readLine();
        tmp = s.split("#");
        if(tmp[0].contains("dictionary")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, dictionary file does not exist");
          }
          else
          {
            configuration.setDictionaryPath(tmp[1]);
            System.out.println("Dictionary set");
          }
        }
        else if(tmp[0].contains("languageModel")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, language file does not exist");
          }
          else
          {
            configuration.setLanguageModelPath(tmp[1]);

            System.out.println("Language model set");
          }
        }
        else if(tmp[0].contains("acousticModel")){
          //test_file = new File(tmp[1]);
          //if(!test_file.exists())
          //{
            //System.out.println("Fatal error, acoustic file does not exist");
          //}
          //else
          {
            configuration.setAcousticModelPath(tmp[1]);
        
            System.out.println("Acoustic model set");
          }
        }
        else if(tmp[0].contains("grammarName")){
          test_file = new File(tmp[2]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, grammar file does not exist");
          }
          else
          {
            configuration.setUseGrammar(true);
            configuration.setGrammarName("hello");
            configuration.setGrammarPath("/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform-supplementary-material/rapp_sphinx4_java_libraries/greekPack/");

            System.out.println("Grammar model set");
          }
        }
        else if(tmp[0].contains("configurationPath")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, configuration file does not exist");
          }
          else
          {
            cm = new ConfigurationManager(tmp[1]);

            System.out.println("Configuration path set");
          }
        }
        else if (tmp[0].contains("audioInput")) {
          test_file = new File(tmp[1]);
          System.out.println(tmp[1]);
          System.out.println(tmp[1]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, audio file does not exist");
          }
          else
          {
            StreamSpeechRecognizer recognizer = new StreamSpeechRecognizer(configuration);
            recognizer.startRecognition(new FileInputStream(tmp[1]));
            SpeechResult result;// = recognizer.getResult();
            while ((result = recognizer.getResult()) != null) {
              System.out.println("#" + result.getHypothesis());
            }
            recognizer.stopRecognition();
            System.out.println("stopPython");
          }
        }
        //s = bufferRead.readLine();
      } 
      catch (IOException e) {
        e.printStackTrace();
        System.out.println("#"+e);
        System.out.println("stopPython");
      }
    }
    }
  }
