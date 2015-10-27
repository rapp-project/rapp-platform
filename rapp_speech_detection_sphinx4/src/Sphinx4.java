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
        System.out.println("#"+e);
        System.out.println("stopPython");
      }
    }
  } 

  public static void main(String[] args) throws IOException {

    BufferedReader bufferRead = new BufferedReader(new InputStreamReader(System.in));

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
            System.out.println("Fatal error, dictionary file does not exist");
          }
          else
          {
            dictionary_path = tmp[1];
            System.out.println("Dictionary set");
          }
        }
        // Language model file setup
        else if(tmp[0].contains("languageModel")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, language file does not exist");
          }
          else
          {
            language_path = tmp[1];
            System.out.println("Language model set");
          }
        }
        // Acoustic model folder setup
        else if(tmp[0].contains("acousticModel")){
          configuration.setAcousticModelPath(tmp[1]);
          System.out.println("Acoustic model set");
        }
        // Grammar file and folder setup
        else if(tmp[0].contains("grammarName")){
          grammar_model_file_path = tmp[1];
          grammar_model_folder_path = tmp[2];
          System.out.println("Grammar model set");
        }
        // Configuration file path set
        else if(tmp[0].contains("configurationPath")){
          test_file = new File(tmp[1]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, configuration file does not exist");
          }
          else
          {
            configuration_model_path = tmp[1];
            System.out.println("Configuration path set");
          }
        }
        // Perform forced configuration
        else if(tmp[0].contains("forceConfiguration")){
          updateConfiguration();
          System.out.println("Configuration performed");
        }
        // Enable grammar
        else if(tmp[0].contains("enableGrammar")){
          grammar_enabled = true;    
          System.out.println("Grammar enabled"); 
        }
        // Disable grammar
        else if(tmp[0].contains("disableGrammar")){
          grammar_enabled = false;
          System.out.println("Grammar disabled"); 
        }
        // Perform audio recognition
        else if (tmp[0].contains("audioInput")) {
          updateConfiguration();
          test_file = new File(tmp[1]);
          System.out.println(tmp[1]); // Check why these are needed!
          System.out.println(tmp[1]);
          if(!test_file.exists())
          {
            System.out.println("Fatal error, audio file does not exist");
          }
          else
          {
            recognizer.startRecognition(new FileInputStream(tmp[1]));
            SpeechResult result;
            while ((result = recognizer.getResult()) != null) {
              System.out.println("#" + result.getHypothesis());
            }
            recognizer.stopRecognition();
            System.out.println("stopPython");
          }
        }
      } 
      catch (IOException | RuntimeException e) {
      //catch (IOException e) {
        e.printStackTrace();
        //System.out.println("#"+e);
        System.out.println("CatchedException " + e);
        //System.out.println("stopPython");
      }
    }
  }
}
