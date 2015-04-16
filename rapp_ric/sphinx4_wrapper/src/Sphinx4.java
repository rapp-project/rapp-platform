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
  public static void main(String[] args) throws IOException {

    Configuration configuration = new Configuration();
    ConfigurationManager cm;
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
            configuration.setDictionaryPath(tmp[1]);
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
            configuration.setLanguageModelPath(tmp[1]);

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
          configuration.setUseGrammar(true);
          configuration.setGrammarName(tmp[1]); // Grammar file name
          configuration.setGrammarPath(tmp[2]); // Grammar folder url

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
            cm = new ConfigurationManager(tmp[1]);

            System.out.println("Configuration path set");
          }
        }
        // Perform audio recognition
        else if (tmp[0].contains("audioInput")) {
          test_file = new File(tmp[1]);
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
      } 
      catch (IOException e) {
        e.printStackTrace();
        System.out.println("#"+e);
        System.out.println("stopPython");
      }
    }
  }
}
