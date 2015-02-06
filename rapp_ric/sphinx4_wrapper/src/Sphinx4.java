import edu.cmu.sphinx.api.Configuration;
import edu.cmu.sphinx.api.SpeechResult;
import edu.cmu.sphinx.api.StreamSpeechRecognizer;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.URL;
import java.net.URLClassLoader;

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
//        int x = 5;
//
//        Configuration configuration = new Configuration();
//        // Set path to acoustic model.
//        configuration.setAcousticModelPath("resource:/edu/cmu/sphinx/models/en-us/en-us");
//        // Set path to dictionary.
//        configuration.setDictionaryPath("resource:/edu/cmu/sphinx/models/en-us/cmudict-en-us.dict");
//        // Set language model.
//        configuration.setLanguageModelPath("resource:/edu/cmu/sphinx/models/en-us/en-us.lm.dmp");
//
//        try {
//            long start_time = System.currentTimeMillis();
//            LiveSpeechRecognizer recognizer = new LiveSpeechRecognizer(configuration);
//            recognizer.startRecognition(true);
//            while(System.currentTimeMillis()-start_time<3000)
//            {
//            //StreamSpeechRecognizer recognizer = new StreamSpeechRecognizer(configuration);
//            
//            // Start recognition process pruning previously cached data.
//            
//            // Pause recognition process. It can be resumed then with startRecognition(false).
//            }
//            
//            //recognizer.
//            SpeechResult result = recognizer.getResult();
//            recognizer.stopRecognition();
//            List<WordResult> takis=result.getWords();;// = new ArrayList<WordResult>();
//            System.out.println(takis.get(0));
//            System.out.println("EEEEEEEEEEEEEEEEEEEEEEEEEEND");
//            //String takis=result.getWords();
//        } catch (IOException ex) {
//
//        }

        ClassLoader cl = ClassLoader.getSystemClassLoader();

        URL[] urls = ((URLClassLoader) cl).getURLs();

        for (URL url : urls) {
            System.out.println(url.getFile());
        }
        Configuration configuration = new Configuration();

// Set path to acoustic model.
        //configuration.setAcousticModelPath("resource:/edu/cmu/sphinx/models/en-us/en-us");
        configuration.setAcousticModelPath("/home/thanos/rapp/speachRecognition/cmuSphinx/sphinx4-5prealpha-src/sphinx4-data/src/main/resources/edu/cmu/sphinx/models/en-us/en-us");

        System.out.println(configuration.getAcousticModelPath());

//set grammar
        //  configuration.setUseGrammar(true);
        //  configuration.setGrammarName("hello");
        //  configuration.setGrammarPath("/home/thanos/rapp/speachRecognition/cmuSphinx/sphinx4-5prealpha-src/sphinx4-data/src/main/resources/edu/cmu/sphinx/models/en-us/");
// Set path to dictionary.
        //configuration.setDictionaryPath("resource:/edu/cmu/sphinx/models/en-us/cmudict-en-us.dict");
        configuration.setDictionaryPath("/home/thanos/rapp/speachRecognition/cmuSphinx/sphinx4-5prealpha-src/sphinx4-data/src/main/resources/edu/cmu/sphinx/models/en-us/cmudict-en-us.dict");
        System.out.println(configuration.getDictionaryPath());

// Set language model.
        //configuration.setLanguageModelPath("resource:/edu/cmu/sphinx/models/en-us/en-us.lm.dmp");
        configuration.setLanguageModelPath("/home/thanos/rapp/speachRecognition/cmuSphinx/sphinx4-5prealpha-src/sphinx4-data/src/main/resources/edu/cmu/sphinx/models/en-us/en-us.lm.dmp");

        System.out.println("done!");
        System.out.println("mark01");
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
        //
        StreamSpeechRecognizer recognizer = new StreamSpeechRecognizer(configuration);
        while (true) {
            try {
                String s = bufferRead.readLine();
                while (!s.contains("home")) {
                    s = bufferRead.readLine();
                }
                //s = bufferRead.readLine();
                String path = s;
                File test;
                recognizer.startRecognition(new FileInputStream(path));
                SpeechResult result;// = recognizer.getResult();
                while ((result = recognizer.getResult()) != null) {
                    System.out.println("#" + result.getHypothesis());
                }
                recognizer.stopRecognition();
                System.out.println("stopPython");
            } catch (IOException e) {
                e.printStackTrace();
                System.out.println("#"+e);
                System.out.println("stopPython");
            }
        }

    }

}
