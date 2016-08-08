#include "ros/ros.h"
#include "ros/package.h"

#include "rapp_visual_localization/VisOdom.hpp"

#include "rapp_platform_ros_communications/Localize.h"

class VisualLocalizationNode {
public:
  void createMap(const std::string & map_xml) {
    method = M_COLORMAX; // typ zestawu cech obrazu
    int numImgs=3; // liczba obrazów sylwetki
    // Obiekt z parametrami definiowanymi przez użytkownika
    upa = CUserParameters(numImgs, numImgs); 
    // liczba obrazów sylwetki i tworzonych modeli sylwetki 
    
    // Krok 3: Utworzenie wizualnej mapy pomieszczenia
    int numx=19; // liczba kolumn w mapie
    int numz=41; // liczba rzędów w mapie
    int numd=4; // liczba kierunków
    int vgrid = 5; // (nieistotne)

    objVO.initMap(numx, numz, numd, vgrid); // inicjalizuj dane związane z rozmiarem mapy
    objVO.initFeatureMemory(method); // alokuj pamięć dla reprezentacji cech obrazów (w fazie tworzenia mapy)
    objVO.initFeatureScaling(method); // inicjalizuj wagi cech (potrzebne zarówno w fazie uczenia jak i testowania/pracy)
    
    
    MapXml = map_xml; // plik do zapisu tworzonej wizualnej mapy pomieszczenia
    MeasXml = "work7MeasureColorMax.xml"; // plik z przygotowaną mapą obserwacji (tryb off-line) 
    
    // Utworzenie mapy i wypisanie jej do pliku xml
    objVO.createMap(method, (char*)map_xml.c_str(), (char*)MeasXml.c_str(), 0, 0); // parameter 1 określa typ zestawu cech obrazu
    // parametr 2: nazwa pliku z istniejącą mapą
    // parametr 3: nazwa pliku z pomiarami dla wszystkich stanów (w trybie testowania/symulacji)
    // parametr 4: tworzenie mapy (1) lub pomiń to (0) bo plik xml z mapą już istnieje (0)
    // parametr 5: na potrzeby testowania/symulacji utwórz wcześniej mapę cech dla wszystkich aktualnych pomiarów (1) lub pomiń (0)  



    char ouDirHuman[] = "C:/RAPP/NAO/logHuman";
    char inDirHuman[] = "C:/RAPP/NAO/human"; // katalog obrazów sylwetki
    char *inH = &inDirHuman[0];
    char *ouH = &ouDirHuman[0];
    char HumanXml[] = "humanModel.xml"; // plik z modelem sylwetki

    // Przełączniki pomocnicze: dla wizualizacji, monitorowania i "utrwalania" wyników 
    const int VIS = 0;  // wizualizacja w oknach obrazów z wynikami częściowymi
    const int MONIT = 1; // wyświetlanie tekstu komunikatów w terminalu
    const int LOG = 0; // utrwalenie wyników - zapis wyników na dysku do plików

    // Krok 4: Utworzenie modelu aktualnej sylwetki
    objH.createModel(inH, ouH, HumanXml, 0, VIS, MONIT, LOG);
    // parametr 3: =0 : wczytaj model sylwetki z domyslnego pliku; =1: utwórz na nowo model sylwetki

    objVO.setInterchange(&(this->interchange));
  }
  
  void doWork() {
    int MAXITER = 100; // dopuszczalna maksymalna liczba kroków jednego procesu nawigacji
    int TESTNUM = 100; // zadana liczba testów nawigacji

    ::cv::Mat results;

    std::cout << "A\n";
    // Metod "work" może być podstawą wątku ale nie jest to konieczne
    // W trybie "test/symulacja": TESTNUM oznacza liczbę testów (ścieżek)
    // W trybie "praca on-line": TESTNUM będzie liczbą ostatnich pamiętanych wyników
    results = objVO.work(TESTNUM, MAXITER, method, objH, (char*)MapXml.c_str(), (char*)MeasXml.c_str(), false);
    // ostatni parametr: true = tryb off-line (testowanie w warunkach symulacji pomiarów)
    // false = tryb on-line (rzeczywista nawigacja z samolokalizacją)
    std::cout << "B\n";
  }

  bool srvLocalize(rapp_platform_ros_communications::Localize::Request  &req,
                   rapp_platform_ros_communications::Localize::Response &res) 
  {
    interchange.img = cv::imread(req.image_files[0]);

    if (interchange.img.empty()) {
      ROS_ERROR("Can't load image %s\n", req.image_files[0].c_str());
      return false;
    }

    interchange.dx = req.pose_deltas[0].x;
    interchange.dz = req.pose_deltas[0].y;
    interchange.dd = req.pose_deltas[0].theta;

    doWork();

    res.belief.push_back(interchange.belief);
    geometry_msgs::Pose2D best_pose;
    best_pose.x = interchange.best_x;
    best_pose.y = interchange.best_z;
    best_pose.theta = interchange.best_d;
    res.best_poses.push_back(best_pose);
    return true;
  }

private:
  // Obiekty potrzebne do analizy obrazów
  CVisOdom objVO; // obiekt dla wizualnej odometrii
  CHuman objH;    // obiekt dla modelowania i detekcji sylwetki
  
  MType method;   // typ zestawu cech obrazu
  // Obiekt z parametrami definiowanymi przez użytkownika
  CUserParameters upa; 
  // liczba obrazów sylwetki i tworzonych modeli sylwetki 

  std::string MapXml;
  std::string MeasXml;

  Interchange interchange;
};

/**
 * Serves the object detection ROS service callback
 * 
 * \param req [in]  The ROS service request
 * \param res [out] The ROS service response
 * 
 * \return The success status of the call
 */
/*bool service_FindObjects(rapp_platform_ros_communications::FindObjectsSrv::Request  &req,
                         rapp_platform_ros_communications::FindObjectsSrv::Response &res)
{
  res.result = detectors[req.user].findObjects(req.user, req.fname, req.limit, res.found_names, res.found_centers, res.found_scores);
  return true;
}*/

/**
 * Serves the object learning ROS service callback
 * 
 * \param req [in]  The ROS service request
 * \param res [out] The ROS service response
 * 
 * \return The success status of the call
 */
/*bool service_LearnObject(rapp_platform_ros_communications::LearnObjectSrv::Request  &req,
                         rapp_platform_ros_communications::LearnObjectSrv::Response &res)
{
  res.result = detectors[req.user].learnObject(req.user, req.fname, req.name);
  return true;
}*/

/**
 * Serves the models loading ROS service callback
 * 
 * \param req [in]  The ROS service request
 * \param res [out] The ROS service response
 * 
 * \return The success status of the call
 */
/*bool service_LoadModels(rapp_platform_ros_communications::LoadModelsSrv::Request  &req,
                         rapp_platform_ros_communications::LoadModelsSrv::Response &res)
{
  detectors[req.user].loadModels(req.user, req.names, res.result);
  return true;
}*/

/**
 * Serves the models clearing ROS service callback
 * 
 * \param req [in]  The ROS service request
 * \param res [out] The ROS service response
 * 
 * \return The success status of the call
 */
/*bool service_ClearModels(rapp_platform_ros_communications::ClearModelsSrv::Request  &req,
                         rapp_platform_ros_communications::ClearModelsSrv::Response &res)
{
  detectors[req.user].clearModels(req.user);
  return true;
}*/

/**
 * The executable's main function.
 * Creates a ROS multispinner to enable concurrent requests and creates services
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_localization_node");
  ros::NodeHandle n;

  std::string service_name;
  
  VisualLocalizationNode vl_node;
  vl_node.createMap(ros::package::getPath("rapp_visual_localization") + "/data/work7ModelColorMax.xml");
 // vl_node.doWork();

  if (!n.getParam("/rapp_visual_localization_topic", service_name))
    ROS_ERROR("rapp_visual_localization_topic not set!");
  ros::ServiceServer service = n.advertiseService(service_name, &VisualLocalizationNode::srvLocalize, &vl_node);
/*  
  if (!n.getParam("/rapp_object_learn_topic", service_name))
    ROS_ERROR("rapp_object_learn_topic not set!");
  ros::ServiceServer service2 = n.advertiseService(service_name, service_LearnObject);
  
  if (!n.getParam("/rapp_object_load_topic", service_name))
    ROS_ERROR("rapp_object_load_topic not set!");
  ros::ServiceServer service3 = n.advertiseService(service_name, service_LoadModels);
  
  if (!n.getParam("/rapp_object_clear_topic", service_name))
    ROS_ERROR("rapp_object_clear_topic not set!");
  ros::ServiceServer service4 = n.advertiseService(service_name, service_ClearModels);
  
  ROS_INFO("Ready to find objects.");*/
  
  int threads = 1;
  /*if(!n.getParam("/rapp_object_recognition_threads", threads))
  {
    ROS_WARN("Hazard detection threads param not found");
  }
  else if(threads < 0)
  {
    threads = 1;
  }*/
  
  ROS_INFO("Ready to recognize!");
  ros::MultiThreadedSpinner spinner(threads);
  spinner.spin();
  
  return 0;
}
