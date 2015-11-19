#include <face_detection/face_detector.h>


FaceDetector::FaceDetector(void)
{
}

cv::Mat FaceDetector::loadImage(std::string file_name)
{
  cv::Mat input_img;
  // Must check if file exists
  input_img = cv::imread(file_name);
  return input_img;
}

std::vector<cv::Rect> FaceDetector::detectFaces(const cv::Mat& input_img)
{
  std::vector<cv::Rect> front_faces, profile_faces, final_faces;
  cv::Mat grayscale_img;
  if( input_img.empty() )
  {
    return final_faces;
  }
  cv::cvtColor(input_img, grayscale_img, CV_BGR2GRAY);
  cv::equalizeHist(grayscale_img, grayscale_img);


  // Detect Front Faces
  std::string haar_file_path =
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
  front_faces = detectFaces( grayscale_img, haar_file_path );
  //std::cout << "Faces size: " << front_faces.size() << std::endl;


  // Detect Profile Faces
  haar_file_path = "/usr/share/opencv/haarcascades/haarcascade_profileface.xml";
  profile_faces = detectFaces( grayscale_img, haar_file_path );
  //std::cout << "AngleFaces size: " << profile_faces.size() << std::endl;

  // Identify unique faces
  final_faces = identifyUniqueFaces( front_faces, profile_faces );

  //std::cout << "Final size: " << front_faces.size() << std::endl << std::endl;

  return final_faces;
}

std::vector<cv::Rect> FaceDetector::detectFaces(const cv::Mat& input_img,
  const std::string& haar_path)
{
  std::vector<cv::Rect> faces, final_faces;

  // The Haar cascade classifier
  cv::CascadeClassifier face_cascade;
  // Create the classifier
  face_cascade.load(haar_path);

  face_cascade.detectMultiScale(input_img, faces, 1.1, 4);

  // If no faces were found make the algorithm less strict
  if(faces.size() == 0)
  {
    face_cascade.detectMultiScale(input_img, faces, 1.1, 3);
  }

  // Check the faces again to eliminate false positives
  for(unsigned int i = 0 ; i < faces.size() ; i++)
  {
    cv::Rect tmp_rect = faces[i];
    tmp_rect.x -= 10;
    tmp_rect.y -= 10;
    tmp_rect.width += 20;
    tmp_rect.height += 20;
    if(tmp_rect.x < 0 || tmp_rect.y < 0 ||
      (tmp_rect.x + tmp_rect.width) > input_img.size().width ||
      (tmp_rect.y + tmp_rect.height) > input_img.size().height)
    {
      continue;
    }
    cv::Mat temp_map = input_img(faces[i]);
    std::vector<cv::Rect> tmp_faces;
    face_cascade.detectMultiScale(temp_map, tmp_faces, 1.1, 3);
    if(tmp_faces.size() == 0)
    {
      continue;
    }
    final_faces.push_back(faces[i]);
  }
  return final_faces;
}

std::vector<cv::Rect> FaceDetector::identifyUniqueFaces(
      const std::vector<cv::Rect> firstFaceVector,
      const std::vector<cv::Rect> secondFaceVector)
{
  std::vector<cv::Rect> initial, unique;
  initial = firstFaceVector;

  for (unsigned int j = 0; j < secondFaceVector.size() ; j++)
  {
    bool exists = false;
    for (unsigned int i = 0; i < firstFaceVector.size() ; i++)
    {
      if ( firstFaceVector[i] == secondFaceVector[j] )
      {
        exists = true;
        break;
      }
    }
    if ( !exists )
    {
      unique.push_back( secondFaceVector[j] );
    }
  }
  std::cout << "Unique size: " << unique.size() << std::endl;

  initial.insert( initial.end(), unique.begin(), unique.end() );

  return initial;
}

std::vector<cv::Rect> FaceDetector::findFaces(std::string file_name)
{
  cv::Mat input_img;
  input_img = loadImage(file_name);
  return detectFaces(input_img);
}
