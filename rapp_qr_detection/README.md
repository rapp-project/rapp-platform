Documentation about the RAPP QR detection: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-QR-Detection)

A QR code (Quick Response code) is a visual two dimensional matrix, firstly introduced in Japan’s automotive industry. A QR is a kind of barcode, with the exception that common barcodes are one dimensional, whereas QRs are two dimensional, thus able to contain much more information. As known, several types of data can be encoded in a barcode or QR, such as strings, numbers etc. The QR code has become a standard in the worldwide consumers’ field, as they are widely used for product tracking, item identification, time tracking, document management and general marketing. One of the main reasons behind its widespread is that efficient algorithms are developed that provide real-time QR detection and identification even from mobile phones. It should be stated that QR tags can have different densities, therefore include different amount of information.


A QR consists of square black and white patterns, arranged in a grid in the plane, which can be detected by a camera in order to perform the decoding process. Regarding the RAPP implementation, a ROS node was developed that uses the well-known ZBar library, in conjunction to OpenCV for image manipulation. 

# ROS Services

##QR detection 
Service URL: ```/rapp/rapp_qr_detection/detect_qrs```

Service type:
```bash
#Contains info about time and reference
Header header
#The image's filename to perform the detection
string imageFilename
---
#Container for detected qr positions
geometry_msgs/PointStamped[] qr_centers
string[] qr_messages
string error
``` 

# Launchers

## Standard launcher

Launches the **qr detection** node and can be launched using
```
roslaunch rapp_qr_detection qr_detection.launch
```

# Web services

## URL
```localhost:9001/hop/qr_detection ```

## Input / Output

```
Input = {
 “file”: “THE_ACTUAL_IMAGE_DATA”
}
```
```
Output = {
  “qr_centers”: [ {x: 100, y: 200} ],
  "qr_messages": ["rapp project qr sample"],
  "error": ""
}
```

The full documentation exists [here](https://github.com/rapp-project/rapp-platform/tree/master/rapp_web_services/services#qr-detection)
