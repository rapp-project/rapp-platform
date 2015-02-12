#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/deadline_timer.hpp>

#include <rapp_server/faceDetector/faceDetector.hpp>
#include <rapp_server/qrDetector/qrDetector.hpp>
#include <rapp_server/speechDetectionGoogle/speechDetectionGoogle.hpp>

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;

