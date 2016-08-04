#include <opencv2/opencv.hpp>

//#include "Parameters.h"
//#include "Properties.h"
//#include "UserParameters.h"
//#include "ResultOD.h"

#include "rapp_visual_localization/VisOdom.hpp"

// Other functions in VisOdom class

::cv::Mat CVisOdom::readImage4(char* fname, int row, int col, int dir)
{
	// Read the 8-bit or 24-bit image in
	// and return a subimage of given size
int X1 = 263;
int X2 = 1015; // x dim is 752
float xc = (X2 - X1)/2 + X1;
int Y1 = 211;
int Y2 = 779; // y dim is 568
float yc = (Y2-Y1)/2 + Y1;

::cv::Mat inImg, outImg;

// Convert the indices to elements of the file name
int z = row + 9; // 1-41 is coded by 10-50
int x = col;
int d;
//%z = fix((INDZ-1)/VGrid) + 1;
//%r = rem((INDZ-1), VGrid); 
// dir is [1, 2 , 3, or 4]
switch (dir)
{
case 1: d = 0;
		break; //%'E';
case 2: d = 3; 
		break; // %'N';
case 3: d = 6;
		break; //%'W';
case 4: d = 9;
		break; //%'S';
}

char FILENM[128];
sprintf_s(FILENM, "%s/%02d_%02d_%02d.png", fname, z, x, d);

//[X, Map1] = imread(FILENM,'bmp');
inImg = ::cv::imread(FILENM);
if (inImg.data==NULL)
	throw("ReadImage4: image file can not be read");

//[ny, nx] = size(inImg(:,:,1));

::cv::getRectSubPix(inImg, ::cv::Size(X2-X1, Y2-Y1), ::cv::Point2f( xc, yc), outImg);

return outImg;
}

////////////////////////////////////////////////
void CVisOdom::ColorMaxFeatures(::cv::Mat& inImg, int my, int mx, ::cv::Mat& MVecMat)
{
	// Assumed image size
	// 752 (w) x 568 (h) px
	// Get image blocks 
	// blsize = 24;
	
	// But the current image size (for measurements vodom1) can vary
	// We need blocks in 11 rows (y) and 15 columns (x)
	int my4 = 11;
	int blsizeY = my/my4;
	int mx4 = 15;
	int blsizeX = my/mx4;
	
	// Default image block size 50 x 51
	// int blsizeX = 51;
	// int blsizeY = 50
	//int my4 = my/blsizeY; // number of block indices along Y (e.g. 11)
	//int mx4 = mx/blsizeX; // (e.g. 15)
	
	int mymx = my4 * mx4; // number of blocks (e.g. 165)

	// Version 5:RGB color space

	// Version 6/7: HSV color space instead of RGB color space
	// ::cv::Mat Obraz1 = rgb2ycbcr(inImg);
	int X1;
	int X2;
	int Y1 = 0;
	int Y2 = blsizeY;
	float xc, yc;
	int II = 0; // running block index
	
	::cv::Mat block, blockRGB;
	
	//std::vector<cv::Mat> block_split;

	// Quantize the colors to 50 (or 40) levels
	int hbins = 40; // v. 6/7
	//int hbins = 50; // v. 5
	int histSize[] = {hbins, 1};
	// values from:
	float hrangesH[] = { 0, 180 }; // v.6/7
	//float hrangesH[] = { 0, 256 };
	const float* rangesH[] = { hrangesH };
	float hranges[] = { 0, 256 };
	const float* ranges[] = { hranges };

	double minVal, maxVal;
	//::cv::Scalar meanVal, stddevVal;
	::cv::Point minLoc, maxLoc;
	
	for (int IY = 0; IY <my4; IY ++)
    {
		X1 = 0; X2 = blsizeX;
		for (int IX = 0; IX < mx4; IX++)
		{
			xc = (X2 - X1)/2 + X1;
			yc = (Y2-Y1)/2 + Y1;
			::cv::getRectSubPix(inImg, ::cv::Size(X2-X1, Y2-Y1), ::cv::Point2f( xc, yc), blockRGB);
			::cv::cvtColor(blockRGB, block, CV_RGB2HSV);
			
			::cv::Mat hist, normHist;
			// Obliczamy niezależne histogramy trzech kanałów
			int channels[] = {0};
			::cv::calcHist( &block, 1, channels, ::cv::Mat(), hist, 1, histSize, rangesH, 
				true, // the histogram is uniform
				false );
			// lokalizacja maksimum
			::cv::minMaxLoc(hist, 0, &maxVal, 0, &maxLoc);
			MVecMat.at<double>(II, 0) = maxLoc.y; //
			MVecMat.at<double>(II, 3) = maxVal; //

			channels[0] = 1;
			::cv::calcHist( &block, 1, channels, ::cv::Mat(), hist, 1, histSize, ranges, 
				true, // the histogram is uniform
				false );
			// lokalizacja maksimum
			::cv::minMaxLoc(hist, 0, &maxVal, 0, &maxLoc);
			MVecMat.at<double>(II, 1) = maxLoc.y; //
			MVecMat.at<double>(II, 4) = maxVal; //

			channels[0] = 2;
				::cv::calcHist( &block, 1, channels, ::cv::Mat(), hist, 1, histSize, ranges, 
				true, // the histogram is uniform
				false );
			// lokalizacja maksimum
			::cv::minMaxLoc(hist, 0, &maxVal, 0, &maxLoc);
			MVecMat.at<double>(II, 2) = maxLoc.y; // 
			MVecMat.at<double>(II, 5) = maxVal; 
	        
			II = II + 1;
			X1 = X1 + blsizeX;
			X2 = X2 + blsizeX;
		}
    Y1 = Y1 + blsizeY;
    Y2 = Y2 + blsizeY;
	}

}

///////////////////////////
::cv::Mat CVisOdom::readMeasImage4(char* fname, int row, int col, int dir, bool testMode)
{
	// Special case for off-line test: read the 8-bit or 24-bit image in (vodom1 set)
	// and return a subimage of variable size (the focal length of dane and vodom1 seem to be different)
	int X1 = 263;
	int X2 = 1015; // x dim is 752
	
	int Y1 = 211;
	int Y2 = 779; // y dim is 568
	

	::cv::Mat inImg, outImg;

	// Convert the indices to elements of the file name
	int z = row + 9; // 1-41 is coded by 10-50
	int x = col;
	int d;

	//%z = fix((INDZ-1)/VGrid) + 1;
	//%r = rem((INDZ-1), VGrid); 
	// dir is [1, 2 , 3, or 4]
	switch (dir)
	{
	case 1: d = 0;
			break; //%'E';
	case 2: d = 3; 
			break; // %'N';
	case 3: d = 6;
			break; //%'W';
	case 4: d = 9;
			break; //%'S';
	}

	char FILENM[128];
	sprintf_s(FILENM, "%s/%02d_%02d_%02d.png", fname, z, x, d);

	// read the image from a file (test mode) or from a buffer (on-line mode(
	if (testMode)
		inImg = ::cv::imread(FILENM);
	else
		inImg = inObservationImage();

	if (inImg.data==NULL)
	{
		printf("ReadImage4: no input image");
		return inImg; // empty result
	}

	// the following modification is needed in the test mode only
	if (testMode)
	{
		int nx = inImg.cols;
		int ny = inImg.rows;

		// The following makes the difference to readImage4():
		// The default window is:
		int X1o = 263;
		int X2o = 1015;
		int Y1o = 211;
		int Y2o = 779;

		switch (dir)
		{
		case 1: 
			// dir 00, size 1020 x 800, shift od (260, 0) (dla 10)do (0,0) (dla 50)
			X1 = cvRound((X1o - 6 * (41- row) - 10 * (10 - col)) * 1.2); 
			X2 = cvRound((X2o - 6 * (41- row) - 10 * (10 - col)) * 1.2);
			if (X1< 0)  X1 = 0;
			if (X2>= nx) X2 = nx -1;
			Y1 = 253; // Y1o * 1.2;
			Y2 = 935; // Y2o * 1.2;
			break;
        
	case 2:
		// dir 03,
        X1 = X1o - 120;
        X2 = X2o - 120;
        Y1 = 214;
        Y2 = 782;
		break;

	case 3:
		// dir 06
        X1 = cvRound((X1o - 6 * (41- row) - 8 * (10 - col)) * 1.15); 
        X2 = cvRound((X2o - 6 * (41- row) - 8 * (10 - col)) * 1.15);
        if (X1< 0) X1 = 0;
        if (X2>= nx) X2 = nx -1;
        Y1 = 244; // Y1o * 1.2;
        Y2 = 896; // Y2o * 1.2;
		break;
	case 4:
		// dir 09
        X1 = X1o - 120;
        X2 = X2o - 120;
        Y1 = 215;
        Y2 = 782;
		break;
		}
	}

	float xc = (X2 - X1)/2 + X1;
	float yc = (Y2-Y1)/2 + Y1;

	::cv::getRectSubPix(inImg, ::cv::Size(X2-X1, Y2-Y1), ::cv::Point2f( xc, yc), outImg);

	return outImg;
}


/////////////////////////////
void CVisOdom::showBelief(double *StateDistribution, int StateNum, char *text)
{
	//  Normalize to 1 
	double Dist = 0.0;
	double *State = new double[StateNum]; // copy the input data
	for (int j=0; j<StateNum; j++)
	{
		State[j] = StateDistribution[j];
		Dist += State[j];
	}
	if (Dist > 0)
		for (int j=0; j<StateNum; j++) 
			State[j] = State[j] / Dist;

	// VIS: show state distribution
	// for simplicity - present it as an image
			
	::cv::Mat BELIEF = ::cv::Mat::zeros(1, NUMX * NUMZ, CV_64FC1);
	::cv::Mat BELIEF2D = ::cv::Mat::zeros( NUMZ, NUMX, CV_64FC1);
			int k = 0;
			int state = 0;
			double maxval, totalmax = 0.0;
			for (int a=0; a<NUMZ; a++)
			{
				for (int b = 0; b< NUMX; b++)
				{
					maxval = State[state];
					for (int d=1; d<NUMD; d++)
						if (State[state+d] > maxval)
							maxval = State[state+d];

					BELIEF.at<double>(0, k) = maxval; // one maximum for all directions at location "k"
					BELIEF2D.at<double>(a, b) = maxval;

					if (maxval > totalmax) totalmax = maxval;
					k = k+1;
					state = state + NUMD;
				}
			}
			// normalize images for view
			int scaleSize = 10;
			int numzx = NUMZ * NUMX;
			double scale = 900.0/ totalmax;
			::cv::Scalar intensity;
			// present as a 1-D color histogram
			::cv::Mat beliefImg = ::cv::Mat::ones(902, numzx + 1, CV_8UC3) * 255;
			double binVal;
			int height; 
			uchar height4; 
			uchar height8; 
			for (int w=0; w<numzx; w++)
			{
				binVal = BELIEF.at<double>(0, w);
				height = int(binVal * scale);
				height4 = (uchar)(height/4);
				height8 = (uchar)(height/8);
				
				if (height > 800) intensity = ::cv::Scalar(0, 0, height4);
				else
					if (height> 500) intensity = ::cv::Scalar(height8, height4, 0);
					else intensity = ::cv::Scalar(height4, height4, height4);

				::cv::rectangle( beliefImg, ::cv::Point( w, 902 - height),
					::cv::Point( w+1, 901), intensity, CV_FILLED );
				//::cv::rectangle( beliefImg, ::cv::Point(w, 0),
				//	::cv::Point( w+1, height), ::cv::Scalar(255, 255, 255), CV_FILLED );
			}
			// the same presented as a 2D histogram
			int hS = NUMZ * scaleSize;
			int wS = NUMX * scaleSize;
			::cv::Mat belief2DImg = ::cv::Mat::ones(hS + scaleSize, wS + scaleSize, CV_8UC3) * 255;
			for( int h = 0; h < NUMZ; h++ )
				for( int s = 0; s < NUMX; s++ )
				{
					binVal = BELIEF2D.at<double>(h, s);
					height = int(binVal * scale);
					height4 = (uchar)(height/4); 
					height8 = (uchar)(height/8);
					if (height > 800) intensity = ::cv::Scalar(0, 0, height4);
					else
						if (height> 500) intensity = ::cv::Scalar(height8, height4, 0);
						else intensity = ::cv::Scalar(height4, height4, height4);
					
					::cv::rectangle( belief2DImg, ::cv::Point(s*scaleSize, (NUMZ-h-1)*scaleSize),
					::cv::Point( (s+1)*scaleSize - 1, (NUMZ-h)*scaleSize - 1), intensity, CV_FILLED );
				}
				char text1[120];
				char text2[120];
				sprintf_s(text1, "%s 1d", text);
				sprintf_s(text2, "%s 2d", text);
			::cv::namedWindow( text1, 1 );
			::cv::imshow( text1, beliefImg  );
			::cv::namedWindow( text2, 1 );
			::cv::imshow( text2, belief2DImg );

			beliefImg.release();
			belief2DImg.release();

		delete [] State;
}



void CVisOdom::featureTest(::cv::Mat* MVecMat, ::cv::Mat* MEASMATVEC, int* stateTest, int NUMBLOCKS)
{
	::cv::Mat* model = MVecMat;
	::cv::Mat* meas = MEASMATVEC;

	double scaling[] = {2.0, 1.0, 0.5};
	double* score = new double[StateNum];
	double* scoreImin = new double[StateNum];
	double* scoreImax = new double[StateNum];
	for (int i =0; i<StateNum; i++)
	{
		double Dist = 0.0;
        for (int jj = 0; jj<NUMBLOCKS; jj++) 
			for (int ll=0; ll<3; ll++) 
				Dist += /*scaling[ll] * */ abs(meas[i].at<double>(jj,ll) - model[i].at<double>(jj,ll));
		 //    Dist += model[i].at<double>(jj, 3) * abs(meas[i].at<double>(jj,ll) - model[i].at<double>(jj,ll));
		score[i] = Dist;
	}

	cout<< "RESULT model-measure distance test: ";
	int posx= 1;
	int posy = 10;
	for(int i =0; i<StateNum; i+=4)
	{ 
	  std::cout<< "state " << i << "("<< posx<<","<<posy<<", 0):" <<score[i] << ", " << score[i+1] <<", "<<score[i+2] << ", " << score[i+3] << endl;
	  
	  posx ++;
	  if (posx ==20) 
	  {
		 posx =1; posy ++;
	  }
	}

	// porównaj z pozostałymi
	for (int k=0; k<4; k++) // dla każdego kierunku z osobna
	{
	for(int i = k; i<StateNum; i+=4)
	{ 
		double mindist = 10000000000.0;
		double maxdist = score[i];

		std::cout<< "state Test: " << i << "; " <<endl;

		stateTest[i] = i;
		for(int j = k; j<StateNum; j+=4)
		{
			double Dist = 0.0;
			for (int jj = 0; jj<NUMBLOCKS; jj++) 
				for (int ll=0; ll<3; ll++)
					Dist += /*scaling[ll] * */ abs(meas[j].at<double>(jj,ll) - model[i].at<double>(jj,ll));
			
			if (Dist < mindist) 
			{ 
				mindist = Dist;
				stateTest[i] = j;
			}
			if (Dist >maxdist) 
				maxdist = Dist;
		}
			
		scoreImin[i] = mindist;
		scoreImax[i] = maxdist;
	}
	}
	
	std::cout<< "RESULT model-measure min max: ";
	posx= 1;
	posy = 10;
	for(int i =0; i<StateNum; i+=4)
	{ 
	    std::cout<< "state " << i << "("<< posx<<","<<posy<<", 0):" <<score[i] << " (" << scoreImin[i]<< ", " << scoreImax[i] << "); " << endl;
	  
	  posx ++;
	  if (posx ==20) 
	  {
		 posx =1; posy ++;
	  }
	}

	std::cout<< "RESULT state test : ";
	posx= 1;
	posy = 10;
	for(int i =0; i<StateNum; i+=4)
	{ 
	   std::cout<< "stateTest " << i << "("<< posx<<","<<posy<<", 0):" <<stateTest[i] ;
			  
	  posx ++;
	  if (posx ==20) 
	  {
		 posx =1; posy ++;
	  }
	}
	std::cout << endl;

	delete [] score;
	delete [] scoreImin;
	delete [] scoreImax;
}
