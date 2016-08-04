ď»ż/// File VizOdo.cpp
/// Methods of class CVizOdom
/// Based on the Matlab scripts: work4.m, work4learn.m
/// Goal: implementation and testing the visual odometry algorithm
///  
/// Author: WĹ‚odzimierz Kasprzak
/// Last modification: 27-07-2016
/// %%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include <opencv2/opencv.hpp>

#include "rapp_visual_localization/Parameters.hpp"
#include "rapp_visual_localization/Properties.hpp"
#include "rapp_visual_localization/UserParameters.hpp"
#include "rapp_visual_localization/ResultOD.hpp"

#include "rapp_visual_localization/VisOdom.hpp"
#include "rapp_visual_localization/Human.hpp"

using namespace std;

/////////////////////////////////////////////////////////////
/// Function initMap
void CVisOdom::initMap(int numx, int numz, int numd, int vgrid)
{
// Number of states: coordinates of the volume-like data
	this->NUMX = numx; // number of columns
	NUMZ = numz; // number of rows
	NUMD = numd; // number of orientations % E (00), N (03), W (06), S (09)
	
	NUMXxD = NUMX * NUMD;
	StateNum = NUMX * NUMZ * NUMD;

	GRID = 0.1f; // distance [in meters] between neigbour states
	VGrid = 5; // number of states per basic grid (e.g 0.5 m), must be= VNum +1 
	
	// data declaration for feature matching
	//double mv[7]; // for M_MEANVAR and M_FFT
	MVec = ::cv::Mat::zeros(StateNum, 6, CV_64FC1); // for M_HISTSMALL
	mvMat = ::cv::Mat::zeros(6, 7, CV_64FC1); // for M_HIST, M_COLORHIST
	MEASMAT = ::cv::Mat::zeros(165, 6, CV_64FC1); // for M_COLORMAX
	
	// for matrix-like features
	MEASMATVEC = new ::cv::Mat[StateNum]; // for off-line measurements
	MVecMat = new ::cv::Mat[StateNum]; // for map

// Create the correspondence between state index and position/orientation in 2D space
	PosIndex = cv::Mat::zeros(StateNum, 3, CV_32SC1);
	int stateind = 0;
	for (int i=1; i<=NUMZ; i++) {
		for (int j=1; j<=NUMX; j++) {
			for (int k=1; k<=NUMD; k++) {
				PosIndex.at<int>(stateind,0) = i;
				PosIndex.at<int>(stateind,1) = j;
				PosIndex.at<int>(stateind,2) = k;
				stateind++;
			}
		}
	}
     
}

/// Function releaseMap
void CVisOdom::releaseMap(MType METHOD)
{

	//double mv[7]; // for M_MEANVAR and M_FFT
	//mvMat = cv::Mat::zeros(6, 7, CV_64FC1); // for M_HIST, M_COLORHIST
	
	delete [] MEASMATVEC;
	delete [] MVecMat;

}

/// Function initFeatureScaling
void CVisOdom::initFeatureScaling(MType METHOD)
{
	// init the scaling coefficients for feature elements
	// appropriate to the method used
	for (int i=0; i<122; i++) ScaleMEASURE[i] = 1.0;
	int j = 0;
	switch(METHOD)
	{
	case M_MEANVAR :
		ScaleMEASURE[0] = 3.5; //%CurrMEANVAL(1);
		ScaleMEASURE[1] = 3.5; //%CurrMEANVAL(2);
		ScaleMEASURE[2] = 3.5; //%CurrMEANVAL(3);
		ScaleMEASURE[3] = 1; //%CurrSTDVAL(1);
		ScaleMEASURE[4] = 1; //%CurrSTDVAL(2);
		ScaleMEASURE[5] = 1; //%CurrSTDVAL(3);
		break;
	case M_FFT:
		ScaleMEASURE[0] = 100.0; 
		ScaleMEASURE[1] = 5.0;
		ScaleMEASURE[2] = 5.0;
		ScaleMEASURE[3] = 5.0;
		ScaleMEASURE[4] = 5.0;
		ScaleMEASURE[5] = 5.0;
		break;
	case M_HIST:
		ScaleMEASURE[0] = 1.0; 
		ScaleMEASURE[1] = 100.0;
		ScaleMEASURE[2] = 2.0;
		ScaleMEASURE[3] = 2.0;
		ScaleMEASURE[4] = 2.0;
		ScaleMEASURE[5] = 2.0;
		break;
	case M_COLORHIST:
		j=0;
		for (int i=0; i<16; i++)
		{
			ScaleMEASURE[j+0] = 1.0; // rel. color component
			ScaleMEASURE[j+1] = 1.0; // relative color component
			ScaleMEASURE[j+2] = 5.0; // location
			ScaleMEASURE[j+3] = 5.0; // location
			ScaleMEASURE[j+4] = 5.0; // area
			ScaleMEASURE[j+5] = 5.0; // area
			ScaleMEASURE[j+6] = 0.1; // density
			j = j + 7;
		} 
		break;
   
	case M_COLOR:
		ScaleMEASURE[0] = 1.0;  
		ScaleMEASURE[1] = 1.0;  
		ScaleMEASURE[2] = 1.0; 
		break;
   
	case M_COLORMAX:
		ScaleMEASURE[0] = 1.0;  
		ScaleMEASURE[1] = 1.0;  
		ScaleMEASURE[2] = 1.0; 
		break;
		
	case M_HISTSMALL:
		ScaleMEASURE[0] = 1.0;
		ScaleMEASURE[1] = 1.0; 
		ScaleMEASURE[2] = 1.0;
		ScaleMEASURE[3] = 100.0;
		ScaleMEASURE[4] = 100.0;
		ScaleMEASURE[5] = 100.0;
	

  /*
   %
   % Additionally: rescale the histogram intervals
   % TEMPORARY COMMENTED OUT
   %PixelCodes(1:256,1:2) = 0; % for rescaling of pixels
   %PixelInterv(1:256) = 0; % for length of intervals
   %PixelCodes(256,2) = 256;
   %for (i=1:1:256)
   %   Val = HistScaleTheMean(i-1, CurrMEANVAL(2), ScaleMEAN(2), ScaleCOEFF(2));
   %   PixelCodes(i,1) = (i-1) * Val;
   %   PixelCodes(i,2) = i * Val;
   %   if (PixelCodes(i,1) < 0) 
   %      PixelCodes(i,1) = 0.0;
   %   end
   %   if (PixelCodes(i,2) > 256)
   %      PixelCodes(i,2) = 256;
   %   end 
   %   PixelInterv(i) = PixelCodes(i,2) - PixelCodes(i,1);
   %   if (PixelInterv < 0.1) % in order to avoid small interval data
   %      PixelInterv(i) = 0.1;
   %      PixelCodes(i,1) = PixelCodes(i,2) - 0.1;
   %   end
   %   if (PixelCodes(i,1) < 0)
   %      PixelCodes(i,1) = 0;
   %   end 
   %end
   */
	break;
	default:
		break;
	}
}

/// Function initFeatureMemory
void CVisOdom::initFeatureMemory(MType METHOD)
{
	// allocate memory for feature representation and scaling
	// appropriate to the method used
	for (int i=0; i<122; i++) ScaleMEASURE[i] = 1.0;
	
	switch (METHOD)
	{
	case M_MEANVAR:
	case M_FFT:
	case M_HIST:
		MVec = ::cv::Mat::zeros(StateNum, 42, CV_64FC1); // Basic feature vector for every state
		for (int i=0; i<StateNum; i++)
		{ 
			MVecMat[i] = ::cv::Mat::zeros(16,7, CV_64FC1); // A feature array for every state
		}
		break;
	
	case M_COLORHIST:
		MVec = ::cv::Mat::zeros(StateNum, 42, CV_64FC1); // Basic feature vector for every state
		for (int i=0; i<StateNum; i++)
		{
			MEASMATVEC[i] = ::cv::Mat::zeros(16,7, CV_64FC1); // For off-line measurements: get all features at once
			MVecMat[i] = ::cv::Mat::zeros(16,7, CV_64FC1);
		}
		//MEASMVec = ::cv::Mat::zeros(StateNum, 112, CV_64FC1); // For off-line measurements
	    break;

	case M_COLOR:
		for (int i=0; i<StateNum; i++)
		{
			MEASMATVEC[i] = ::cv::Mat::zeros(1645, 3, CV_64FC1); // For off-line measurements: get all features at once
			MVecMat[i] = ::cv::Mat::zeros(1645, 3, CV_64FC1);
		}
		break;

	case M_COLORMAX:
		// for blocks 24x24
		// NUMBLOCKS = 713
		// MEASMATVEC(1:713, 1:3, StateNum) = 0; % For effective computation: get all features at once
		// MVecMat(1:713, 1:3, StateNum) = 0; 
		// for blocks 50x50
		NUMBLOCKS = 165; 
		for (int i=0; i<StateNum; i++)
		{
			MEASMATVEC[i] = ::cv::Mat::zeros(165, 6, CV_64FC1); // For off-line measurements: get all features at once
			MVecMat[i] = ::cv::Mat::zeros(165, 6, CV_64FC1);
		}
		break;

	case M_HISTSMALL:
   		MVec = ::cv::Mat::zeros(StateNum, 42, CV_64FC1); // Basic feature vector for every state
		for (int i=0; i<StateNum; i++)
		{ 
			MVecMat[i] = ::cv::Mat::zeros(16,7, CV_64FC1); // A feature array for every state
		}
		break;

	default:
		break;
	}
}

///////////////////////////////////////////////////////////
/// Function createMap
int CVisOdom::createMap(MType METHOD, char* mapXml, char* measXml, int mapSwitch, int measSwitch)
{
	// The modelling phase for different methods (image features)
	// Last modification: 20-04-2016
	
	// OLD feature sets (CAIP_2005)
	// METHOD = 'MEANVAR';
	// METHOD = 'FFT';
	// METHOD = 'HISTSMALL';
	// METHOD = 'HIST';
	// New feature sets (in RAPP)
	// METHOD = 'COLORHIST';
	// METHOD = 'COLOR';
	// METHOD = 'COLORMAX';
	
	// Generate model/measurment map
	// - mapSwitch =1 : generate model map
	// - measSwitch =1 : generate measurement map (off-line measurement)

	// In simulated (off-line) mode:
	// FILENM = 'dane1'; // model data - image folder
	// FILENMACT = 'vodom'; // test data - actual images
	// OR
	// in real-time mode:
	// FILENM = "vodom"; // model data
	// no FILENMACT: on-line image acquisition
	
	// createMap()) follows initMap() - the volume size is already set
	// NUMX = 19; % no of columns EW
	// NUMZ = 41; % no of rows NS
	// NUMDIR = 4; % E (code 00), N (03), W (06), S (09)
	// potentially 12 directions are possible: code 00 - code 11
	// GRID = 0.1; % distance [in meters] between neigbour states
	// VGrid = 5; % number of states per basic grid (e.g 0.5 m), must be= VNum +1 

	if ((mapSwitch !=1) && (measSwitch !=1) ) // no map is generated
		return 0;

	//Initialize global scaling factors
	double MEANVAL[] = {1.0, 1.0, 1.0};
	double STDVAL[] = {1.0, 1.0, 1.0};
	double MAXVAL[] = {1.0, 1.0, 1.0};
	
	// STATE grid 
	int my = this->NUMZ;
	int mx = this->NUMX;
	int md = this->NUMD;
	// StateNum = mx * my * md;
	
	// Main loop for feature generation
	
	double i=0.0; // localization [in meters]
	int NrStanu = -1;
	::cv::Mat Obraz, ObrazM;
	int nx, ny, nyxnx;
	for (int k = 1; k <= my; k++)  // for rows, e.g. from 1 to 41: coded= from 10 to 50
	{   
		double j = 0.0; 
		for (int l=1; l<=mx; l++) // for columns, e.g. from 1 to 19
		{
			for (int dir= 1; dir <= md; dir++) // for directions, e.g. from 1 to 4
			{
				NrStanu = NrStanu + 1;
				//PosIndex(NrStanu,:) = [k l dir]; // for a fast access 
				if (mapSwitch == 1)
				{
					Obraz = readImage4(this->sInMap, k, l, dir); // Get image in state [k,l,dir]
					
				}
				if (measSwitch == 1)
				{
					//Get the off-line measurement (test image)
					ObrazM = readMeasImage4(this->sInTest, k, l, dir); // Get test image in state [k,l,dir]
					
				}

				switch (METHOD)
				{
				case M_MEANVAR:
		           /* SObraz = double(reshape(Obraz(:,:,1), 1, nyxnx ));
                   MVec(NrStanu,1) = mean(SObraz); % First feature = mean of R
                   MVec(NrStanu,4) = std(SObraz); % 4-th feature = std deviation of R
                   SObraz = double(reshape(Obraz(:,:,2), 1, nyxnx));
                   MVec(NrStanu,2) = mean(SObraz); % 2-nd feature = mean of G
                   MVec(NrStanu,5) = std(SObraz); % 5-th feature = std deviation of G
                   
                   SObraz = double(reshape(Obraz(:,:,3), 1, nyxnx));
                   MVec(NrStanu,3) = mean(SObraz); % 3-th feature = mean of B
                   MVec(NrStanu,6) = std(SObraz); % 6-th feature = std deviation of B
				   */
					break;

				case M_FFT:
					/*
                   SObraz = double(Obraz(:,:,1)); % Only one colour image
                   FFTVEC = fft2(SObraz);
                   MVec(NrStanu, 1:3) = abs(FFTVEC(1,1:3));%/abs(FFTVEC(1,1)); % 3 coefficients
                   MVec(NrStanu, 4:6) = abs(FFTVEC(1,4:6));%/abs(FFTVEC(1,1)); % 3 coefficients
				   */
					break;
				case M_HIST:
					/*
                   SObraz = double(Obraz(:,:,1));
                   MVecMat(1:6,1:6, NrStanu) = HistogramFeatures(SObraz, ny, nx);
                   MVec(NrStanu, 1:6) = MVecMat(1, 1:6, NrStanu);
				   */
					break;
				case M_HISTSMALL:
					/*
                   SObraz = double(Obraz(:,:,1));
                   MVec(NrStanu, 1:6) = HistogramSmallFeat(SObraz, ny, nx, 0.5);      
				   */
					break;
				case M_COLORHIST:
					/*
                   SObraz = double(Obraz);
                   MVecMat(1:16,1:7, NrStanu) = ColorHistogramFeatures(SObraz, ny, nx); % Learn the feature vector
                   % FOR p-val evaluation purpose use the vector MVec:
                   kk = 1;
                   for (ii=1:1:6)
                       for (jj=1:1:7)
                           MVec(NrStanu, kk) = MVecMat(ii, jj, NrStanu); 
                           % Information about color, its density, area and location
                           kk = kk+1;
                       end
                   end
                   %Already get the slightly distorted measurement (test
                   %image)
                   Obraz = ReadMeasImage4(FILENMACT, k, l, dir, VGrid); % Get testimage in state [k,l,jl]
                   SObraz = 0.9 * double(Obraz); % modify intensity
                   MEASMATVEC(1:16, 1:7, NrStanu) = ColorHistogramFeatures(SObraz, ny, nx); 
                   % Simulate the measured features
                   % FOR p-val evaluation purpose use the vector MEASMVec:
                   %kk = 1;
                   %for (ii=1:1:6)
                    %   for (jj=1:1:7)
                     %      MEASMVec(NrStanu, kk) = MEASMATVEC(ii, jj, NrStanu); 
                           % Only the best information about color and density
                      %     kk = kk+1;
                      % end
                   % end
                   */
					break;
                 
				case M_COLOR:
					/*
                   SObraz = double(Obraz);
                   MVecMat(1:1645,1:3, NrStanu) = ColorFeatures(SObraz, ny, nx); % Learn the feature vector
                   
                   %Already get the slightly distorted measurement (test
                   %image)
                   Obraz = ReadMeasImage4(FILENMACT, k, l, dir, VGrid); % Get testimage in state [k,l,jl]
                   SObraz = double(Obraz); % 
                   MEASMATVEC(1:1645, 1:3, NrStanu) = ColorFeatures(SObraz, ny, nx); 
                   % Simulate the measured features
				   */
					break;
                  
				case M_COLORMAX:
                   if (mapSwitch == 1)
				   { 
					   ny = Obraz.rows; nx = Obraz.cols; 
					   nyxnx = ny * nx;
					   // MVecMat(1:NUMBLOCKS,1:3, NrStanu)
					   ColorMaxFeatures(Obraz, ny, nx, MVecMat[NrStanu]); //> Compute feature vector
				   }
				   if (measSwitch == 1)
				   { 
					   ny = ObrazM.rows; nx = ObrazM.cols; 
					   nyxnx = ny * nx;
					   // MVecMat(1:NUMBLOCKS,1:3, NrStanu)
					   ColorMaxFeatures(ObrazM, ny, nx, MEASMATVEC[NrStanu]); //> Compute feature vector
				   }
				   break;

				default:
					break;
				}
			}
			j = j + GRID;
			
		}
		i = i + GRID;
		if (this->monitorSwitch ==  1)
				std::cout<<"createMap: "<< NrStanu << endl; 
	}

//save('work5Dane1ModelColorMax.mat', 'MVecMat');
//save('work6MeasureColorMax.mat', 'MEASMATVEC');

	// Write the model data out
	::cv::FileStorage fmodel, fmeasure;
	switch(METHOD)
	{
	case M_COLORHIST:
		//for (int i=0; i<StateNum; i++)
		//{
		//	MEASMATVEC[i] = ::cv::Mat::zeros(16,7, CV_64FC1); 
		//	MVecMat[i] = ::cv::Mat::zeros(16,7, CV_64FC1);
		//}
		break;

	case M_COLOR:
		//for (int i=0; i<StateNum; i++)
		//{
		//	MEASMATVEC[i] = ::cv::Mat::zeros(16, 3, CV_64FC1); 
		//	MVecMat[i] = ::cv::Mat::zeros(16,3, CV_64FC1);
		//}
		fmodel = ::cv::FileStorage("workModel.xml", ::cv::FileStorage::WRITE);
		if (!fmodel.isOpened())
		{
			printf("Can not open file workModel.xml\n");
			return 0;
		}

		fmeasure = ::cv::FileStorage("workMeasure.xml", ::cv::FileStorage::WRITE);
		if (!fmeasure.isOpened())
		{
			printf("Can not open file workMeasure.xml\n");
			return 0;
		}
		for (int i=0; i<StateNum; i++)
		{
			fmeasure << "MEASMATVEC" << MEASMATVEC[i]; 
			fmodel << "MVecMat" << MVecMat[i]; 
		}
		// load workModel.mat; // model
		// load workMeasure.mat; // pomiary
		fmodel.release();
		fmeasure.release();
		break;

	case M_COLORMAX:
		NUMBLOCKS = 165; // for image blocks of size 50x50
		
		//for (int i=0; i<StateNum; i++)
		//{
		//	MEASMATVEC[i] = ::cv::Mat::zeros(NUMBLOCKS,3, CV_64FC1); 
		//	MVecMat[i] = ::cv::Mat::zeros(NUMBLOCKS,3, CV_64FC1);
		//}
		
		//load work5Dane1ModelColorMax.mat; // model
		//%load work4ModelColorMax.mat;
		//%load work5MeasureColorMax.mat;
		//load work6MeasureColorMax.mat; // skorygowane pomiary
		if (mapSwitch == 1)
		{
			fmodel = ::cv::FileStorage(mapXml, ::cv::FileStorage::WRITE);
			if (!fmodel.isOpened())
			{
				printf("Can not open file <mapXml>\n");
				return 0;
			}
			fmodel << "MVecMat"<< "[" ;
			for (int i=0; i<StateNum; i++)
			{
				fmodel << MVecMat[i]; 
			}
			fmodel << "]" ;

			fmodel.release();
		}


		if (measSwitch == 1)
		{
			fmeasure = ::cv::FileStorage(measXml, ::cv::FileStorage::WRITE);
			if (!fmeasure.isOpened())
			{
				printf("Can not open file <measXml>\n");
				return 0;
			}
			fmeasure << "MEASMATVEC"<< "[" ;
			for (int i=0; i<StateNum; i++)
			{
				fmeasure << MEASMATVEC[i]; 
			}
			fmeasure << "]" ;
			fmeasure.release();
			
		}
		break;
	
	default:
		break;
	}

	return 1;
}

//////////////////////////////////////////////////////
/// Function "work"
::cv::Mat CVisOdom::work(int MAXTESTNUM, int MAXITERATION, MType METHOD, CHuman& objH, 
						 char* mapXml, char* measXml, bool testMode)
{

	int* stateTest = new int[StateNum];

// 1) Read the model data in
	::cv::FileStorage fmodel, fmeasure;
	::cv::FileNode n1, n2;
	::cv::FileNodeIterator it1, it1_end, it2, it2_end;

	switch(METHOD)
	{
	case M_COLORHIST:
		//for (int i=0; i<StateNum; i++)
		//{
		//	MEASMATVEC[i] = ::cv::Mat::zeros(16,7, CV_64FC1); 
		//	MVecMat[i] = ::cv::Mat::zeros(16,7, CV_64FC1);
		//}
		break;

	case M_COLOR:
		
		fmodel = ::cv::FileStorage("workModel.xml", ::cv::FileStorage::READ);
		if (!fmodel.isOpened())
		{
			printf("Can not open file workModel.xml\n");
			return mvMat;
		}
		// read the data sequences from files
		n1 = fmodel["MVecMat"];
		if (n1.type() != ::cv::FileNode::SEQ)
		{
			cerr << "MVecMat is not a sequence! FAIL" << endl;
			return mvMat;
		}
		if (this->monitorSwitch == 1) 
			std::cout << "VisOdom: reading model and measurements" << endl;
   
		it1 = n1.begin(), it1_end = n1.end();
		//for (; it != it_end; ++it)
		//OR
		for (int i=0; i<StateNum; i++)
		{
			*it1 >> MVecMat[i];
			it1++;
		}
		fmodel.release();

		if (testMode)
		{
		fmeasure = ::cv::FileStorage("workMeasure.xml", ::cv::FileStorage::READ);
		if (!fmeasure.isOpened())
		{
			printf("Can not open file workMeasure.xml\n");
			return mvMat;
		}
		
		n2 = fmeasure["MEASMATVEC"];		
		if (n2.type() != ::cv::FileNode::SEQ)
		{
			cerr << "MEASMATVEC is not a sequence! FAIL" << endl;
			return mvMat;
		}
		it2 = n2.begin(), it2_end = n2.end();
		//for (; it != it_end; ++it)
		//OR
		for (int i=0; i<StateNum; i++)
		{
			*it2 >> MEASMATVEC[i];
			it2++;
		}
		fmeasure.release();
		}
		break;

	case M_COLORMAX:
		NUMBLOCKS = 165; // for image blocks of size 50x50
		//for (int i=0; i<StateNum; i++)
		//{
		//	MEASMATVEC[i] = ::cv::Mat::zeros(NUMBLOCKS,3, CV_64FC1); 
		//	MVecMat[i] = ::cv::Mat::zeros(NUMBLOCKS,3, CV_64FC1);
		//}
		
		//load work5Dane1ModelColorMax.mat; // model
		//%load work4ModelColorMax.mat;
		//%load work5MeasureColorMax.mat;
		//load work6MeasureColorMax.mat; // skorygowane pomiary
		fmodel = ::cv::FileStorage(mapXml, ::cv::FileStorage::READ);
		if (!fmodel.isOpened())
		{
			printf("Can not open file <mapXml>\n");
			return mvMat;
		}
		n1 = fmodel["MVecMat"];
		if (n1.type() != ::cv::FileNode::SEQ)
		{
			cerr << "MVecMat is not a sequence! FAIL" << endl;
			return mvMat;
		}
		
		if (this->monitorSwitch == 1) 
			std::cout << "VisOdom: reading model and measurements" << endl;
   
		it1 = n1.begin(), it1_end = n1.end();
		//for (; it != it_end; ++it)
		//OR
		for (int i=0; i<StateNum; i++)
		{
			*it1 >> MVecMat[i];
			it1++;
		}
		fmodel.release();

		if (testMode)
		{
		fmeasure = ::cv::FileStorage(measXml, ::cv::FileStorage::READ);
		if (!fmeasure.isOpened())
		{
			printf("Can not open file <measXml>\n");
			return mvMat;
		}
		
		n2 = fmeasure["MEASMATVEC"];

		if (n2.type() != ::cv::FileNode::SEQ)
		{
			cerr << "MEASMATVEC is not a sequence! FAIL" << endl;
			return mvMat;
		}

		it2 = n2.begin(), it2_end = n2.end();
		//for (; it != it_end; ++it)
		//OR
		for (int i=0; i<StateNum; i++)
		{
			*it2 >> MEASMATVEC[i];
			it2++;
		}
		fmeasure.release();
		}
 
		// porĂłwnaj elementy modelu z odpowiadajacymi elementami zbioru obserwacji (pomiarĂłw)
		//if (this->visualSwitch==1)
			//featureTest(MVecMat, MEASMATVEC, stateTest, NUMBLOCKS);
		break;

	default:
		break;
	}
	
	char FILENM[100];
	sprintf_s(FILENM, "%s", this->sInMap);  // 'dane1/'; training image folder

	char FILENMACT[100];
	sprintf_s(FILENMACT, "%s", this->sInTest); // = 'vodom/'; test image folder


// Parameters of the feature-space metrics
	double ScaleMEASURE[7];
	ScaleMEASURE[0] = 2.0; // relative color component H
	ScaleMEASURE[1] = 1.0; // relative color component S
	ScaleMEASURE[2] = 0.5; // color V // location center Y
	ScaleMEASURE[3] = 5.0; // location of center X
	ScaleMEASURE[4] = 5.0; // enlargement along X
	ScaleMEASURE[5] = 5.0; // enlargement along Y
	ScaleMEASURE[6] = 0.1; // density

// Normalization factors
	double ScaleMEAN[] = {1.0, 1.0, 1.0};
	double ScaleMAX[] = {1.0, 1.0, 1.0};
	double ScaleCOEFF[] = {1.0, 1.0, 1.0};

	
// Initialize the state simulation data
// Set the first goal and first real location state
	int GoalX = NUMX - 3;
	int GoalY = NUMZ;
	int GoalD = NUMD;
	int InitPosX = 1; // it represents the column index <1, 19>
	int InitPosY = 1; // it represents the row index <1, 41>
	int InitPosD = 1; // it represents the direction index <1, 4>

	int CurrX = (InitPosX - 1) * 50 + 3; // real position in pixels
	int CurrY = (InitPosY - 1) * 50 + 3; // real position in pixels
// Remark: the real positions are shifted a little against the learned positions
	
	::cv::Mat TestGoal;
	::cv::Mat TestInitPos; 
	if (testMode)
	{
// Set in advance the initial data for all the test runs 
		TestGoal = cv::Mat::zeros(MAXTESTNUM, 3, CV_32SC1);
		TestInitPos = cv::Mat::zeros(MAXTESTNUM, 3, CV_32SC1);

	for (int i=0; i< MAXTESTNUM; i++)
	{
		// Generate goal positions
		GoalX = ( (GoalX - 1) % NUMX) - 1;
		GoalY = ( (GoalY - 2) % NUMZ) - 2;
		GoalD = ( (GoalD + 3) % NUMD);
		if (GoalD == 0)  GoalD = NUMD;
		
		// avoiding the border columns
		if (GoalX < 3) GoalX = 3;
		if (GoalX > (NUMX - 2) ) GoalX = NUMX - 2;
		// avoiding negative Y
		if (GoalY < 1)  GoalY = 1 - GoalY;
		
		// set the goal positions
		TestGoal.at<int>(i,0) = GoalX;
		TestGoal.at<int>(i,1) = GoalY;
		TestGoal.at<int>(i,2) = GoalD;
		
		// generate init positions
		InitPosX = ((InitPosX + 2) % NUMX) + 1;
		InitPosY = ((InitPosY + 1) % NUMZ) + 1;
		InitPosD = ((InitPosD + 1) % NUMD);
		if (InitPosD == 0)  InitPosD = NUMD;
		if (InitPosX < 3)  InitPosX = 3;
	    if (InitPosX > (NUMX -2)) InitPosX = NUMX -2;
		// avoiding negative Y
		if (InitPosY < 1)  InitPosY = 1 - InitPosY;

		// set the initial positions
		TestInitPos.at<int>(i,0) = InitPosX;
		TestInitPos.at<int>(i,1) = InitPosY;
		TestInitPos.at<int>(i,2) = InitPosD;
	}
	} // end testMode
   

///////
// Discrete visual odometry 
// 2) Set the prior transition probability
	int ksize = NUMX+NUMZ+4*NUMD;
	::cv::Mat PNormXAll = ::cv::getGaussianKernel(2*ksize + 1, 1.0, CV_64F) * 2.5;
	// tablica PNormX jest rozmiaru (2*ksize+1) x 1; 
	// The upper half will be accessed only:
	::cv::Mat PNormX = PNormXAll.rowRange(ksize, 2 * ksize); 
	// Print normal distribution
		if (this->monitorSwitch == 1)
		{
			std::cout<< "Normal distribution: ";
			for(int i = 0; i<10; i++)
			{ 
				std::cout<< i<<": "<< PNormX.at<double>(i,0) << "; ";
			}
			std::cout << endl;
		 }
// Densities according to possible X differences
// PNormX = [0.3989    0.2420    0.0540    0.0044    0.0001    0.0000   0.0000 ... ]
// PNormX = 2.5 * PNormX; % increase the probability values
//PNormY = pdf('Normal',0:(NUMZ -1),0,0.5);% Densities according to possible Y differences
//PNormY = 2 * PNormY;
//PNormD = pdf('Normal',0:(NUMD -1),0,0.5);% Densities according to possible Y differences
//PNormD = 2 * PNormD;

	// PSunderS - the prior transition probability matrix
	int stateD, stateX, stateY;
	int predD, predX, predY;
	int DeltaD, DeltaX, DeltaY;
	int RealDeltaD, RealDeltaX, RealDeltaY;
	::cv::Mat PSunderS = cv::Mat::zeros(StateNum, StateNum, CV_64FC1); // p(s_k |s_k-1)
	for (int i=0; i< StateNum; i++)
	{    
		stateD = PosIndex.at<int>(i,2); 
		stateX = PosIndex.at<int>(i,1);
		stateY = PosIndex.at<int>(i,0);
		
		for (int j=0; j<StateNum; j++)
		{
			predD = PosIndex.at<int>(j,2);
			predX = PosIndex.at<int>(j,1);
			predY = PosIndex.at<int>(j,0);
			DeltaX = abs(stateX - predX);
			DeltaY = abs(stateY - predY);
			DeltaD = abs(stateD - predD);
			PSunderS.at<double>(i,j) = PNormX.at<double>(DeltaX+DeltaY+4*DeltaD, 0);
		}
	}

////////////////
// The testing LOOP
	// memory allocation for the returned result - basically needed in the test mode (off-line) -
	// in the work mode (on-line) the last MAXTESTNUM runs will be stored
	::cv::Mat testResult =  cv::Mat::zeros(4, MAXTESTNUM, CV_32SC1); // matrix of integer values
	int* RESULT = (int*)testResult.ptr(0);
	int* RESULTReal = (int*)testResult.ptr(1);
	int* RESULTAction = (int*)testResult.ptr(2);
	int* RESULTShortest = (int*)testResult.ptr(3);
	//int *RESLT= new int[MAXTESTNUM];
	//int *RESULTReal = new int[MAXTESTNUM];
	//int *RESULTAction = new int[MAXTESTNUM];
	//int *RESULTShortest = new int[MAXTESTNUM]; 
	for (int i=0; i<MAXTESTNUM; i++)
	{
		//RESULT[i] = 0;
		//RESULTReal[i] = 0;
		RESULTAction[i] = MAXITERATION;
		//RESULTShortest[i] = 0; 
	}

	// test mode: storing the initial and goal positions of all test runs
	::cv::Mat RESULTPosition = cv::Mat::zeros(MAXTESTNUM, 4, CV_32SC1);
	
	// preferably test mode: for visualisation of a single path
	::cv::Mat LOG_REALPOSITION;
	::cv::Mat LOG_ASSUMEDPOSITION;
	if (this->visualSwitch == 1)
	{
		LOG_REALPOSITION = cv::Mat::zeros(MAXITERATION, 3, CV_32SC1);
		LOG_ASSUMEDPOSITION = cv::Mat::zeros(MAXITERATION, 3, CV_32SC1);
	}

	// for tracking of state data and belief distribution 
	int PreviousState = 0;
	int PrevToPrevState = -1;
	int PrevToPrevPrev = -2;
	int RealState;
	
	double *BeliefState = new double[StateNum]; // will be the estimated distribution 
	double *ExpectState = new double[StateNum]; // will be the predicted next distribution
	double *PMunderS = new double[StateNum]; // the observation model
	for (int i=0; i<StateNum; i++)
	{
		BeliefState[i] = 0.0;
		ExpectState[i] = 0.0;
		PMunderS[i] = 0.0;
	}

	int PosX, PosY, PosD; // will be current position 
	int AssumedX, AssumedY, AssumedD; // will be the assumed position
	double uniformValue = 1.0 / StateNum;
	
	// Main LOOP 
	int RESSuccess = 0;
	bool terminationCall = false;
	int testnum = 0;
	//for(testnum=0; testnum < MAXTESTNUM; testnum++)
	while(testnum < MAXTESTNUM)
	{
		// for visualization
		if (this->visualSwitch == 1)
		{
			for (int i=0; i<MAXITERATION; i++)
			{
				LOG_REALPOSITION.at<int>(i, 0) = 0;
				LOG_REALPOSITION.at<int>(i, 1) = 0;
				LOG_REALPOSITION.at<int>(i, 2) = 0;
				LOG_ASSUMEDPOSITION.at<int>(i, 0) = 0;
				LOG_ASSUMEDPOSITION.at<int>(i, 1) = 0;
				LOG_ASSUMEDPOSITION.at<int>(i, 2) = 0;
			}
		}
     
		// Initial belief distribution is a uniform one
		for(int i=0; i<StateNum; i++)
		{
			BeliefState[i] = uniformValue;
			PMunderS[i] = 0.0; // p(m|s)
		}
	
		if (testMode)
		{
    // Get the current start position and goal position 
		GoalX = TestGoal.at<int>(testnum, 0);
		GoalY = TestGoal.at<int>(testnum, 1);
		GoalD = TestGoal.at<int>(testnum, 2);
		InitPosX = TestInitPos.at<int>(testnum, 0);
		InitPosY = TestInitPos.at<int>(testnum, 1);
		InitPosD = TestInitPos.at<int>(testnum, 2);
   
   // current position
		PosX = InitPosX;
		PosY = InitPosY;
		PosD = InitPosD;
   
   // current field position
   //CurrX = (PosX - 1) * 50 + 3; % Position in pixels
   //CurrY = (PosY - 1) * 50 + 3; % Position in pixels
   
		RESULTPosition.at<int>(testnum,0) = GoalX;
		RESULTPosition.at<int>(testnum,1) = GoalY;
		RESULTPosition.at<int>(testnum,2) = InitPosX;
		RESULTPosition.at<int>(testnum,3) = InitPosY;
		
		RESULTShortest[testnum] = max(abs(GoalX - InitPosX), abs(GoalY - InitPosY));
		}

		PreviousState = -1;
		PrevToPrevState = -2;
		PrevToPrevPrev = -3;
		
		int beststate = 0;
        double bestbelief = 0.0;
		
		int ITER;
		int MAXITERATION1 = MAXITERATION - 1;
		for (ITER = 0; ITER < MAXITERATION1; ITER++) 
		{
			// Step 1: increase ITER - done
			// Step 2: Select best state
			beststate = 0;
			bestbelief = 0.0;
			for (int j=0; j<StateNum; j++)
			{
				if (bestbelief < BeliefState[j])
				{
					beststate = j;  bestbelief = BeliefState[j];
				}
			}
			// current best belief state is the state of assumed position
			AssumedY = PosIndex.at<int>(beststate,0);
			AssumedX = PosIndex.at<int>(beststate,1);
			AssumedD = PosIndex.at<int>(beststate,2); 

			//
			if (!testMode) // send the current best state and the distribution to navigation control
				outBelief(&BeliefState[0], AssumedY, AssumedX, AssumedD); // from CVisOdom to robot's navigation control
			//

// fprintf('Previous state: %d, PrevToPrev state: %d\n', PreviousState, PrevToPrevState); 
// fprintf('Now best state is: (%d, %d)\n', AssumedX, AssumedY); 
			
			// for visualization
			if (this->visualSwitch == 1)
			{
				LOG_ASSUMEDPOSITION.at<int>(ITER, 0) = AssumedY;
				LOG_ASSUMEDPOSITION.at<int>(ITER, 1) = AssumedX;
				LOG_ASSUMEDPOSITION.at<int>(ITER, 2) = AssumedD;
				LOG_REALPOSITION.at<int>(ITER, 0) = PosY;
				LOG_REALPOSITION.at<int>(ITER, 1) = PosX;
				LOG_REALPOSITION.at<int>(ITER, 2) = PosD;
			}
			//
			// Present the assumed and best state
			// PresAandBestState();
			if (this->monitorSwitch == 1)
			{
				printf("ITER: %d. Assumed:(X=%d, Z=%d, dir=%d). Real: (X=%d, Z=%d, dir=%d)\n", ITER, AssumedX, AssumedY, AssumedD, PosX, PosY, PosD);
				printf("Best belief=%f\n", bestbelief);
			}

			// Step 3: TERMINATION TEST (for test mode)
			int IsRowMax = 0;
			int IsRowMin = 0;
			int IsColumnMax = 0;
			int IsColumnMin = 0;
			int IsBorder = 0; // for on-line mode, no specific border line can be detected
			
			if (testMode)
			{
			// The current state index is: 
			RealState = (PosY - 1) * NUMXxD + (PosX-1)* NUMD + PosD - 1;
			if (ITER > 2)
			{
				// Check if real position was reached
				if ((AssumedX == PosX) && (AssumedY == PosY) && (AssumedD == PosD))
				{
					if (this->monitorSwitch == 1)
						printf("assumed state is real state after %d actions\n", ITER); 
					if (RESULTReal[testnum] == 0) RESULTReal[testnum] = ITER;
				}
			
				// Check the SUCCESS end condition
				if ((AssumedX == GoalX) && (AssumedY == GoalY) && (AssumedD == GoalD))
				{
					if ((PosX == GoalX) && (PosY == GoalY) && (PosD == GoalD) )
					{
						if (this->monitorSwitch == 1)
							printf("%d: END with SUCCESS: goal state achieved after %d actions\n", testnum, ITER);
						RESULT[testnum] = 1;
						RESULTAction[testnum] = ITER;
						break;
					}
					
					// Check the potential success
					if ( (abs(PosX - GoalX) < 3) && (abs(PosY - GoalY) < 3) && (abs(PosD - GoalD) < 2) ) // the goal is within reach
					{
						if (this->monitorSwitch == 1)
							printf("%d: END with POTENTIAL SUCCESS: goal state nearly reached after %d actions\n", testnum, ITER);
						RESULT[testnum] = 1;
						RESULTAction[testnum] = ITER;
						break;
					}
					else
					{
						if (this->monitorSwitch == 1)
							printf("%d: END with FAILURE: assumed state is goal but not real after %d actions\n", testnum, ITER);
						RESULT[testnum] = 0;
						RESULTAction[testnum] = ITER;
						break;
					}
				}
			} // end ITER>2
			} // end test mode
			
			// The current state index is: 
			//RealState = (PosY - 1) * NUMXxD + (PosX-1)* NUMD + PosD - 1;

			// Check the FAILURE stopping condition
			// a ZERO action as the combination of 2 or 3 consecutive actions.
			if (ITER > 2)
			{
			if ((RealState == PrevToPrevState) || (RealState == PrevToPrevPrev))
			{
				if (this->monitorSwitch == 1)
				{ 
					printf("Test %d: LOOP detected - make a reset after %d actions\n", testnum, ITER);
					printf(" >>> Real PosX=%d, PosX=%d, PosD=%d; AssumedX=%d, AssumedZ=%d, AssumedD=%d\n", PosX, PosY, PosD, AssumedX, AssumedY, AssumedD);
					printf(" >>> Goal GoalX=%d,GoalZ=%d,GoalD=%d; \n", GoalX, GoalY, GoalD);
				}
				// Reset the belief state distribution
				for (int k = 0; k<StateNum; k++) BeliefState[k] = uniformValue;
				// again get at least 3 measurements	
				PreviousState = -1;
				PrevToPrevState = -2;
				PrevToPrevPrev = -3;
				// assume a new initial beststate
				beststate = (beststate + NUMZ + NUMX) % StateNum; 
				AssumedY = PosIndex.at<int>(beststate,0);
				AssumedX = PosIndex.at<int>(beststate,1);
				AssumedD = PosIndex.at<int>(beststate,2);
					// RESULT[testnum] = 0;
					// RESULTAction[testnum] = MAXITERATION;
					//restart from a new position
				}
			else // continue
			{
					PrevToPrevPrev = PrevToPrevState;
					PrevToPrevState = PreviousState;
					PreviousState = RealState;
			}
			} // end ITER > 2
			
			//
			// Step 4: compute the action expected by assumed best state
			if (testMode)
			{
				if (ITER >2)
				{
				DeltaX = GoalX - AssumedX;
				DeltaY = GoalY - AssumedY;
				DeltaD = GoalD - AssumedD;
				RealDeltaX = GoalX - PosX;
				RealDeltaY = GoalY - PosY;
				RealDeltaD = GoalD - PosD;
   
				// Check the weak stopping condition
				if ( (abs(DeltaX) < 2) && (abs(DeltaY) < 2) && (abs(DeltaD) < 2))  // We can stop after the next selected action
				{	
					if ( (abs(RealDeltaX) < 3) && (abs(RealDeltaY) < 3) && (abs(RealDeltaD) < 2) ) 
					{
						//if ( ((PosX + DeltaX) == GoalX) && ((PosY + DeltaY) == GoalY) ) 
						if (this->monitorSwitch == 1)
							printf("%d: END with SUCCESS: goal state will be achieved after %d actions\n", testnum, ITER+1);
						RESULT[testnum] = 1;
						RESULTAction[testnum] = ITER + 1; 
						if (this->visualSwitch == 1)
						{
							int ITER4 = ITER + 1;
							LOG_ASSUMEDPOSITION.at<int>(ITER4, 0) = AssumedY + DeltaY;
							LOG_ASSUMEDPOSITION.at<int>(ITER4, 1) = AssumedX + DeltaX;
							LOG_ASSUMEDPOSITION.at<int>(ITER4, 2) = AssumedD + DeltaD;
							LOG_REALPOSITION.at<int>(ITER4, 0) = PosY + RealDeltaY;
							LOG_REALPOSITION.at<int>(ITER4, 1) = PosX + RealDeltaX;
							LOG_REALPOSITION.at<int>(ITER4, 2) = PosD + RealDeltaD;
						}
						break;
					}
					else
					{
						if (this->monitorSwitch == 1)
							printf("%d: END with FAILURE: assumed state will be goal state but not real state after %d actions\n", testnum, ITER +1);
						RESULT[testnum] = 0;
						RESULTAction[testnum] = ITER;
						break;
					}
				}
			
				// 4a: the action is limited to elementary length 1
				if (DeltaX > 1)  DeltaX = 1;
				if (DeltaX < 0)  DeltaX = -1;
				if (DeltaY > 1)  DeltaY = 1;
				if (DeltaY < 0)  DeltaY = -1;
			
				// 4b: the action must keep the robot inside of the room space
				// The following check simulates a distance sensor
				IsColumnMax = 0;
				IsColumnMin = 0;
				IsRowMax = 0;
				IsRowMin = 0;
				if ((PosX + DeltaX) > (NUMX-2)) // limit the max N column to 17
				{
					if (DeltaY !=0)
					{
						DeltaX = 0;
						if (this->monitorSwitch == 1)
							printf("Warning: action dx set to 0!\n");
					}
					else
					{
						DeltaX =  0; //- DeltaX;         
						if (this->monitorSwitch == 1)
							printf("Warning: action dx set to 0 (was:reversed!\n");
					}
					IsColumnMax = 1;
				}
			
				if ((PosX + DeltaX) < 3) // limit the S column to 3
				{
					if (DeltaY !=0)
					{
						DeltaX = 0; 
						if (this->monitorSwitch == 1)
							printf("Warning: action dx set to 0!\n");
					}
					else
					{
						DeltaX = 0; //- DeltaX;         
						if (this->monitorSwitch == 1)
							printf("Warning: action dx set to 0 (was: reversed!\n");
					}
					IsColumnMin = 1;
				}
			
				if ((PosY + DeltaY) > NUMZ)
				{
					if (DeltaX !=0)
					{
						DeltaY = 0;
						if (this->monitorSwitch == 1)
							printf("Warning: action dy set to 0!\n");
					}
					else
					{
						DeltaY = 0; // - DeltaY;         
						if (this->monitorSwitch == 1)
							printf("Warning: action dy set to 0 (was: reversed!\n");
					}
					IsRowMax = 1;
				}
			
				if ((PosY + DeltaY) < 1)
				{
					if (DeltaX !=0)
					{
						DeltaY = 0; 
						if (this->monitorSwitch == 1)
							printf("Warning: action dy set to 0!\n");
					}
					else
					{
						DeltaY = 0; //-DeltaY;         
						if (this->monitorSwitch == 1)
							printf("Warning: action dy set to 0 (was: reversed!\n");
					}
					IsRowMin=1;
				}

				//4.c Modify action selection by orientation check:
				if ( (DeltaX==0) && (DeltaY==0) )
					// rotation only according to assumed DeltaD
				{
				}
				else
				{ 
					if (DeltaX < 0) DeltaD = 4 - AssumedD; // rotate to S
					else
						if (DeltaX > 0)  DeltaD = 2 - AssumedD; // rotate to N 
						else  
							if (DeltaY < 0) DeltaD = 1 - AssumedD; // rotate to E
							else
								DeltaD = 3 - AssumedD; // rotate to W
				}
			}
			else //  ITER < 3
			{
				// First 3 steps are rotations, performed in initial place
				DeltaX = 0;
				DeltaY = 0; 
				DeltaD = 1;
				if (this->visualSwitch == 1)
				{
					int ITER4 = ITER;
					LOG_ASSUMEDPOSITION.at<int>(ITER4, 0) = AssumedY;
					LOG_ASSUMEDPOSITION.at<int>(ITER4, 1) = AssumedX;
					LOG_ASSUMEDPOSITION.at<int>(ITER4, 2) = AssumedD;
					LOG_REALPOSITION.at<int>(ITER4, 0) = PosY;
					LOG_REALPOSITION.at<int>(ITER4, 1) = PosX;
					LOG_REALPOSITION.at<int>(ITER4, 2) = PosD;
				}
			}	

			// Step 5: Perform the action and set the next state
			PosX = PosX + DeltaX;
			PosY = PosY + DeltaY;
			PosD = PosD + DeltaD;
			if (PosD < 1) PosD += NUMD; // correction if out-of-range value
			if (PosD > NUMD) PosD -= NUMD;
			}
			else // Step 5 (on-line mode) get the robot's motion vector
			{
				IsBorder = inMotionVector(&DeltaY, &DeltaX, &DeltaD); // from robot's odometry to CVisOdom
				if (IsBorder) // border detected
				{
					IsColumnMin = 1; // any border is possible
					IsColumnMax = 1;
					IsRowMin = 1;
					IsRowMax = 1;
				}
			} // end if testMode

		// CurrX = CurrX + DeltaX * 50;
		// CurrY = CurrY + DeltaY * 50;
   
		// Step 6: Compute the expected (predicted) state beliefs
		if (ITER == 0)
		{
			for(int i=0; i<StateNum; i++)
			{
				ExpectState[i] = BeliefState[i];
			}
			// The above is effective for the first iteration only
		}
		else // perform prediction of next time belief distribution
		{
			int prevX, prevY, prevD;
			for (int j=0; j< StateNum; j++)
			{
				// Determine the predecessor state of every state under given action
				stateX = PosIndex.at<int>(j,1);
				stateY = PosIndex.at<int>(j,0);
				stateD = PosIndex.at<int>(j,2);
				prevX = stateX - DeltaX;
				prevY = stateY - DeltaY;
				prevD = stateD - DeltaD;
				if (prevD < 1) prevD += NUMD;
				if (prevD > NUMD) prevD -= NUMD; 
				if (prevX > NUMX) prevX = stateX; // assume remaining in state
				if (prevX < 1) prevX = stateX; // assume remaining in state
				if (prevY > NUMZ) prevY = stateY; // assume remaining in state
				if (prevY < 1) prevY = stateY; // assume remaining in state
			
				// Determine the index of predecessor state
				int prevInd = (prevY - 1) * NUMXxD + (prevX - 1) * NUMD + prevD - 1;
				// Compute the value of p(s_k| s_k-1, a_k)
				// The expected state depends on the predecessor's p(s|s)
				double total = 0.0;
				for (int k = 0; k<StateNum; k++)
					total += PSunderS.at<double>(k, prevInd) * BeliefState[k];
	
				ExpectState[j] = total;
			}
		} // end if ITER==0

		// Determine the state correspoding to real (but unknown) position
	    RealState = (PosY - 1) * NUMXxD + (PosX-1)* NUMD + PosD - 1;
		
		// Step 7: get measurement (image and its feature vector) at current real position
		// Read in the image in given real state (off-line mode) or get current image (on-line)
		::cv::Mat ObrazM;
		ObrazM = readMeasImage4(this->sInTest, PosY, PosX, PosD, testMode); // Get test image in state [k,l,dir]
		if (ObrazM.data==NULL) terminationCall = true; // finish the on-line work

		// ObrazO = ReadImage(FILENM, PosX, PosY); % the original image
		// Obraz = DisturbImage(ObrazO, ny, nx); %my function for disturbing the measurement
		
		// Step 8: eventually detect the person in current image
		::cv::Rect humanArea = objH.detectPerson(ObrazM);

		// Step 9: compute image features
		int nx, ny, nyxnx;
		switch (METHOD)
		{
		case M_MEANVAR:
			/*
      SObraz = double(reshape(Obraz(:,:,1), 1, nyxnx));
      mv = [0 0 0 0 0 0];
      mv(1) = mean(SObraz);
      %mv(1) = mv(1) * ScalingTheMean(mv(1), CurrMEANVAL(1), ScaleMEAN(1), ScaleCOEFF(1)); % First feature = mean of R
      mv(4) = std(SObraz) * CurrSTDVAL(1) ; % 4-th feature = std deviation of R
      SObraz = double(reshape(Obraz(:,:,2), 1, nyxnx));
      mv(2) = mean(SObraz);
      %mv(2) = mv(2) * ScalingTheMean(mv(2), CurrMEANVAL(2), ScaleMEAN(2), ScaleCOEFF(2)); % Second feature = mean of G
      mv(5) = std(SObraz) * CurrSTDVAL(2); % 5-th feature = std deviation of G
      SObraz = double(reshape(Obraz(:,:,3), 1, nyxnx));
      mv(3) = mean(SObraz); % 3-th feature = mean of B
      %mv(3) = mv(3) * ScalingTheMean(mv(3), CurrMEANVAL(3), ScaleMEAN(3), ScaleCOEFF(3)); % Third feature = mean of B
      mv(6) = std(SObraz) * CurrSTDVAL(3); % 6-th feature = std deviation of B
   %TEST
   mvO(1:6) = 0.0;
%   SObrazO = double(reshape(ObrazO(:,:,1), 1, 65536));
 %  mvO(1) = mean(SObrazO); % First feature = mean of R
 %  mvO(4) = std(SObrazO); % 4-th feature = std deviation of R
 %  SObrazO = double(reshape(ObrazO(:,:,2), 1, 65536));
 %  mvO(2) = mean(SObrazO); % 2-nd feature = mean of G
 %  mvO(5) = std(SObrazO); % 5-th feature = std deviation of G
 %  SObrazO = double(reshape(ObrazO(:,:,3), 1, 65536));
 %  mvO(3) = mean(SObrazO); % 3-th feature = mean of B
 %  mvO(6) = std(SObrazO); % 6-th feature = std deviation of B
 */
			break;
		case M_FFT:
			/*
      SObraz = double(Obraz(:,:,1)); % Only one colour image
      FFTVEC = fft2(SObraz);
      mv(1:3) = abs(FFTVEC(1,1:3)) ;%/abs(FFTVEC(1,1)); % 3 coefficients
      mv(4:6) = abs(FFTVEC(1,4:6));%/abs(FFTVEC(1,1)); % 3 coefficients
      %mv(1) = mv(1) * ScalingTheMean(mean(reshape(SObraz,1, nyxnx)), CurrMEANVAL(1), ScaleMEAN(1), ScaleCOEFF(1)); % First feature is approx. to energy, ca. mean of R
      mv(2:6) = mv(2:6) * CurrSTDVAL(1); % Remaining 5 featureas are similar to std deviation of R
%TEST
      mvO(1:6) = 0.0;
  %    SObrazO = double(ObrazO(:,:,1));
  %    FFTVECO = fft2(SObrazO);
  %    mvO(1:3) = abs(FFTVECO(1,1:3));%/abs(FFTVEC(1,1)); % 3 coefficients
  %    mvO(4:6) = abs(FFTVECO(1,4:6));%/abs(FFTVEC(1,1)); % 3 coefficients
  */
			break;
		case M_HIST:
			/*
      SObraz = double(Obraz(:,:,1)); % Only one image at first
      mvMat(1:6,1:6) = HistogramFeatures(SObraz, ny, nx);
      %for (l=1:1:6)
        % mv(2) = ScalingTheMean(mvMat(l, 1), CurrMEANVAL(1), ScaleMEAN(1), ScaleCOEFF(1)); %
         %mvMat(l, 1) = mv(2) * mvMat(l, 1); % The intensities are rescaled
         %end
      mv(1:6) = mvMat(1,1:6); % The first largest density
   %TEST
      %mvOMat(1:6, 1:6) = 0.0;
      %SObrazO = double(ObrazO(:,:,1));
      %mvOMat(1:6,1:6) = HistogramFeatures(SObrazO, ny, nx);
      %mvO(1:6) = mvOMat(1,1:6); % The first largest density
   case 'HISTSMALL',
      SObraz = double(Obraz(:,:,1)); % Only one image at first
      mv = HistogramSmallFeat(SObraz, ny, nx, 0.2);
      %for (ll=1:1:3)
       %  val = ScalingTheMean(mv(ll), CurrMEANVAL(1), ScaleMEAN(1), ScaleCOEFF(1)); %
       %  mv(ll) = mv(ll) * val;
      %mv(ll+3) = mv(ll+3) / val; % The intensities are rescaled
      %end
      %TEST
      mvO(1:6) = 0.0;
      SObrazO = double(ObrazO(:,:,1));
      mvO = HistogramSmallFeat(SObrazO, ny, nx, 0.5);
      %TEST
      %for (testind=1: 1: 120)
      %   fprintf('Orig: %d, Contrast: %d, Orig/Con = %d\n', SObrazO(testind,testind), SObraz(testind,testind), SObrazO(testind,testind)/SObraz(testind,testind));
      %   input('press key');
      %end
	  */
			break;
		case M_COLORHIST:
			/*
      mvMat(1:6,1:7) = MEASMATVEC(1:6, 1:7, RealState);
      mv(1:7) = mvMat(1, 1:7); % Only the best one
	  */
			break;

		case M_COLORMAX:
			 ny = ObrazM.rows; nx = ObrazM.cols; 
			 nyxnx = ny * nx;
					   // MVecMat(1:NUMBLOCKS,1:3, NrStanu)
			 ColorMaxFeatures(ObrazM, ny, nx, MEASMAT); //> Compute feature vector

			break;
		default:
			break;
		}


   // Step 9: Compare current image feature vector with the model of states --> get current distribution p (s / m)
		for (int k=0; k< StateNum; k++) { PMunderS[k] = 0.0; }
   
		double Dist = 0.0;
		int realMeas;
		for (int k=0; k< StateNum; k++)
		{
			Dist = 0.0; 
			switch (METHOD)
			{
			case M_MEANVAR:
			case M_FFT:
			case M_HISTSMALL:
				for (int j=0; j<6; j++)
					Dist = Dist + (abs(mv[j] - MVec.at<double>(k,j)))/ScaleMEASURE[j]; 
				//mean distance 
				break;
				
			case M_HIST:
				for (int j=0; j<6; j++)
				{
					double DistJJ = 99999999999.9; 
					for (int jj = 0; jj<6; jj++)
					{
						double DistP = 0.0;
						for (int l=0; l<6; l++)
							DistP = DistP + (abs(mvMat.at<double>(j,l) - MVecMat[k].at<double>(jj,l))) / ScaleMEASURE[l];
						//matrix of features
						if (DistP < DistJJ) DistJJ = DistP;
					}
					Dist += DistJJ;
				}
				break;
				
			case M_COLORHIST:
                for (int jj = 0; jj<16; jj++)
					for (int ll=0; ll<7; ll++)
						Dist += abs(MEASMATVEC[RealState].at<double>(jj,ll) - MVecMat[k].at<double>(jj,ll)) / ScaleMEASURE[ll]; 
					//matrix of features
                break;
                
			case M_COLOR:
				if (testMode)
				{
				for (int jj = 0; jj<16; jj++)
					for (int ll=0; ll<3; ll++) 
						Dist += abs(MEASMATVEC[RealState].at<double>(jj,ll) - MVecMat[k].at<double>(jj,ll));
				}
				else
				{
				for (int jj = 0; jj<16; jj++)
					for (int ll=0; ll<3; ll++) 
						Dist += abs(MEASMAT.at<double>(jj,ll) - MVecMat[k].at<double>(jj,ll));
				}
                break;
                     
			case M_COLORMAX:
				if (testMode)
				{
					//realMeas= stateTest[RealState];
					realMeas = RealState;
                for (int jj = 0; jj<NUMBLOCKS; jj++) 
					for (int ll=0; ll<3; ll++) 
						//Dist += abs(MEASMATVEC[RealState].at<double>(jj,ll) - MVecMat[k].at<double>(jj,ll));
					    Dist += /* MVecMat[k].at<double>(jj, 3) * */ abs(MEASMATVEC[realMeas].at<double>(jj,ll) - MVecMat[k].at<double>(jj,ll));
				}
				// / ScaleMEASURE(1:7);
				else
				{
				for (int jj = 0; jj<16; jj++)
					for (int ll=0; ll<3; ll++) 
						// Dist += ScaleMEASURE[ll] * abs(MEASMAT.at<double>(jj,ll) - MVecMat[k].at<double>(jj,ll));
						Dist += /* MVecMat[k].at<double>(jj, 3) * */ abs(MEASMAT.at<double>(jj,ll) - MVecMat[k].at<double>(jj,ll));
				}
				break;
			}
			// avoiding small distance values
			if (Dist <= 0.001) 
				PMunderS[k] = 1000.0; // maximum belief
			else
				PMunderS[k] = 1.0 / Dist; // belief weight is inverse of distance
		}

		// 8a. eventually correct belief weights by information from the distance sensor
		int StateId, StId;
		if (IsColumnMin == 1) // x is 3
		{
			StateId = 2 * NUMD;
			for (int ky=0; ky<NUMZ; ky++)
			{
				for (int kd=0; kd <NUMD; kd++)
				{
					//RealState = (PosY - 1) * NUMXxD + (PosX-1)* NUMD + PosD -1;
					StId = StateId + kd;
					PMunderS[StId] = PMunderS[StId] * 10.0;
				}
				StateId = StateId + NUMXxD;
			}
		}
		if (IsColumnMax == 1) //  x=17
        { 
			StateId = 16 * NUMD;
			for (int ky=0; ky<NUMZ; ky++)
			{
				for (int kd=0; kd<NUMD; kd++)
				{
                 //RealState = (PosY - 1) * NUMXxD + (PosX-1)* NUMD + PosD -1;
					StId = StateId + kd;
					PMunderS[StId] = PMunderS[StId] * 10.0;
				}
				StateId = StateId + NUMXxD;
			}
		}
		
		if (IsRowMin == 1) // y=1
        {
			StateId = 1;
			for (int kx=0; kx<NUMX; kx++)
			{ 
				for (int kd=0; kd<NUMD; kd++)
				{
                 //RealState = (PosY - 1) * NUMXxD + (PosX-1)* NUMD + PosD -1;
					StId = StateId + kd;
					PMunderS[StId] = PMunderS[StId] * 10.0;
				}
				StateId = StateId + NUMD;
			}
		}
		
		if (IsRowMax == 1) // y=41
		{
			StateId = 40 * NUMXxD;
			for (int kx=0; kx<NUMX; kx++)
			{
				for (int kd=0; kd<NUMD; kd++)
				{
                 //RealState = (PosY - 1) * NUMXxD + (PosX-1)* NUMD + PosD-1;
					StId = StateId + kd;
					PMunderS[StId] = PMunderS[StId] * 10.0;
				}
				StateId = StateId + NUMD;
			}
		}

		////////////////
		//if (this->visualSwitch == 1) { showBelief(BeliefState, StateNum, "belief i"); }
		// Step 9: update the belief state distribution
		for (int j=0; j<StateNum; j++)
			BeliefState[j] = PMunderS[j] * ExpectState[j];
		
		//if (this->visualSwitch == 1) 
		//{ 
	//		showBelief(ExpectState, StateNum, "predict i+1"); 
		//	showBelief(PMunderS, StateNum, "PMunderS i+1"); 
		//	showBelief(BeliefState, StateNum, "belief i+1"); 
		//	::cv::waitKey();
		//}

		//  Normalize to 1 
		Dist = 0.0;
		for (int j=0; j<StateNum; j++)
			Dist += BeliefState[j];
		if (Dist > 0)
			for (int j=0; j<StateNum; j++) 
				BeliefState[j] = BeliefState[j] / Dist;

		//////////////////
		// Eventually show the current results at iteration ITER of test "testnum
		if (this->visualSwitch == 1)
		{
			// VIS: show current belief state distribution
			// for simplicity - present as an image
			
			::cv::Mat BELIEF = ::cv::Mat::zeros(1, NUMX * NUMZ, CV_64FC1);
			::cv::Mat BELIEF2D = ::cv::Mat::zeros(NUMZ, NUMX, CV_64FC1);
			int k = 0;
			int state = 0;
			double maxval, totalmax = 0.0;
			for (int a=0; a<NUMZ; a++)
			{
				for (int b = 0; b< NUMX; b++)
				{
					maxval = BeliefState[state];
					for (int d=1; d<NUMD; d++)
						if (BeliefState[state+d] > maxval)
							maxval = BeliefState[state+d];

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
					
					::cv::rectangle( belief2DImg, ::cv::Point(s*scaleSize, (NUMZ - h-1) *scaleSize),
					::cv::Point( (s+1)*scaleSize - 1, (NUMZ-h)*scaleSize - 1), intensity, CV_FILLED );
				}
			::cv::namedWindow( "belief state histogram", 1 );
			::cv::imshow( "belief state histogram", beliefImg  );
			::cv::namedWindow( "belief state image", 1 );
			::cv::imshow( "belief state image", belief2DImg );

			if (this->logSwitch == 1)
			{
				char outName[128];
				sprintf_s(outName, "%s/Belief_%03d_%02d.png", this->sOUT, testnum, ITER); // file name
				::cv::imwrite(outName, beliefImg); 

				sprintf_s(outName, "%s/Belief2D_%03d_%02d.png", this->sOUT, testnum, ITER); // file name
				::cv::imwrite(outName, belief2DImg); 
			}
			beliefImg.release();
			belief2DImg.release();
		//	::cv::waitKey();
		}
		// Print belief state distribution
		//if (this->monitorSwitch == 1)
		//{
		//	cout<< "RESULT BELIEF STATE DISTRIBUTION: ";
		//	for(int i =0; i<StateNum; i+=NUMD)
		//	{ 
		//		cout<< i<<": "<< BeliefState[i] <<", "<<BeliefState[i+1]<<", "<< BeliefState[i+2]<< ", "<< BeliefState[i+3] << "; ";
		//	}
		//	cout << endl;
		 //}
		::cv::waitKey();
		if (this->monitorSwitch == 1)
		{
			// print intermediate data
			printf("ITER %d. Best state: %d with (X=%d, Y=%d);\n",ITER, beststate, PosIndex.at<int>(beststate,1), PosIndex.at<int>(beststate,0)); 
			printf("Action: %d, %d.  Assumed Position: X=%d, Z=%d\n", DeltaX, DeltaY, AssumedX + DeltaX, AssumedY + DeltaY); 
			printf("Real state(X,Z): (%d, %d); Goal state(X,Z): (%d, %d).\n",PosX, PosY, GoalX, GoalY); 
		}
		
	} // end ITER loop

	// Check the termination condition 
	if (terminationCall) // the on-line work is terminated
		break;

// Show both paths - assumed and real
	
	int ITER4 = ITER;

	if (RESULT[testnum] == 0) 
	{
		if (ITER == MAXITERATION1)
		{
			if (this->monitorSwitch == 1) 
				printf("Test %d: FAILURE - maxiteration reached\n", testnum);
			ITER4 = ITER - 1;
		}
		else
		{
			if (this->monitorSwitch == 1) 
				printf("Test %d: FAILURE \n", testnum);
			ITER4 = ITER ;
		}

	} 
	else
	{
		RESSuccess ++; 
		if (this->monitorSwitch == 1) 
			printf("After %d tests: SUCCESS (%d), FAILURE (%d) \n", testnum +1, RESSuccess, testnum - RESSuccess +1);
	}
			
	if (this->visualSwitch == 1)
	{
		int scale = 10;
		::cv::Point p1, p2;
		::cv::Mat pathImg = ::cv::Mat::ones((NUMZ+1) * scale, (NUMX+1) * scale, CV_8UC3) * 255;
		for (int i=1; i<ITER4; i++)
		{
			p1 = ::cv::Point((LOG_REALPOSITION.at<int>(i-1,1) -1) * scale, (NUMZ - LOG_REALPOSITION.at<int>(i-1,0) ) * scale );
			p2 = ::cv::Point((LOG_REALPOSITION.at<int>(i,1) -1) * scale , (NUMZ - LOG_REALPOSITION.at<int>(i,0) ) * scale );

			::cv::line(pathImg, p1, p2, ::cv::Scalar(0, 255, 255), 2, 8, 0); 

			p1 = ::cv::Point((LOG_ASSUMEDPOSITION.at<int>(i-1,1)-1) * scale, (NUMZ - LOG_ASSUMEDPOSITION.at<int>(i-1,0) ) * scale );
			p2 = ::cv::Point((LOG_ASSUMEDPOSITION.at<int>(i,1) -1) * scale , (NUMZ - LOG_ASSUMEDPOSITION.at<int>(i,0) ) * scale );
			
			::cv::line(pathImg, p1, p2, ::cv::Scalar(100, 255, 0), 2, 8, 0); 
		}

		if (this->logSwitch == 1)
		{
			char outName[128];
			sprintf_s(outName, "%s/Path_%03d.png", this->sOUT, testnum); // file name
			::cv::imwrite(outName, pathImg); 
		}
		
		pathImg.release();
	}

	//
	

	testnum++; // otherwise continue

	if (!testMode) // it is the on-line work mode
	{
		if (testnum == MAXTESTNUM)
		{
			testnum = 0; // again continue from the front of the result buffer
			RESSuccess = 0;
		}
	}

  } //END of main loop ("while testnum")
  

  // Show the summary of test results
  if (this->monitorSwitch == 1)
  {
	  int sumShortest=0;
	  std::cout<< "RESULT shortest path length: ";
	  for(int i =0; i<testnum; i++)
	  { 
		  cout<< i<<": "<<RESULTShortest[i] << "; ";
		  sumShortest += RESULTShortest[i];
	  }
	  cout << endl;
	  int sumAction = 0;
	  cout<< "RESULT action: ";
	  for(int i =0; i<testnum; i++)
	  { 
		  cout<< i<<": "<<RESULTAction[i] << "; ";
		  sumAction += RESULTAction[i];
	  }
	  cout << endl;
	  cout<< "RESULT real path achieved after: ";
	  for(int i =0; i<testnum; i++)
	  { 
		  cout<< i<<": "<<RESULTReal[i] << "; ";
	  }
	  cout << endl;
	  
	  RESSuccess =0;
	  double RESDistance;
	  for(int i =0; i<testnum; i++)
	  { 
		  RESSuccess += RESULT[i];
	  }
	  if (RESSuccess > 0)
		  RESDistance = ((double)sumAction) / RESSuccess;
	  else
		  RESDistance = MAXITERATION;

	  double RESShortest = ((double)sumShortest) / testnum;
	  
	  double RESDistToShortest = RESDistance / RESShortest;

	  cout<< "Success number: "<<RESSuccess<<endl;
	  cout<< "Average distance: "<<RESDistance<<endl;
	  cout<< "Shortest average distance: "<<RESShortest<<endl;
	  cout<< "Average distance excess rate: "<<RESDistToShortest<<endl;

	  ::cv::waitKey();
  }
  
  delete [] stateTest;

  return testResult;

} // END of function

