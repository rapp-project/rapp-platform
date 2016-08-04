/// Plik: VisOdom.h
/// Deklaracja klasy CVisOdom

#ifndef VISODOM
#define VISODOM

#include "rapp_visual_localization/Parameters.hpp"
#include "rapp_visual_localization/Properties.hpp"
#include "rapp_visual_localization/UserParameters.hpp"
#include "rapp_visual_localization/Human.hpp"
#include "rapp_visual_localization/ResultOD.hpp"

#include <time.h>

#include <opencv2/opencv.hpp>

using namespace std;

////////////////////////////////////////
enum MType{
	M_MEANVAR, M_FFT, M_HISTSMALL, M_HIST, M_COLOR, M_COLORHIST, M_COLORMAX
};

/////////////////////////////////////////
/// The visual odometry class
///
	class CVisOdom{
	public:
		///
		/// ...

	private:
		/// pola : dane we/wy i parametry analizy
		char *sInMap; ///> katalog obrazów dla procesu modelowania (tworzenia mapy)
		char *sInTest; ///> katalog obrazów - pomiarów
		char *sOUT; ///> katalog dla wyników procesu modelowania
        CProperties prop;
        CParameters param;
        CUserParameters userparam;
        
		/// wymiary dyskretnej mapy pomieszczenia 2D
		int NUMX; ///> number of columns
		int NUMZ; ///> number of rows
		int NUMD; ///> number of orientations % E (00), N (03), W (06), S (09)
	
		int NUMXxD;
		int StateNum; ///> number of states
		float GRID; ///> distance [in meters] between neigbour states
		int VGrid; ///> number of states per grid
		::cv::Mat PosIndex; //> correspondence between 2D location and index of every state
		
		/// reprezentacja (cechy) obrazu w każdym stanie
		int NUMBLOCKS; ///> number of image blocks
		double ScaleMEASURE[112]; //> scaling of feature elements
		
		/// memory for image features
		double mv[7]; // for M_MEANVAR and M_FFT
		::cv::Mat MVec; // for M_HISTSMALL
		::cv::Mat mvMat; // for M_HIST, M_COLORHIST, M_COLORMAX (on-line mode)
		::cv::Mat MEASMAT; // for M_COLORMAX
		
		::cv::Mat *MVecMat; //  matrix-like features (for model map)
		// measurements in test mode
		::cv::Mat *MEASMATVEC; // for matrix-like features (for off-line test measurments - created in advance) 

		// pola: przełączniki wizualizacji i monitorowania
		int visualSwitch; // 1: czy ma prezentować okna on-line
		int monitorSwitch; // 1: czy ma się zatrzymywać krok-po-kroku
		int logSwitch; // 1: czy zapisywać wyniki do plików 
	
	public:

	   	CVisOdom(char *inM, char* inT, char* out, CProperties pr, CParameters pa, CUserParameters upa,
			int vis=0, int monit=0, int log=0)
		{
			this->sInMap = inM;
			this->sInTest = inT;
			this->sOUT = out;
            this->prop = pr;
            this->param = pa;
            this->userparam = upa;
            //this->pattern = new CResultOD[upa.getViews()];
			
			this->visualSwitch = vis; // 1: czy ma prezentować okna on-line
			this->monitorSwitch = monit; // 1: czy ma drukować komentarze
			this->logSwitch = log; // 1: czy ma utrwalać obrazy wynikowe na dysku
		}
		
		//
		CVisOdom()
		{
		}

		// Metody
		void initMap(int numx=19, int numz=41, int numd=4, int vgrid=5);
		void initFeatureMemory(MType METHOD);
		void initFeatureScaling(MType METHOD);
		int createMap(MType method, char* mapXml, char* measXml, 
			int mapSwitch=1, int measSwitch=1 );

		void releaseMap(MType METHOD);

		::cv::Mat work(int TESTNUM, int MAXITER, MType METHOD, CHuman& objH, 
			char* mapXml, char* measXml, bool testMode);

		// subfunctions
		::cv::Mat readImage4(char* fname, int row, int col, int dir);
		void ColorMaxFeatures(::cv::Mat& Obraz, int ny, int nx, ::cv::Mat& MVecMat);
		::cv::Mat readMeasImage4(char* fname, int row, int col, int dir, bool testMode=true);

		// monitoring
		void showBelief(double *State, int length, char *text); 
		void featureTest(::cv::Mat* MVecMat, ::cv::Mat* MEASMATVEC, int* stateTest, int NUMBLOCKS);

		// TO DO (interfacing the on-line navigation process)
		::cv::Mat inObservationImage(); // from robot's camera to CVisOdom
		bool inMotionVector(int *dz, int *dx, int *dd); // from robot's odometry to CVisOdom
		void outBelief(double *belief, int bestZ, int bestX, int bestD); // from VisOdom to robot's navigation control
	
	}; // END of CVisOdom
	////////////////////////////////////////////
	

#endif
