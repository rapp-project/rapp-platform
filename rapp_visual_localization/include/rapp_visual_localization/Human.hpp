// Human.h
// Deklaracja klasy CHuman

#ifndef HUMAN
#define HUMAN

#include "rapp_visual_localization/Parameters.hpp"
#include "rapp_visual_localization/Properties.hpp"
#include "rapp_visual_localization/UserParameters.hpp"
#include "rapp_visual_localization/ResultOD.hpp"

#include <time.h>

#include <opencv2/opencv.hpp>

using namespace std;


/////////////////////////////////////////
// Klasa
//
	class CHuman{
	public:
		// obiekt pomocniczy
		// ...

	private:
		// pola : dane we/wy i parametry analizy
		char  *streamIN; // katalog dla procesu modelowania
		char *streamOUT; // katalog dla wyników procesu modelowania
        CProperties prop;
        CParameters param;
        CUserParameters userparam;
        CResultOD *pattern; // alternatywne modele sylwetki

		// pola: przełączniki wizualizacji i monitorowania
		int visualSwitch; // 1: czy ma prezentować okna on-line
		int monitorSwitch; // 1: czy ma się zatrzymywać krok-po-kroku
		int logSwitch; // 1: czy zapisywać wyniki do plików 
	
	public:

	   	CHuman(char *in, char* out, CProperties pr, CParameters pa, CUserParameters upa,
			int vis=0, int monit=0, int log=0)
		{
			this->streamIN = in;
			this->streamOUT = out;
            this->prop = pr;
            this->param = pa;
            this->userparam = upa;
            this->pattern = new CResultOD[upa.getViews()];
			
			this->visualSwitch = vis; // 1: czy ma prezentować okna on-line
			this->monitorSwitch = monit; // 1: czy ma drukować komentarze
			this->logSwitch = log; // 1: czy ma utrwalać obrazy wynikowe na dysku
		}
		
		//
		CHuman()
		{
		}

		//
		void createModel(char* directoryIN, char* directoryOUT, char* modelFilenameXml,
			bool isModel=false, int visualswitch=0, int logswitch=0, int monitorswitch=0);

		::cv::Rect detectPerson(::cv::Mat obraz);
		

	}; // Koniec definicji klasy Analysis
	////////////////////////////////////////////
	

#endif
