#include "rapp_visual_localization/Human.hpp"

using namespace std;


void CHuman::createModel(char* directoryIN, char* directoryOUT, char* modelFilenameXml,
			bool isModel, int visualswitch, int logswitch, int monitorswitch)
{

}

::cv::Rect CHuman::detectPerson(::cv::Mat obraz)
{
	::cv::Rect detected;


	return detected;
}

		
/*

namespace ROD{
	//const double PI = CV_PI;//

	// "Rozciąganie histogramu"
	void HistoStretching(int a0, int A, int B, int a1, ::cv::Mat &MImg, ::cv::Mat &NormMImg)
	{
		int my=MImg.rows;
		int mx=MImg.cols;
		int val, nval;

		NormMImg = MImg.clone();
		double wsp =  ((double)(a1 - a0)) / ((double)(B - A));


		for (int j=0; j<mx; j++) { 
			for (int i=0; i<my; i++) {
				val = (int) MImg.at<uchar>(j, i);
				if (val <= A) 
					NormMImg.at<uchar>(j, i) = a0; // min value, e.g. 0
				else
					if (val >=B)
						NormMImg.at<uchar>(j, i) = a1; // max value, e.g. 230
					else { 
						nval = round(a0 + wsp * (val - A)); 
						if (nval  < a0) nval = a0;
						else if (nval > a1) nval = a1;
						NormMImg.at<uchar>(j, i) = nval;
					}
			}
		}

	}

	// Funkcja pomocnicza w wizualizacji - rysowanie prostokąta w obrazie mono
	void DrawBox(::cv::Mat &InImg, int colMin, int colMax, int rowMin, int rowMax, uchar box_col)
	{
		for( int y = rowMin; y < rowMax; y++ )
		{
			InImg.at<uchar>(y, colMin) = box_col;
			InImg.at<uchar>(y, colMax) = box_col;
		}

		for( int x = colMin; x < colMax; x++ )
		{
			InImg.at<uchar>(rowMin, x) = box_col;
			InImg.at<uchar>(rowMax, x) = box_col;
		}
		
	} // Koniec DrawBox

	// Funkcja pomocnicza - wykreśl prostą przechodzącą przez punkt (Cx, Cy) i 
	// zorientowaną pod katem alfa (w radianach) - w obrazie mono
	void DrawLine(double alfa, uchar value, int Cx, int Cy, ::cv::Mat &FCImg)
	{
		int my = FCImg.rows;
		int mx = FCImg.cols;
		double tgalfa = tan(alfa);
		int xr, yr;
		double x, y;
		
		// Generuj wsp. y odpowiadające kolejnym indeksom x
		for (int i=0; i< mx; i++) {
			y = - (tgalfa * (i - Cx) - Cy); // współrzędne obrazu, ale zamień znak Y 
			yr = round(y); 
			if ((yr >= 0) && (yr < my)) {
				FCImg.at<uchar>(yr, i) = value;
			}
		}
		// Generuj wsp. x odpowiadające kolejnym indeksom y 
		for (int i=0; i< my; i++) {
			x = (-(i - Cy))/tgalfa + Cx; // współrzędne obrazu, ale zamień znak Y 
			xr = round(x); 
			if ((xr >= 0) && (xr < mx)) {
				FCImg.at<uchar>(i, xr) = value;
			}
		}
	}

	// Funkcja pomocnicza - tworzy obraz 2D histogramu dla funkcji 1D
	void DrawFunct( ::cv::Mat &histoIR, int histSize, ::cv::Mat &histImg) {				 	
		::cv::normalize(histoIR, histoIR, 0, histImg.rows, CV_MINMAX, CV_32F);
		histImg = ::cv::Scalar::all(255);
		int binW = cvRound((double)histImg.cols/histSize);
		for( int i = 0; i < histSize; i++ ) {
			rectangle( histImg, ::cv::Point(i*binW, histImg.rows), 
					 ::cv::Point((i+1)*binW, histImg.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 );
		}
	}

	*/


/////////////////////////////////////////
// Metody głównej klasy CHuman programu RappAppl
//
/*	
		void Detekcja(char* dirName, char* outDir, const ResultODline *storedRes)
		{
			// Detekcja - zasadnicza funkcja detekcji jabłka i szypułki w obrazach IR 
            // należących do "pakietu obrazów"
			//
            // Dla każdego widoku wywołuje funkcję AnalizaJablka()
            //
            // W wersji off-line funkcji - obrazy są pobierane z plików graficznych w katalogu 
                           
			int numV = prop.getNumViews();
			int numL = prop.getNumLines();

			char fName[128];
			char outName[128];

            // Główna pętla: dla każdej linii i każdego widoku 
			ResultOD jedenWynik;
            for (int i=0; i<numL; i++) //domyślnie linie taśmociągu 
				for (int j=0; j<numV; j++) // domyślnie 8 widoków jabłka
				{
					// dla wersji off-line:
					// określamy nazwy obrazów w plikach
                    sprintf(fName, "%02d/A/%02d", i+1, j+1); // ścieżka i początek nazwy plików wejściowych 
                    sprintf(outName, "%02d_A_%02d", i+1, j+1); // % element nazwy plików z wynikami

					// dla wersji on-line
					// ...

					int offset = i * numL + j;

					

                    // Wywołanie funkcji AnalizaJablka() dla jednego widoku jednego jabłka
                    jedenWynik = AnalizaJablka(dirName, fName, outDir, outName, offset);
					
					// dla wyniku:
					storedRes[j].resultL[i] = jedenWynik;
					
					// Tu należy przechwycić pojedynczy wynik zwracany zmienną jedenWynik
					// ...
					// ...


					if (this->monitorSwitch == 1) // 1: czy ma się zatrzymywać krok-po-kroku
					{
						cout << "Wykonano analizę: linia " << i << "; widok " << j<< endl;
						cv::waitKey(0);
					}

				}
		} // Koniec funkcji Detekcja()
               
  */

          
		
            
        /**
        * @fn Analiza pojedynczego widoku jabłka w celu wykrycia jego połoźenia i szypułki
        
        // PARAMETRY FUNKCJI
        //  fileName - podstawowa nazwa pliku graficznego png;
        //  dirName - nazwa katalogu
        //  outDir - katalog dla wyników
		//  wynik - referncja typu ResultOD dla reprezentacji wyniku

		// WYNIK
		// wynik - opis pojedynczego widoku jednego jabďż˝ka typu ResultOD
      
        // Autor: Wlodzimierz Kasprzak  & Jan Figat
        // Modyfikacja: 4-11-2014
        // 
		

        ResultOD AnalizaJablka(char* dirName, char* fileName, char* ouDir, char* ouName, int offset)
		{
			// Ustaw obiekty - PARAMETRY analizy obrazu
			Parameters param = this->param;
            Properties prop = this->prop;
            UserParameters uparam = this->userparam;

			// Przełączniki - wizualizacja i monitorowanie wyników częściowych
			int VISUAL = this->visualSwitch; // 1: czy ma prezentowaďż˝ okna on-line
			int MONITOR = this->monitorSwitch; // 1: czy ma siďż˝ zatrzymywaďż˝ krok-po-kroku
			int LOGS = this->logSwitch; // 1: utrwalenie wynikďż˝w w plikach dla ilustracji dziaďż˝ania funkcji


			// Parametry szczegółowe
			int DIRS = param.directions; // 720: podział kąta pełnego na 720 części

			// Nazwy plików
			char fIrName[128];
			char outName[128];
			// Pomiar czasu
			double t0, elapsed;
			time_t start, end;
			// Zakres kątowy przeszukiwania konturu i wnętrza
			cv::Mat wycinekPoIr = cv::Mat::zeros(2,1,CV_32SC1);
			cv::Mat wycinek = cv::Mat::zeros(3,1,CV_64FC1);
			
			// Zaczynamy
			
			if (MONITOR == 1) std::cout<< "AnalizaJablka() start\n"; 
			
			/////////////////////////////////
			// Krok A1. Ścieżka dostępu do obrazu, jego wczytanie i normalizacja
			//
			t0=clock();		// zliczanie czasu
			start=time(0);
			//
			// 1.1 Wczytaj obraz IR 
			//
			sprintf(fIrName, "%s/%s_0.png\0", dirName, fileName); 
			
			if (MONITOR == 1) 
				std::cout<< fIrName << std::endl; 
			
			cv::Mat IRImg;
			// TEST only:
			//IRImg = cv::imread("C:/SORTER/img/Database/01_Diffuse/01/A/02_0.png");

			IRImg = ::cv::imread(fIrName);
	

			if(! IRImg.data ){							///sprawdzenie czy plik istnieje
			    cout<<"\n"<<fIrName<<" -- IR"<<endl;
			        throw  "Nie można otworzyć pliku" ;
			}
			if (MONITOR == 1) 
				std::cout << "L kanałow IRImg: " << IRImg.channels() << std::endl; 
			
			if (VISUAL== 1) { 	
				::cv::imshow("A1.1 Obraz IR", IRImg); 
			}

			int gy = IRImg.rows;
			int gx = IRImg.cols;
			
			// Obraz IR posiada niepotrzebnie 3 płaty
			vector<cv::Mat> IRplanes; // Vector to klasa szablonowa
			::cv::split(IRImg, IRplanes); // Rozdziel obraz na 3 płaty 

			//
			//1.2. Normalizacja histogramu - rozciąganie lub przeskalowanie
			//
			int histSize = 256;
			::cv::Mat histoIR;
			::cv::Mat histImage = ::cv::Mat::ones(256, 256, CV_8U)*255;
			
			//Wyznacz histogram obrazu IR (tylko w trybie wizualizacji)
			if (VISUAL== 1) {
				 ::cv::calcHist(&IRplanes[0], 1, 0, ::cv::Mat(), histoIR, 1, &histSize, 0);
				 
				 ::cv::normalize(histoIR, histoIR, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				::cv::imshow("A1.1 Histogram obrazu IR", histImage); 
			}

			// Normalizacja histogramu
			// W oryginale: było rozciąganie histogramu obrazu
			// z parametrami [0.05 0.95] i odwzorowaniem [0.0 0.90]*255
			
			double minVal, maxVal;			
			double alpha ; //
			// Rozciąganie/przeskalowanie dla obrazu IR - 
			::cv::Mat NormIRImg;
			::cv::minMaxIdx(IRplanes[0], &minVal, &maxVal, NULL, NULL, ::cv::noArray());
			alpha = 230.0/ maxVal;
			alpha = 0.9 * maxVal;
			if (MONITOR == 1)
				cout<< "MaxVal for IR: "<< maxVal << " alpha: " << alpha << endl;

			if (alpha > 200)
				alpha = 200;
			
			// Rozciąganie
			ROD::HistoStretching(0, 20, (int)alpha, 230, IRplanes[0], NormIRImg);
			// O ile IRImg ma 3 płaty, to NormIRImg bedzie miał już tylko 1
			
			// Lub przeskalowanie:
			// IRplanes[0].convertTo(NormIRImg, CV_8U, alpha);
			
			// Wizualizacja wyniku
			if (VISUAL== 1) { 
				::cv::imshow("A1.2 Norm IR", NormIRImg); 
			}
			// Utrwalenie wyniku
			if (LOGS == 1)
			{ 
				sprintf(outName, "%s/NormIR_%s.png", ouDir, ouName); 
				cv::imwrite(outName, NormIRImg);
			}


			//1.3 Wyznacz histogram po przeskalowaniu (tylko w trybie wizualizacji)
			if (VISUAL== 1) { 
				 ::cv::calcHist(&NormIRImg, 1, 0, ::cv::Mat(), histoIR, 1, &histSize, 0);
				 
				 ::cv::normalize(histoIR, histoIR, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				::cv::imshow("A1.3 Histogram IR po skalowaniu", histImage); 	
				
			}

			// Koniec pomiaru czasu dla kroku A1
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 1: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
			// Koniec A1
			////////////////////////////
	

			////////////////////////////
			// A2. Obraz krawędziowy w IR ( w zasadzie będzie potrzebny jedynie dla szypułki wewnętrznej) 
			//
			t0=clock();		// zliczanie czasu
			start=time(0);
			
			// Canny
			//::cv::Canny(NormMonoImg, EdgeMImg, param.edgeThresh *255, param.edgeThresh*2, 3, false);
			// Jednak nie daje on kierunku krawędzi dla elementu krawędziowego
			// Dlatego potrzebna jest własna funkcja

			// Operator krawędziowy w IR
			::cv::Mat EdgeIRMImg, EdgeIRDirImg;
			// Funkcja globalna
			// Operator krawędziowy i pocienianie z progiem względnym, np. edgeThresh=0.2:
			EdgeDetection( param.lowThresh, param.edgeThresh, NormIRImg, EdgeIRMImg, EdgeIRDirImg);
			
			if (VISUAL== 1) { 
				::cv::Mat ShowImg = ::cv::Mat::zeros(20, 20, CV_8U);
				NormIRImg.copyTo(ShowImg, EdgeIRMImg);
				cv::imshow("A2. Krawędzie w IR", ShowImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/EdgeIR_%s.png", ouDir, ouName); 
					::cv::imwrite(outName, EdgeIRMImg);
				}
				ShowImg.release();
			}   

			// Koniec pomiaru czasu dla kroku 2
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 2: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
			// Koniec A2
			///////////////////////////////


			///////////////////////////////
			// A3. Wyznaczenie prostokątnego obszaru zainteresowania ROI IR
			// 
			
			t0=clock();		// zliczanie czasu
			start=time(0);
		
			//3.1 ROI w obrazie IR
			int IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax;
			int bbox[4];
			
			DetectROI2(NormIRImg, param.cornerMax, param.cornerMin, bbox);
			
			IRcolumnMin=bbox[0];
			IRcolumnMax=bbox[1];
			IRrowMin=bbox[2];
			IRrowMax=bbox[3];

			if (MONITOR == 1)
				cout<<"IR bbox:" << IRcolumnMin << ", "<< IRcolumnMax << ", " << IRrowMin << ", " << IRrowMax <<endl;

			// Pokaż obszar ROI w obrazie IR
			if(VISUAL==1)
			{
				::cv::Mat ShowROI_IRImg = NormIRImg.clone();
				ROD::DrawBox(ShowROI_IRImg, IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 255);
				cv::imshow("A3.1 ROI IR", ShowROI_IRImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/ROIIR_%s.png", ouDir, ouName); 
					::cv::imwrite(outName, ShowROI_IRImg);
				}
				ShowROI_IRImg.release();
			}
			

			/*
			// 3.2 ? Sprawdzenie detekcji konturu operatorami morfologicznymi 
			::cv::Mat dst2;
			::cv::Rect rect;
			::cv::Mat dst = NormMonoImg.clone();
			floodFill(dst, ::cv::Point(120, 150), ::cv::Scalar(230), &rect, ::cv::Scalar(3), ::cv::Scalar(3));

			//ErosionEllipse( NormMonoImg,  dst);
			// 3.3
			DilationEllipse( dst,  dst2);
			// Wizualizacja wyniku operacji morfologicznych
			if (VISUAL == 1) {
				cv::imshow( "A3.3 Fill", dst );
				cv::imshow( "A3.4 Dilation", dst2 );
			}
			

			// Koniec pomiaru czasu dla kroku 3
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 3: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
	
			
			/////////////////////////
			// A4. Znajdź dokładny brzeg obiektu (zamknięty kontur) w obrazie IR
			//
			// Kontur w obrazie IR
			int Cirx, Ciry; // środek masy konturu w obrazie IR
			::cv::Mat IrKontur = ::cv::Mat::zeros(DIRS, 2, CV_64FC1); // wektor współrzędnych punktów konturu
			::cv::Mat IrKontur1D = ::cv::Mat::zeros(DIRS, 1, CV_64FC1); // odległości punktów konturu od środka masy
			::cv::Mat IrGrad1D = ::cv::Mat::zeros(DIRS, 1, CV_64FC1); // gradient funkcji odległości
			//

			t0=clock();             // zliczanie czasu
			start=time(0);
			   
			int InitCy, InitCx; // środek obszaru ROI w obrazie IR
			double dIRcolumnMin, dIRcolumnMax, dIRrowMin, dIRrowMax;
			int radius; // dla końcowego promienia konturu
			int minRadius; // ograniczenie w procesie poszukiwania konturu
			
			// IRrowMin, IRrowMax, IRcolumnMin, IRcolumnMax to spodziewane ROI
			dIRcolumnMin = IRcolumnMin;
			dIRcolumnMax = IRcolumnMax;
			dIRrowMin = IRrowMin;
			dIRrowMax = IRrowMax;
			InitCy = round((dIRrowMin + dIRrowMax)/2);
			InitCx = round((dIRcolumnMin + dIRcolumnMax)/2);
			minRadius = round(((dIRcolumnMax - dIRcolumnMin) + (dIRrowMax - dIRrowMin))/ 4.0);

			// Funkcja szukania konturu w obrazie IR
			// Wynik zwracany jest w postaci zmiennych:
			// Cirx, Ciry, IrKontur, IrKont1D, GradIrKont1D
			// oryg. parametry = 0.2, 0.3
			KonturIR(0.15, 0.20, DIRS, InitCx, InitCy, minRadius, NormIRImg, 
				 Cirx, Ciry, IrKontur, IrKontur1D,  IrGrad1D);
						
			// Teraz właściwe BBox odpowiada ograniczeniom konturu IR:
			cv::minMaxLoc(IrKontur.col(0), &dIRcolumnMin, &dIRcolumnMax,0,0,cv::Mat());
			if (dIRcolumnMin <0)	dIRcolumnMin =0;
			if (dIRcolumnMax >= gx) dIRcolumnMax = gx-1;
			
			cv::minMaxLoc(IrKontur.col(1),&dIRrowMin,&dIRrowMax,0,0,cv::Mat());
			if (dIRrowMin <0)  dIRrowMin =0;
			if (dIRrowMax >= gy)  dIRrowMax = gy-1;
			
			// Zamiana na wartości całkowite
			IRcolumnMin = int(dIRcolumnMin);
			IRcolumnMax = int(dIRcolumnMax);
			IRrowMin = int(dIRrowMin);
			IRrowMax = int(dIRrowMax);

			// Wizualizacja konturu w IR w kolejnym kroku 5 

			// Koniec pomiaru czasu dla kroku 4
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 4: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;

			// Koniec kroku 4
			////////////////////////

			///////////////
			// 5. Detekcja szypułki odstającej w obrazie IR
			
			//
			t0=clock();         // początek zliczania czasu
			start=time(0);

			// 5.1 Analiza funkcji 1D konturu w obrazie IR
			cv::Mat Wycinek = cv::Mat::zeros(2,1,CV_32SC1);
			// Szukamy w całym zakresie konturu
			Wycinek.at<int>(0,0)= 0; 
			Wycinek.at<int>(1,0)= DIRS-1;

			// Funkcja zwraca wynik w postaci zmiennych globalnych:
			// [jestOgonIr, pozIrOgon, alfaIr, CirxNew, CiryNew, MaskaIrOgon, BBoxIr]
			// warto to zmienić
			
			int CirxN, CiryN;

			//Zmieniona wartość - oryginalnie 120.0 (wspPIK)
			::cv::Mat BBoxIr = OgonekIR(gy, gx, 5, Cirx, Ciry, Wycinek, IrKontur, IrKontur1D, IrGrad1D,
				CirxN, CiryN);
			
			// Po ewentualnym uwzględnieniu szypułki odstającej
			// zmienił się BBox obszaru samego jablka
				IRcolumnMin = BBoxIr.at<int>(0,0); // xmin
				IRcolumnMax = BBoxIr.at<int>(2,0); // xmax
				IRrowMin = BBoxIr.at<int>(1,0); // ymin
				IRrowMax = BBoxIr.at<int>(3,0); // ymax
			// Promień konturu:
				radius = BBoxIr.at<int>(4,0); // promień

			// Utwórz prostokatny ROI szypułki w IR
			cv::Mat roiSzypIr = cv::Mat::zeros(4,1,CV_32SC1);
			if (jestOgonIr == 1)
			{
				roiSzypIr.at<int>(0,0) =(pozIrOgon.at<int>(0,0) - 20); // xmin
				if (roiSzypIr.at<int>(0,0) < 0) 
					roiSzypIr.at<int>(0,0) = 0;
				roiSzypIr.at<int>(1,0) =(pozIrOgon.at<int>(1,0) - 20); // ymin
				if (roiSzypIr.at<int>(1,0) < 0) 
					roiSzypIr.at<int>(1,0) = 0;
				roiSzypIr.at<int>(2,0) =(pozIrOgon.at<int>(0,0) + 20); // xmax
				if (roiSzypIr.at<int>(2,0) >= gx) 
					roiSzypIr.at<int>(2,0) = gx-1;
				roiSzypIr.at<int>(3,0) =(pozIrOgon.at<int>(1,0) + 20); // ymax
				if (roiSzypIr.at<int>(3,0) >= gy) 
					roiSzypIr.at<int>(3,0) = gy-1;

				if (MONITOR == 1)
					cout <<"IrOgon: x="<< pozIrOgon.at<int>(0,0) <<", y=" ;
					cout << pozIrOgon.at<int>(1,0) << endl;
				
				
			}
			else
			{
				roiSzypIr.release(); 
			}

			// Wizualizacja konturu i ogonka w obrazie IR
			cv::Mat MonoE3Img;
			cv::Mat ShowMonoE3Img;
			std::vector<cv::Mat> MonoE3Img_planes(3);
			if (VISUAL == 1)
			{
				// Wizualizacja konturu w obrazie
				int x,y;
				//cv::split(F1ColorImg,MonoE3Img_planes);

				MonoE3Img_planes[0] = NormIRImg.clone();
				MonoE3Img_planes[1] = NormIRImg.clone();
				MonoE3Img_planes[2] = NormIRImg.clone();
				
			    for (int i=0;i<DIRS;i++)
			    {
			        y = int(IrKontur.at<double>(i,1) );
			        x = int(IrKontur.at<double>(i,0) );

			        if (MaskaIrOgon.at<uchar>(i,0) == 1)
			        {
			        	// ogonek
			        	MonoE3Img_planes[0].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[1].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[2].at<uchar>(y,x) = 255;
			        }
			        else
			        {
			            // kontur
			        	MonoE3Img_planes[0].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[1].at<uchar>(y,x) = 100;
			        	MonoE3Img_planes[2].at<uchar>(y,x) = 0;
			        }
			    }
				// Ewentualnie dorysuj ROI ogonka
				if (jestOgonIr == 1) {
					ROD::DrawBox(MonoE3Img_planes[0], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),100);
					ROD::DrawBox(MonoE3Img_planes[1], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),100);
					ROD::DrawBox(MonoE3Img_planes[2], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),255);

				// Dorysuj oś
					DrawLine(alfaIr.at<double>(0,0), 255, CirxN, CiryN, MonoE3Img_planes[0]); // red plane
					DrawLine(alfaIr.at<double>(1,0), 255, CirxN, CiryN, MonoE3Img_planes[0]); // red plane
					DrawLine(alfaIr.at<double>(0,0), 150, CirxN, CiryN, MonoE3Img_planes[1]); // g plane
					DrawLine(alfaIr.at<double>(1,0), 150, CirxN, CiryN, MonoE3Img_planes[1]); // g plane
					DrawLine(alfaIr.at<double>(0,0), 50, CirxN, CiryN, MonoE3Img_planes[2]); // b plane
					DrawLine(alfaIr.at<double>(1,0), 50, CirxN, CiryN, MonoE3Img_planes[2]); // b plane
				}

				// Narysuj prostokąt obejmujący obszar jabłka (bez szypułki) 
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 0);
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 100);
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 255);

				// Dorysuj środek
				ROD::DrawBox(MonoE3Img_planes[0], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 255);
				ROD::DrawBox(MonoE3Img_planes[1], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 0);
				ROD::DrawBox(MonoE3Img_planes[2], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 0);

				//
			    cv::merge(MonoE3Img_planes, ShowMonoE3Img );
			    cv::imshow("A5.1 Kontur i szypulka w IR", ShowMonoE3Img );
			    if (LOGS == 1)
			    {
			    	sprintf(outName, "%s/KonturIRSzyp_%s.png", ouDir, ouName);
			    	cv::imwrite(outName, ShowMonoE3Img );
			    }
			    
				ShowMonoE3Img.release();
				MonoE3Img_planes[0].release();
				MonoE3Img_planes[1].release();
				MonoE3Img_planes[2].release();

			}

			// 

		    //
		    // 5.2 Detekcja szypułki wewnętrznej lub granicznej
		    // - na podstawie obrazu krawędziowego i obrazu IR
			// ...
			int jestSzyp = 0; // inicjalizacja

		    
			// Koniec pomiaru czasu dla kroku 5
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 5: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;
			//
			//////////////////////


			//////////////////////
		    // 6. Analiza 2D obrazu IR (AW) - detekcja szypułki odstającej
		    //
		    t0=clock();             // zliczanie czasu
		    start=time(0);

		    // Koniec pomiaru czasu dla kroku 6
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 6: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;

			///////////////////
		    //% 7. Detekcja zagłębień w Mono (AW) - detekcja szypułki wewnętrznej / kielicha 
		    t0=clock();             // zliczanie czasu
		    start=time(0);

		    // Koniec pomiaru czasu dla kroku 7
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 7: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;


			/////////////////////
			// 8. Spakuj wyniki
			if (VISUAL == 1)
				cv::destroyAllWindows();
			
			ROD::ResultOD result = ROD::ResultOD();
			// TO DO: trzeba zapakowac wyniki do struktury zwracanej
			// ...
			result.cx = CirxN; // Uwaga: w układzie obrazu !
			result.cy = CiryN; //  - " -
			result.radius = radius;
			result.jestOgon = jestOgonIr; // odstająca szypułka
			result.jestSzyp = jestSzyp; // wewnętrzna 
			result.ogonCx = pozIrOgon.at<int>(0,0); // położenie nasady szypułki
			result.ogonCy = pozIrOgon.at<int>(1,0);
			result.bBox[0] = IRcolumnMin; // minX - prostokąt obejmujący jabłko
			result.bBox[2] = IRcolumnMax; // maxX
			result.bBox[1] = IRrowMin; // minY
			result.bBox[3] = IRrowMax; // maxY

			return result;
			}
	//////////////////////////////////////////
		
	////////////////////////////////////////////
	*/
