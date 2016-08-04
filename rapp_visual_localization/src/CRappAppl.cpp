/// Aplikacja CRappAppl
///
/// Cel: 
/// 1) samolokalizacja w znanym pomieszczeniu "wizualna odometria",
/// w oparciu o "wizualną mapę pomieszczenia",
/// 2) z uwzględnieniem detekcji sylwetki człowieka.
/// 
/// Plik CRappAppl.cpp z główną funkcją programu
///
/// Funkcja main(): punkt wejścia do programu - aplikacji konsolowej
/// Progtram korzysta z biblioteki OpenCV
///
/// Autor: Włodzimierz Kasprzak
/// Modyfikacja: 27-07-2016
///
/// Klasy: 1) CVisOdom - deklaracja i definicja w plikach VisOdom.h i VisOdom.cpp
/// 2) CHuman - deklaracja i definicja w plikach Human.h i Human.cpp
///


//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

//
#include "rapp_visual_localization/Parameters.hpp"
#include "rapp_visual_localization/Properties.hpp"
#include "rapp_visual_localization/UserParameters.hpp"
#include "rapp_visual_localization/ResultOD.hpp"

#include "rapp_visual_localization/VisOdom.hpp"
#include "rapp_visual_localization/Human.hpp"

using namespace std;


///
// Funkcja main() - punkt wejścia do programu
///

int main(int argc, char * argv[])
{
/// Użytkowanie programu:
	/// 1: Przekazać i ustawić parametry programu
	/// 2: Utworzyć i zainicjalizować obiekty klas CVisOdom i CHuman
	/// 3: Utworzyć wizualną mapę pomieszczenia metodą CVisOdom.createMap()
	/// 4: Utworzyć model osoby metodę CHuman.createModel() 
	/// 5: Uruchomić metodę wizualnej odometrii CVisOdom.work()
		

	// Przełączniki pomocnicze: dla wizualizacji, monitorowania i "utrwalania" wyników 
	const int VIS = 0;  // wizualizacja w oknach obrazów z wynikami częściowymi
	const int MONIT = 1; // wyświetlanie tekstu komunikatów w terminalu
	const int LOG = 0; // utrwalenie wyników - zapis wyników na dysku do plików

	// Pomiar czasu dla oceny wydajności kroków programu
	double t0, elapsed;
	time_t start, end;

	// KROK 1
	// Parametry programu

	// Własności wynikają z dostępnej informacji o środowisku i kamerze
	double space_dx = 200.0; // rozmiar dostępnego człowiekowi pomieszczenia w cm
	double space_dz = 300.0; // rozmiar dostepnego człowiekowi pomieszczenia w cm
	double camera_y = 100.0; // wysokość środka układu kamery nad podłogą
	double camera_alfa = 0.0; // kąt nachylenia osi "z" kamery względem osi Z układu (podłogi)
	double camera_f = 0.01; // wartość ogniskowej kamery
	double camera_x = 100.0; // położenie początkowe (aktualne) robota
	double camera_z = 0.0; //   -- " --

	// ... // pobierz własności środowiska i pozycję kamery

	// Utwórz obiekt z własnościami środowiska
	CProperties pr = CProperties(space_dx, space_dz, camera_y, camera_alfa, camera_f, camera_x, camera_z); 
	
	// Obiekt z parametrami analizy obrazu - dobieranymi podczas instalacji
	CParameters pa = CParameters(16, 0.2, 0.5, 0.20, 25, 42, 720); // low edge, edge threshold, corner max, corner min
		// red color thresh min, intensity thresh min, direction's number
	
	
	
	// Obiekty potrzebne do analizy obrazów
	CVisOdom objVO; // obiekt dla wizualnej odometrii
	CHuman objH; // obiekt dla modelowania i detekcji sylwetki

	// Katalog WE: tryb off-line (testowanie)
	char inDirHuman[] = "C:/RAPP/NAO/human"; // katalog obrazów sylwetki
	char inDir[] = "C:/RAPP/NAO/"; // katalog nawigacji
	char inTest[] = "C:/RAPP/NAO/vodom1"; // katalog obrazów pomiarowych (wizualna odometria) 
	char inMap[] = "C:/RAPP/NAO/dane1"; // katalog obrazów dla modelu (tworzenie mapy)
	
	char MapXml[] = "work7ModelColorMax.xml"; // plik do zapisu tworzonej wizualnej mapy pomieszczenia
	char MeasXml[] = "work7MeasureColorMax.xml"; // plik z przygotowaną mapą obserwacji (tryb off-line) 
	//char MapXml[] = "work5Dane1ModelColorMax.xml"; // plik do zapisu tworzonej wizualnej mapy pomieszczenia
	//char MeasXml[] = "work5MeasureColorMax.xml"; // plik z przygotowaną mapą obserwacji
	char HumanXml[] = "humanModel.xml"; // plik z modelem sylwetki

	//Katalog WY: tryb off-line
	char ouDirHuman[] = "C:/RAPP/NAO/logHuman";
	char ouDir[] = "C:/RAPP/NAO/log";

	char *inH = &inDirHuman[0];
	char *ouH = &ouDirHuman[0];
	char *inT = &inTest[0];
	char *inM = &inMap[0];
	char *ou = &ouDir[0];
	
	std::cout<<" CRappAppl: l.parametrow = "<< argc << std::endl;

	// Analiza parametrów wywołania programu
	if( argc < 3) // jeśli w wywołaniu brak argumentów podających pierwszy katalog 
    {
     std::cout <<" Argumenty: katalog_map katalog_test katalog_log" << std::endl;
	 std::cout <<" Ustawiam katalogi domyslne" << std::endl;
	}
	else
	{
		inM = argv[1];
	}
	
	if( argc < 4) // jeśli w wywołaniu brak argumentów podających drugi katalog 
    {
		cout <<" Argumenty: katalog_map katalog_test katalog_log" << endl;
		cout <<" Ustawiam katalogi domyslne" << endl;
	}
	else
	{
		inT = argv[2];
	}

	if( argc < 5) // jeśli w wywołaniu brak argumentów podających drugi katalog 
    {
		cout <<" Argumenty: katalog_map katalog_test katalog_log" << endl;
		cout <<" Ustawiam katalogi domyslne" << endl;
	}
	else
	{
		ou = argv[3];
	}

	//
	t0=clock();		// zliczanie czasu
	start=time(0);

	// KROK 2:
	// Inicjalizacja obiektów klas CVisOdom i CHuman:
	try
	{
		MType method = M_COLORMAX; // typ zestawu cech obrazu

		int numImgs=3; // liczba obrazów sylwetki
		// Obiekt z parametrami definiowanymi przez użytkownika
		CUserParameters	upa = CUserParameters(numImgs, numImgs); 
		// liczba obrazów sylwetki i tworzonych modeli sylwetki 
		
		// Utwórz obiekt klasy CVisOdom
		objVO = CVisOdom(inM , inT, ou, pr, pa, upa, VIS, MONIT, LOG);
	
		// Utwórz obiekt klasy CHuman
		objH = CHuman(inH, ouH, pr, pa, upa, VIS, MONIT, LOG);

		// Analiza obrazów

		// Krok 3: Utworzenie wizualnej mapy pomieszczenia
		int numx=19; // liczba kolumn w mapie
		int numz=41; // liczba rzędów w mapie
		int numd=4; // liczba kierunków
		int vgrid = 5; // (nieistotne)

		objVO.initMap(numx, numz, numd, vgrid); // inicjalizuj dane związane z rozmiarem mapy
		objVO.initFeatureMemory(method); // alokuj pamięć dla reprezentacji cech obrazów (w fazie tworzenia mapy)
		objVO.initFeatureScaling(method); // inicjalizuj wagi cech (potrzebne zarówno w fazie uczenia jak i testowania/pracy)
		
		// Utworzenie mapy i wypisanie jej do pliku xml
		objVO.createMap(method, MapXml, MeasXml, 0, 0); // parameter 1 określa typ zestawu cech obrazu
		// parametr 2: nazwa pliku z istniejącą mapą
		// parametr 3: nazwa pliku z pomiarami dla wszystkich stanów (w trybie testowania/symulacji)
		// parametr 4: tworzenie mapy (1) lub pomiń to (0) bo plik xml z mapą już istnieje (0)
		// parametr 5: na potrzeby testowania/symulacji utwórz wcześniej mapę cech dla wszystkich aktualnych pomiarów (1) lub pomiń (0)  

		// Krok 4: Utworzenie modelu aktualnej sylwetki
		objH.createModel(inH, ouH, HumanXml, 0, VIS, MONIT, LOG);
		// parametr 3: =0 : wczytaj model sylwetki z domyslnego pliku; =1: utwórz na nowo model sylwetki

		// Krok 5: Testowanie (symulacja, off-line) lub rzeczywista wizualna odometria (tryb on-line)
		
		int MAXITER = 100; // dopuszczalna maksymalna liczba kroków jednego procesu nawigacji
		int TESTNUM = 100; // zadana liczba testów nawigacji

		::cv::Mat results;
  
		// Metod "work" może być podstawą wątku ale nie jest to konieczne
		// W trybie "test/symulacja": TESTNUM oznacza liczbę testów (ścieżek)
		// W trybie "praca on-line": TESTNUM będzie liczbą ostatnich pamiętanych wyników
		results = objVO.work(TESTNUM, MAXITER, method, objH, MapXml, MeasXml, true);
		// ostatni parametr: true = tryb off-line (testowanie w warunkach symulacji pomiarów)
		// false = tryb on-line (rzeczywista nawigacja z samolokalizacją)
			
		//
		// ewentualnie skorzystaj z wyników testów lub pracy on-line
		// ...
		// ...
		// ...		

	}
	catch(cv::Exception &e){
	    cout<<"Exception"<<endl;
		cerr << e.what() <<endl;
	}
	catch(std::exception &e){
		cerr << e.what() <<endl;
	}
	catch(char const * e){
		cerr << e <<endl;

	}
		
	end=time(0);
	elapsed= (double)(clock() - t0) ;

	
	cout<< "t0= " << t0 << " t1= " << clock() << endl;
	cout << "Czas wykonania main():" << endl;
	cout << elapsed<< " ms" <<  endl;
	cout << end-start << " s" <<  endl;
		
	
	return 0;
} 

// Koniec pliku CRappAppl.cpp
