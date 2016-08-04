#ifndef RESULT_OD
#define RESULT_OD

#include <opencv2/opencv.hpp>    

	struct CResultOD{
		int isHuman; // sylwetka wykryta (1) lub nie (0)
		int poseId; // klasa (poza)
		int cx; // wspĂłĹ‚rzÄ™dna X Ĺ›rodka masy sylwetki czĹ‚owieka
		int cy; // wspĂłĹ‚rzÄ™dna Y Ĺ›rodka masy sylwetki czĹ‚owieka w obrazie
		int bBox[4]; // [minx miny maxx maxy] prostokÄ…t obejmujÄ…cy
		int X[8]; // wsp. X poĹ‚oĹĽenia czÄ™Ĺ›ci sylwetki
		int Y[8]; // wsp. Y poĹ‚oĹĽenia czÄ™Ĺ›ci sylwetki
		::cv::Mat cImage; // wycinek obrazu sylwetki
	};

#endif
