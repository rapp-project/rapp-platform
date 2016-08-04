#ifndef PARAMETERS
#define PARAMETERS


	const double defaultThresh = 0.2;
	
	class CParameters{
		// Wewnętrzne parametry ustawiane przez inżyniera serwisowego
	public:
        int lowThresh; // nieistotna siła krawędzi, np. 8    
		double edgeThresh; // adaptacyjny próg pocieniania krawędzi
            double cornerMax; // for ROI detection in IR image
            double cornerMin;
            double redColorThresh;
            double intensityThresh;
			int directions; // podział kąta pełnego
			
			
	public: 
		CParameters(int low = 8, double th = 0.2, double cmax= 100.0, double cmin = 50.0, double red = 48.0, double intens = 80.0, int dirs = 720)
		{
			this->lowThresh = 8;
			this->edgeThresh = th; // edge strength threshold
            this->cornerMax = cmax;
            this->cornerMin = cmin;
            this->redColorThresh = red;
            this->intensityThresh = intens;
			this->directions = dirs;
		}
		
	};

#endif
