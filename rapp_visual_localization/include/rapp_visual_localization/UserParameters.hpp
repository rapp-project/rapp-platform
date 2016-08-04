#ifndef USER_PARAMETERS
#define USER_PARAMETERS

	
	const int defaultTypeNum = 3; //  front = 1; back = 2; profile = 3;
	const int defaultViews = 3; // number of processed images

	class CUserParameters{
	private:
		int views;
		int poses; 
        
	public:
		CUserParameters(int v = defaultViews, int tt=defaultTypeNum)
		{
			this->views = v;
			this->poses = tt; 
		}

		void setViews(int v) { this->views = v; }
		int getViews() {return this->views; }
		void setPoses(int tt) { this->poses = tt; }
		int getPoses() {return this->poses; }
	};

#endif
                