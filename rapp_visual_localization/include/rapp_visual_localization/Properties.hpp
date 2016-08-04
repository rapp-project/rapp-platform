#ifndef PROPERTIES
#define PROPERTIES
	
	class CProperties{
	private:
		double spaceWidth;
		double spaceDepth;
		double cameraY0;
		double cameraAlfa;
		double cameraF;
		double cameraX0;
		double cameraZ0;
            
	public:
        CProperties(double s_dx, double s_dy, double c_y, 
			 double c_alfa, double c_f, double c_x, double c_z)
		{
			this->spaceWidth = s_dx; 
			this->spaceDepth = s_dy; 
			this->cameraY0 = c_y;
			this->cameraAlfa = c_alfa;
			this->cameraF = c_f;
			this->cameraX0 = c_x;
			this->cameraZ0 = c_z;

		}
		
		CProperties()
		{}


		double getCameraY0()  { return this->cameraY0; }
		double getCameraX0()  { return this->cameraX0; }
		double getCameraZ0()  { return this->cameraZ0; }
		double getCameraAlfa()  { return this->cameraAlfa; }
		double getCameraF()  { return this->cameraF; }

		void setCamera(double c_y, double c_alfa, double c_f, double c_x, double c_z)
		{
			this->cameraY0 = c_y;
			this->cameraAlfa = c_alfa;
			this->cameraF = c_f;
			this->cameraX0 = c_x;
			this->cameraZ0 = c_z;
		}

	};

#endif

 