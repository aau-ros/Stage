#include <math.h>

class Result{
public:
  float output[1];
  Result(float outp[], long outputsize){
    long i;
	for (i=0L;i<outputsize; i=i+1){
	  output[i]=outp[i];
	}
  }
};


float bias[11]={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -2.5072455f, 1.0410601f, -0.5844688f};

float randombias[11]={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float weight[11][11]={{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.6362177f, 0.866917f, -1.2217003f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.375616f, -0.72870064f, 3.0547278f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -2.3183353f, 0.20539147f, 0.9318697f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.5054164f, 2.3493996f, 1.0253928f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.1544592f, 0.46617234f, 2.5891843f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.2255348f, -1.1654584f, -1.1058412f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.012082368f, 0.5894197f, 2.0529456f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.1221751f, -1.158708f, -1.9888122f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.28990307f, -0.8350679f, 0.24195716f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.2402443f, -0.7372633f, 0.7372331f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.1246126f, 1.5533552f, -0.72174925f}};

float netOutput[11];

long long int seed=12345LL;

float sigmoidActivate(float x) {
	float y=(float)(1.0f / (1.0f + exp(-x)));
	return y;
}

float linearActivate(float x){
	if (x >= 1)
		{return 1;}
	else {
		if (x <= 0)
			{return 0;}
		else
			{return x;}
		}
}

Result getStep(float netInput[], long inputsize){
    long i;
	long j;
	float activation [11];
	for (i=0L; i < 11L; i=i+1){
		activation[i]=0.0f;
	}
    for (i=0L; i < inputsize; i=i+1) {
      netOutput[i]=netInput[i];
    }
	for (i=8L; i < 11L; i=i+1) {
      float sumValue=0.0f;
	  for (j=0L; j < 11L; j=j+1) {
        sumValue=sumValue+weight[j][i]*netOutput[j];
      }
	  activation[i]=bias[i]+sumValue;
    }
	float outputVector [1];
	for (i = 8L; i < 11L; i=i+1) {
      netOutput[i]=linearActivate(activation[i]);
    }
	for (i = (11L - 1L); i < 11L; i=i+1) {
	  j = i - (11L - 1L);
      outputVector[j]=netOutput[i];
    }
	Result r(outputVector,1L);
	return r;
}

Result getOutput(float netInput[],long inputsize){
  long i;
  for (i=0L; i < 11L; i=i+1) {
    netOutput[i]=0;
  }
  for (i=0L; i < 2L - 1; i=i+1) {
    getStep(netInput,inputsize);
  }
  return getStep(netInput,inputsize);
}
