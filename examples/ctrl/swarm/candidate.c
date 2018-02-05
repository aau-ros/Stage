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


float bias[11]={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.5176322f, 2.03293f, 0.6761036f};

float randombias[11]={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float weight[11][11]={{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.8680084f, 1.1554936f, -0.9806155f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.3365494f, 0.5880204f, 0.10033585f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.45760846f, -0.9922776f, -0.6150167f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.1525373f, 0.90615773f, 1.4134395f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.059792906f, -2.0338578f, 0.15850064f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.16937117f, -2.1041234f, -1.7051442f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.84000266f, -1.3124723f, 0.5518271f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.49362487f, 0.50003403f, 0.6370157f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.012453616f, -0.51089895f, -1.1126348f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.587156f, -2.426098f, 0.55471236f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.41363388f, -0.19404039f, 1.7172735f}};

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
