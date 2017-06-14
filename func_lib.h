#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "gptcall.h"

#define pi 3.1416
#define  NN 2
/*                       Definition                             */
/* ------------------------------------------------------------ */
#define NUM_LEARN 120000      /* Learning number of NN          */
#define NUM_SAMPLE 2+1        /* Sample number of training data and initial data */
#define NUM_INPUT 2           /* Number of input layer          */
#define NUM_HIDDEN 21         /* Number of hidden layer         */
#define NUM_OUTPUT 7          /* Number of output layer         */
#define ALPHA 0.5             /* Learning rate                  */
#define ERROR_THRESHOLD 0.00000005/* Threshold error                */
#define SIGMOID_GAIN 2.0      /* Gain of sigmoid function       */
#define TSTEP 0.001           /* time step [sec]                */
#define LAST_TIME 20.000      /* simulation time [sec]          */
#define NUM_RANDOM 32768.0    
/* ------------------------------------------------------------ */
/* definition of parameters */
/* (1) reference signal */
struct ref {
	float A, T, AA;
};
/* (2) model */
struct IVPmodel {
	float ma, mt; //mass 
	float k1; // spring stiffness
	float k2;
	float g; // gravity
	float l; // length of pole
	float kp_osc, kv_osc; // coefficient of oscillator
};
/* (3) Neural Oscllator */
struct CPG {
	float Tr, Ta, b, cc, a1, a2, hi;
};

/* (4) all reference parameter */
struct input_param {
	float A[NUM_SAMPLE], T[NUM_SAMPLE];
};

/* ------------------------------------------------------------ */

/* parameter initialization                                     */
/* ------------------------------------------------------------ */
/*
struct ref refZMP = {0.12, 2.0, 0.09};
struct IVPmodel model={5.7455, 0.001, 200.0, 1.0, 9.81, 0.313, 1.2, 0.15};
struct CPG NOparam = {0.8, 0.8*2.0, 0.8, 1.2, 2.5, 2.5, 1/refZMP.A*1.5};
*/

struct ref refZMP = {0.09, 1.0, 0.09};
struct IVPmodel model={5.7455, 0.001, 250.0, 1.0, 9.81, 0.313, 1.5, 0.15};
struct CPG NOparam = {0.13, 0.13*2.0, 1.0, 0.6, 2.5, 2.5, 1/0.09};
struct input_param INPUTparam = {{0.0, 0.0, 0.0, 0.0},{0.0, 0.0, 0.0, 0.0}};
/*
struct ref refZMP = {0.02, 1.3, 0.09};
struct IVPmodel model={5.7455, 0.001, 240.0, 1.0, 9.81, 0.313, 1.2, 0.15};
struct CPG NOparam = {1.8, 1.8*2.0, 0.6, 1.0, 2.5, 2.5, 1/refZMP.A*1.5};
*/
/* ------------------------------------------------------------- */

/* initialization for NN */

/* t_input : input training data, t_output: teach training data */
float t_input[NUM_SAMPLE][NUM_INPUT], t_output[NUM_SAMPLE][NUM_OUTPUT];
/* signals scaled training data */
float scaling_input[NUM_SAMPLE][NUM_INPUT+1], scaling_output[NUM_SAMPLE][NUM_OUTPUT+1];
/* iout : each neuron output in input layer                     */
/* hout : each neuron output in hidden layer                    */
/* oout : each neuron output in output layer                    */
float iout[NUM_INPUT+1], hout[NUM_HIDDEN], oout[NUM_OUTPUT+1];
/* Neural Network output */
float NN_output[NUM_SAMPLE][NUM_OUTPUT+1];
/* w_ih : weight between input layer and hidden layer           */
/* w_ho : weight between hidden layer and output layer          */
float w_ih[NUM_INPUT+1][NUM_HIDDEN], w_ho[NUM_HIDDEN][NUM_OUTPUT+1];
/* thresh_h : threshold in hidden layer                         */
/* thresh_o : threshold in output layer                         */
float thresh_h[NUM_HIDDEN], thresh_o[NUM_OUTPUT+1];
/* back propagation value in hidden and output layers           */
float h_back[NUM_HIDDEN], o_back[NUM_OUTPUT+1];
/* Network error                                                */
float NNerror[NUM_LEARN][NUM_SAMPLE];

float NNparam[NUM_OUTPUT];
/* initial parameter */
float ini_parami[NUM_INPUT], ini_paramt[NUM_OUTPUT];
/* ------------------------------------------------------------ */

float max(float a, float b)
{
	float max_value;
	if (a >= b)
		max_value = a;
	else
		max_value = b;
	return max_value;
}
float min(float a, float b)
{
	float min_value;
	if (a <= b)
		min_value = a;
	else
		min_value = b;
	return min_value;
}

float square(struct ref ref_sq, float time)
{
	float REtime, output;
	float A, T, AA; // for ref ZMP

	A = ref_sq.A; T = ref_sq.T; AA = ref_sq.AA;

	REtime = fmod(time, T);
	if (REtime <= T/2.0)
		output = 1;
	else
		output = -1;
	return output;
}

/* -*-*-*-*- function : differencial equation -*-*-*-*- */

void Dif_eq(struct ref ref_param, struct IVPmodel  model, struct CPG param, float x[], float dotx[], float t)
/* change dotx[5] */
{   
	float zmp_d, osc_out, NO_controller;
	float A, T, AA; // for ref ZMP
	float ma, mt, k1, k2, l, g, kp_osc, kv_osc;
	float Tr, Ta, b, cc, a1, a2, hi;

	/* exchange parameter */
	// (1) refZMP
	A = ref_param.A; T = ref_param.T; AA = ref_param.AA;
	// (2) IVPmodel
	ma = model.ma; mt = model.mt;
	k1 = model.k1; k2 = model.k2;
	l = model.l;
	g = model.g;
	kp_osc = model.kp_osc; kv_osc = model.kv_osc;
	// (3) CPG
	Tr = param.Tr; Ta = param.Ta; 
	b = param.b; cc = param.cc;
	a1 = param.a1; a2 = param.a2; hi = param.hi;
	/*--------------------*/

	/* Reference ZMP */
	zmp_d = A*square(ref_param, t);
	/*---------------*/

	/* Equation of Neural Oscillator */
	dotx[0] = (-x[0]-a1*max(x[2], 0.0)-b*x[1]+cc-hi*max(x[4], 0.0))/Tr;
	dotx[1] = (-x[1]+max(x[0], 0.0))/Ta;
	dotx[2] = (-x[2]-a2*max(x[0], 0.0)-b*x[3]+cc+hi*min(x[4], 0.0))/Tr;
	dotx[3] = (-x[3]+max(x[2], 0.0))/Ta;
	AA = 1.0*AA;
	osc_out = AA*(max(x[0], 0.0)-max(x[2], 0.0)); // oscillator output
	/*-------------------------------*/

	/* Neural Oscillator controller */
	NO_controller = 2.0*k1/ma*(k2*osc_out-kp_osc*x[4]-kv_osc*x[5]);
	/*------------------------------*/

	/* IVP model equation */
	dotx[4] = x[5];
	dotx[5] = g/l*(x[4]-zmp_d)+NO_controller;
	//dotx[5] = g/l*(x[4]-zmp_d)+2.0*k1/ma*(-x[4])-10*x[5]+100*osc_out;
	/*--------------------*/

}

/* -*-*-*-*- function : Runge Kutta -*-*-*-*- */
void Runge_Kutta(struct ref ref_param, struct IVPmodel model, struct CPG param, float x[], float t)
/* change x[5] */
{
	/* definition of parameters */
	float k1[5], k2[5], k3[5], k4[5];
	float dotx[5];
	float REt, REx[5];
	int i;

	/* k1[] */
	Dif_eq(ref_param, model, param, x, dotx, t); /* get dotx[5] */
	for(i=0;i<6;i++){ // 6:state number
		k1[i] = TSTEP*dotx[i];
		REx[i] = x[i] + k1[i]/2.0;
	}
	REt = t + TSTEP/2.0;

	/* k2[] */
	Dif_eq(ref_param, model, param, REx, dotx, REt); /* get dotx[5] */
	for(i=0;i<6;i++){ // 6:state number
		k2[i] = TSTEP*dotx[i];
		REx[i] = x[i] + k2[i]/2.0;
	}
	REt = t + TSTEP/2.0;

	/* k3[] */
	Dif_eq(ref_param, model, param, REx, dotx, REt); /* get dotx[5] */
	for(i=0;i<6;i++){ // 6:state number
		k3[i] = TSTEP*dotx[i];
		REx[i] = x[i] + k3[i];
	}
	REt = t + TSTEP;

	/* k4[] */
	Dif_eq(ref_param, model, param, REx, dotx, REt); /* get dotx[5] */
	for(i=0;i<6;i++){ // 6:state number
		k4[i] = TSTEP*dotx[i];
	}

	/* change x[] */
	for(i=0;i<6;i++){ 
		x[i] = x[i] + (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i])/6.0;
	}
}

/* -*-*-*-*- function : joint angle generating for rolling motion -*-*-*-*- */
void roll_angle(float COMpos, float joint[])
{	
	/* hip join angle */
	joint[0] = COMpos*180.0/pi; joint[1] = COMpos*180.0/pi;
	/* ankle joitn ankgle */
	joint[2] = -COMpos*180.0/pi; joint[3] =-COMpos*180.0/pi;
}

/* -*-*-*-*- function : joint angle generating for pitching motion -*-*-*-*- */
void pitch_angle(float COMx, float joint[]){
	float COMz = 0.187939, A = 0.10, B = 0.10; /* [m] */
	float C, alp;

	/* inverse kinematics */
	/* method1 */
	C = sqrt(COMz*COMz+COMx*COMx);
	joint[1] = -acos((A*A+B*B-C*C)/(2.0*A*B))+pi;
	alp = asin(A*sin(pi-joint[1])/C);
	joint[2] = - atan2(COMx, COMz) - alp;
	joint[0] = - (joint[1] + joint[2])*180.0/pi; 
	joint[1] = joint[1]*180.0/pi; joint[2] = joint[2]*180.0/pi;
	/* method2 *//*
	C = -sqrt((COMx*COMx+COMz*COMz+A*A+B*B)*(COMx*COMx+COMz*COMz+A*A+B*B)-2*((COMx*COMx+COMz*COMz)*(COMx*COMx+COMz*COMz)+A*A*A*A+B*B*B*B));
	joint[0] = -(pi/2.0 - atan2(COMz,COMx)-atan2(C,(COMx*COMx+COMz*COMz+A*A-B*B)))*180/pi;
	joint[1] = -(atan2(C,(COMx*COMx+COMz*COMz-A*A-B*B)))*180/pi;
	joint[2] = -(joint[0] + joint[1]);
	*/
}


/* -*-*-*-*- function : Make_Graph(using GNUPLOT) -*-*-*-*- */
void mk_graph(){
	int n[NN] ;
	int i;
	char *data1={"data/square.dat"};
	char *data2={"data/state.dat"};
    
	for( i=0 ; i<NN ; i++ ){
		n[i] = gptopen("");
	}
	/* plot1 */
	gptsend(n[0],"plot \"%s\" using 1:2 with lines,\ \"%s\" using 1:6 with lines\n",data1,data2);
	gptsend(n[0],"set grid\n"); //grid
	gptsend(n[0],"replot\n") ;
	
	/* plot2 */
	/*
	gptsend(n[1],"plot \"%s\" using 1:2 with lines,\ \"%s\" using 1:7 with lines\n",data1,data2);
	gptsend(n[1],"set grid\n"); //grid
	gptsend(n[1],"replot\n");
	*/
	/* save image file [*.png] */
	gptsend(n[0],"set term png color\n");
	gptsend(n[0],"set output 'output\desire_COM.png'\n"); 
	gptsend(n[0],"replot\n");
	/*
	gptsend(n[1],"set term png color\n");
	gptsend(n[1],"set output 'test.png'\n"); 
	gptsend(n[1],"replot\n");
	*/
	getchar();
   	gptcloseall();
}

/* sigmoid function */
float sigmoid(float y)
{
	return (float)(1.0/(1.0 + exp((double)(SIGMOID_GAIN * (-y)))));
}

/* read and display training data */
void read_data(void)
{
	int isample, i;
	FILE *training, *initial;
	
	printf("\n-------------- FUNCTION : read_data ----------------\n");
	initial = fopen("initial.dat", "r");
	training = fopen("training.dat", "r");

	/* (1) initial data  */
	if (initial == NULL){
		printf("ERROR : Can't open initial.dat");
		exit(0);
	}
	else {
		/* read initial data */
		for (i = 0; i < NUM_INPUT; i++)
			fscanf(initial, "%f", &ini_parami[i]);
		for (i = 0; i < NUM_OUTPUT; i++)
			fscanf(initial, "%f", &ini_paramt[i]);
		/* display initial data */
		printf("initial data \n");printf("INPUT : ");
		for (i = 0; i < NUM_INPUT; i++)
			printf(" %f ", ini_parami[i]);
		printf("\n");printf("OUTPUT : ");
		for (i = 0; i < NUM_OUTPUT; i++)
			printf(" %f ", ini_paramt[i]);
		printf("\n");
	}
	fclose(initial);
	/* (2) training data */
	if (training == NULL){
		printf("ERROR : Can't open training.dat");
		exit(0);
	}
	else {
		/* read training data */
		for (isample = 0; isample < NUM_SAMPLE; isample++){
			if (isample < NUM_SAMPLE-1){
				for (i = 0; i < NUM_INPUT; i++)
					fscanf(training, "%f", &t_input[isample][i]);
				for (i = 0; i < NUM_OUTPUT; i++)
					fscanf(training, "%f", &t_output[isample][i]);
			}
			else{
				for (i = 0; i < NUM_INPUT; i++)
					t_input[isample][i] = ini_parami[i];
				for (i = 0; i < NUM_OUTPUT; i++)
					t_output[isample][i] = ini_paramt[i];
			}
		}
		/* display training data */
		for (isample = 0; isample < NUM_SAMPLE; isample++){
			printf("Training data NO. %d : \n", isample+1);
			printf("INPUT : ");
			for (i = 0; i < NUM_INPUT; i++)
				printf(" %f ", t_input[isample][i]);
			printf("\n");printf("OUTPUT : ");
			for (i = 0; i < NUM_OUTPUT; i++)
				printf(" %f ", t_output[isample][i]);
			printf("\n");
		}
		fclose(training);
	}
}
/* parameter substitution */
void substi_param(float ttparam[])//(struct IVPmodel model, struct CPG param, float ttparam[])
{
	NOparam.Tr = ttparam[0]; NOparam.Ta = NOparam.Tr*2.0; NOparam.b = ttparam[1];
	NOparam.cc = ttparam[2]; NOparam.hi = ttparam[3];
	model.k1 = ttparam[4]; model.kp_osc = ttparam[5]; model.kv_osc = ttparam[6];
}
/* set up all input parameters */
void all_input(void)//(struct input_param INPUTparam)
{
	INPUTparam.A[0] = ini_parami[0]; INPUTparam.A[1] = t_input[0][0]; INPUTparam.A[2] = t_input[1][0];
	INPUTparam.T[0] = ini_parami[1]; INPUTparam.T[1] = t_input[0][1]; INPUTparam.T[2] = t_input[1][1];
}
/* wight and threshold initialization */
void init_wt(void)
{
	int i, j;

	srand(time(NULL));
	/* initialization : weight */
	for (i = 0; i < NUM_INPUT+1; i++){
		for (j = 0; j < NUM_HIDDEN; j++){
			w_ih[i][j] = (float)(rand() / (RAND_MAX+1.0));
		}
	}
	for (i = 0; i < NUM_HIDDEN; i++){
		for (j = 0; j < NUM_OUTPUT+1; j++){
			w_ho[i][j] = (float)(rand() / (RAND_MAX+1.0));
		}
	}
	/* initialization : threshold */
	for (i = 0; i < NUM_HIDDEN; i++) 
		thresh_h[i] = (float)(rand() / (RAND_MAX+1.0));
	for (i = 0; i < NUM_OUTPUT+1; i++) 
		thresh_o[i] = (float)(rand() / (RAND_MAX+1.0));
}

/* scaling NN training data */
void scale_NN(void)
/* (1) scaling with inial parameter         */
/* (2) scaling with a sum of each parameter */
{
	int i, j;
	float tempi, tempt, sumi[NUM_SAMPLE], sumt[NUM_SAMPLE];
	float new_parami[NUM_SAMPLE][NUM_INPUT], new_paramt[NUM_SAMPLE][NUM_OUTPUT];
	
	printf("\n-------------- FUNCTION : scale_NN ----------------\n");
	/* (1) scaling with inital parameter         */
	for (i = 0; i < NUM_SAMPLE; i++){
		for (j = 0; j < NUM_INPUT; j++){
			new_parami[i][j] = t_input[i][j]/ini_parami[j];
		}
		for (j = 0; j < NUM_OUTPUT; j++){
			new_paramt[i][j] = t_output[i][j]/ini_paramt[j];
		}
	}

	/* (2) scaling with a sum of each parameter */
	for (i = 0; i < NUM_SAMPLE; i++){
		tempi = 0; tempt = 0;
		for (j = 0; j < NUM_INPUT+1; j++){
			if (j > NUM_INPUT-1)
				scaling_input[i][j] = 1;
			else
				scaling_input[i][j] = new_parami[i][j];
			tempi = tempi + scaling_input[i][j];
		}
		sumi[i] = tempi;
		for (j = 0; j < NUM_OUTPUT+1; j++){
			if (j > NUM_OUTPUT-1)
				scaling_output[i][j] = 1;
			else
				scaling_output[i][j] = new_paramt[i][j];
			tempt = tempt + scaling_output[i][j];
		}
		sumt[i] = tempt;
	}
	for (i = 0; i < NUM_SAMPLE; i++){
		printf("Scaled training data NO. %d : \n", i+1);
		printf("INPUT : ");
		for (j = 0; j < NUM_INPUT+1; j++){
			scaling_input[i][j] = scaling_input[i][j]/sumi[i];
			printf("%f ", scaling_input[i][j]);
		}
		printf("\n");printf("OUTPUT : ");
		for (j = 0; j < NUM_OUTPUT+1; j++){
			scaling_output[i][j] = scaling_output[i][j]/sumt[i];
			printf("%f ", scaling_output[i][j]);
		}
		printf("\n");
	}
}

/* NN learning function */
void learn_NN(void)
{
	int i, j, k, l;
	float errorV, max_error, net_input, alpha;

	alpha = ALPHA;

	while (1){
		for (l = 0; l < NUM_LEARN; l++) {
			max_error = 0;
			for (k = 0; k < NUM_SAMPLE; k++){
				/* (1) forward calculation */
				for (i = 0; i < NUM_HIDDEN; i++){
					net_input = 0;
					for (j = 0; j < NUM_INPUT+1; j++){
						net_input = net_input + w_ih[j][i] * scaling_input[k][j];
					}
					hout[i] = sigmoid(net_input - thresh_h[i]);
				}
				for (i = 0; i < NUM_OUTPUT+1; i++){
					net_input = 0;
					for (j = 0; j < NUM_HIDDEN; j++){
						net_input = net_input + w_ho[j][i]*hout[j];
					}
					oout[i] = sigmoid(net_input- thresh_o[i]);
					NN_output[k][i] = oout[i];
				}
				/* (2) Network error */
				errorV = 0;
				for (i = 0; i < NUM_OUTPUT+1; i++){
					errorV = errorV + (scaling_output[k][i] - oout[i]) * (scaling_output[k][i] - oout[i]);
				}
				NNerror[l][k] = errorV / (float)NUM_OUTPUT;
				if (NNerror[l][k] > max_error) max_error = NNerror[l][k];
				
				/* (3) back propagation */
				for (i = 0; i < NUM_OUTPUT+1; i++)
					o_back[i] = (oout[i] - scaling_output[k][i]) * ((float)1.0 - oout[i])*oout[i];
				for (i = 0; i < NUM_HIDDEN; i++){
					 net_input = 0;
					 for (j = 0; j < NUM_OUTPUT+1; j++){
						  net_input = net_input + w_ho[i][j] * o_back[j];
					 }
					 h_back[i] = net_input * ((float)1.0 - hout[i]) * hout[i];
				}
				/* (4) weight and threshold Modification */
				for (i = 0; i < NUM_INPUT+1; i++){
					for (j = 0; j < NUM_HIDDEN; j++)
						w_ih[i][j] = w_ih[i][j] - alpha * scaling_input[k][i] * h_back[j];
				}
				for (i = 0; i < NUM_HIDDEN; i++){
					for (j = 0; j < NUM_OUTPUT+1; j++)
						w_ho[i][j] = w_ho[i][j] - alpha * hout[i] * o_back[j];
				}
				for (i = 0; i < NUM_HIDDEN; i++)
					thresh_h[i] = thresh_h[i] - alpha * h_back[i];
				for (i = 0; i < NUM_OUTPUT+1; i++)
					thresh_o[i] = thresh_o[i] - alpha * o_back[i];
			}
			if (max_error < ERROR_THRESHOLD) break;
		}
		if (max_error < ERROR_THRESHOLD) break;
	}

	/* display */
	printf("\n-------------- FUNCTION : learn_NN ----------------\n");
	printf("Learnin result : \n");
	for (k = 0; k < NUM_SAMPLE; k++){
		printf("NN output NO %d : ", k+1);
		for (i = 0; i < NUM_OUTPUT+1; i++)
			printf("%f ", NN_output[k][i]);
		printf("\n");
	}
}
/* test input generating */
void testINPUT(struct ref ref_param, float testinput[])
{
	int i;
	float temp, temp_param[NUM_INPUT];
	float input[NUM_INPUT];

	/* input signal substitution */
	input[0] = ref_param.A; input[1] = ref_param.T;

	/* scaling */
	for (i = 0; i < NUM_INPUT; i++){
		temp_param[i] = input[i]/ini_parami[i];
	}
	temp = 0;
	for (i = 0; i < NUM_INPUT+1; i++){
		if (i > NUM_INPUT-1)
			testinput[i] = 1;
		else
			testinput[i] = temp_param[i];
		temp = temp + testinput[i];
	}
	for (i = 0; i < NUM_INPUT+1; i++){
		testinput[i] = testinput[i]/temp;
	}
}
/* NN matching function */
void match_NN(float testinput[])/* parameter search */
{
	int i, j;
	float net_input, index, mhidden[NUM_HIDDEN], moutput[NUM_OUTPUT+1];

	/* (1) NN output */
	for (i = 0; i < NUM_HIDDEN; i++){
		net_input = 0;
		for (j = 0; j < NUM_INPUT+1; j++){
			net_input = net_input + w_ih[j][i] * testinput[j];
		}
		mhidden[i] = sigmoid(net_input - thresh_h[i]);
	}
	for (i = 0; i < NUM_OUTPUT+1; i++){
		net_input = 0;
		for (j = 0; j < NUM_HIDDEN; j++){
			net_input = net_input + w_ho[j][i] * mhidden[j];
		}
		moutput[i] = sigmoid(net_input- thresh_o[i]);
	}
	/* (2) scaling */
	index = moutput[NUM_OUTPUT];
//	printf("\n-------------- FUNCTION : match_NN ----------------\n");
//	printf("NNparameter : ");
	for (i = 0; i < NUM_OUTPUT; i++){
		NNparam[i] = moutput[i]/index*ini_paramt[i];
//		printf("%f ", NNparam[i]);
	}
}

