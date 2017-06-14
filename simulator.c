// posture control of inverted pendulum
// created by humanoid team oct20 2006 (last updata)


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "func_lib.h"
#include "gptcall.h"


int main(void){
	int i;
	float t;
	int dt = 1;

	/* parameter for file out */
	FILE *f1, *f2, *f3, *f4, *RollJ, *Pitch;

	/* fopen */
	f1 = fopen("data/square.csv","w");
	f2 = fopen("data/state.csv","w");
	f3 = fopen("data/square.dat","w");
	f4 = fopen("data/state.dat","w");
	RollJ = fopen("data/roll_joint.csv","w");
	Pitch = fopen("data/pitch_joint.csv","w");

	/* Initialization                                     */
	/* -------------------------------------------------- */
	
	float zmp_d; // desired zmp signal
	float state[5]; // state value for differencial equation
	float roll_joint[3];
	/*     roll_joint[0] : hip joint for right leg
	       roll_joint[1] : hip joint for left leg
	       roll_joint[2] : ankle joint for right leg
	       roll_joint[3] : ankle joitn for left leg      */
	float pitch_joint[2];
	/*     pitch_joint[0] : hip joint for pitching       */
	/*     pitch_joint[1] : knee joint for pitching      */
	/*     pitch_joint[2] : ankle joint for pitching     */
	
	for (i=0; i<6; i++){
		state[i] = 0.0;
	}
	/* state[0] : exensor neuron                   */
	/* state[1] : neruon value for extensor neruon */
	/* state[2] : flexor neuron                    */
	/* state[3] : neuron value for flexor neuron   */
	/* state[4] : COM position                     */
	/* state[5] : COM velocity                     */
	float test_input[NUM_INPUT];

	/* -------------------------------------------------- */

	/* NN learning for parameters knowledge 	      */
	/* -------------------------------------------------- */
	/* (1) read and display training data       	      */
	read_data();
	/* (2) parameter substitution & all input parameter   */
	substi_param(ini_paramt);
	all_input();
	/* (3) weight and threshold initialization            */
	init_wt();
	/* (4) scaling NN training data                       */
	scale_NN();
	/* (5) Learning weight and threshold of NN            */
	learn_NN();
	/* -------------------------------------------------- */

	printf("INPUTparam.A : ");
	for (i=0;i<NUM_SAMPLE+1;i++){
		printf("%f ", INPUTparam.A[i]);
	}
	printf("\n");
	 for(t=0;t < LAST_TIME; t += (float)dt/1000.0) {

		 // Reference ZMP //
		 if (t<=7.5){
			 refZMP.A = INPUTparam.A[0]; refZMP.T = INPUTparam.T[0];
        	 }
	 	 else if(t<=15.0){
			 refZMP.A = INPUTparam.A[1]; refZMP.T = INPUTparam.T[1];
		 }
		 else {
			 refZMP.A = INPUTparam.A[2]; refZMP.T = INPUTparam.T[2];
		 }
		 zmp_d = (float)(refZMP.A*square(refZMP, t));
		 // parameter matching with NN                //
		 testINPUT(refZMP, test_input);
		 match_NN(test_input);
		 substi_param(NNparam);
		 model.k1 = NNparam[4]; model.kp_osc = NNparam[5]; model.kv_osc = NNparam[6];
		 // Runge-Kutta //
		 Runge_Kutta(refZMP, model, NOparam, state, t);

		 // rolling joint angle //
		 roll_angle(state[4], roll_joint);
		 // pitching joint angle //
		 pitch_angle(state[4], pitch_joint);
		 
		 // fprint //
		 fprintf(f1,"%f,\n",zmp_d); // desired ZMP //
		 fprintf(f3,"%f %f\n",t,zmp_d); // desired ZMP //
		 fprintf(f4,"%f ",t); // time //
		 for (i = 0; i<6; i++){ // state //
			fprintf(f2,"%f,",state[i]);
			fprintf(f4,"%f ",state[i]);
		 }
		 fprintf(f2,"\n");
		 fprintf(f4,"\n");
		 for (i = 0; i<4; i++){ // rolling angle //
			fprintf(RollJ, "%f,", roll_joint[i]);
		 }
		 for (i = 0; i<3; i++){ // pitching angle //
			fprintf(Pitch, "%f,", pitch_joint[i]);
		 }
		 fprintf(RollJ,"\n");
		 fprintf(Pitch,"\n");
	 }

	/* fclose */
	fclose(f1);
	fclose(f2);
	fclose(f3);
	fclose(f4);
	fclose(RollJ);
	fclose(Pitch);

	/* graph */
	mk_graph();

	return 0;
}

