#include <stdio.h>
//#include "io.c"

#define thresh1
#define thresh2
#define finding_delay						
#define right_turn_delay

//obstacle avoidance and PID
									//12/right back		19/ left back			wiring PI 1==>lf, 23==>rf, 0==>lr, 27==>rr
const int pwmrf = 1, pwmlf = 26;//, pwmrr,pwmlr;		18/left_forward  13/right forward          17left back	16right back
const int usl, usfl, usf, usfr, usr;		//wait 1ms for US signal to come back

unsigned char obs, obs1, obs2, obs3, obs4, obs5;

//1=>G18,  26=>G12,  24=>G19,   23=>Not Working

const int alignindexsmall, alignindexlarge,aligndelay;		//make it so that the goal doesn't go from too right to too left, i.e,
															//thresholds should be so that a reasonable delay doesn't make a close
															//goal go out of bounds			//includes and #defines




unsigned char obs=0, obs1=0, obs2=0, obs3=0, obs4=0, obs5=0;
unsigned char echo = 0;
void poll(unsigned char i)
{
	echo = 0;
	digitalWrite(uso[i],HIGH);
	delayMicrosecondsHard(10);
	digitalWrite(uso[i],LOW);
	delayMicrosecondsHard(100);
	echo = !(digitalRead(usi[i]));
	if(echo){
	switch (i)
	{
		case 1:
		obs1 = 1;		break;
		case 2:
		obs2 =1;		break;
		case 3:
		obs3 =1;		break;
		case 4:
		obs4 =1;		break;
		case 5:
		obs5 =1;		break;
	}
	}
	else{
		case 1:
		obs1 = 0;		break;
		case 2:
		obs2 =0;		break;
		case 3:
		obs3 =0;		break;
		case 4:
		obs4 =0;		break;
		case 5:
		obs5 =0;		break;
	}
	return;
}







//(char) 65 ==> Typecasting 65 to char

void align(){
	//if (centroid <= thresh1)
	//{	pwm();
		//delay(aligndelay);}
		unsigned char cil = 0, cir = 0, cit = 0;
		if ((centroid<=thresh2)&&(centroid>=thresh1)){cit = 1; return;}
		else if (centroid<=thresh1){cil = 1;cit = 0;}
		while(cil)
		{pwm(alignindexsmall,alignindexlarge)
			delay(aligndelay);
			pwm(0,0);
			ros::Subscriber width = n.subscribe("objects", 2, centroid_callback);
			if ((centroid<=thresh2)&&(centroid>=thresh1)){cit = 1;cil = 0;return;}
			else if (centroid<thresh1){cil = 1;} //also goes in the compute_centroid fn
		}
		if (centroid >= thresh2){cir = 1; cit =0;}
		while(cir)
		{pwm(alignindexlarge,alignindexsmall)
			delay(aligndelay);
			pwm(0,0);
			ros::Subscriber width = n.subscribe("objects", 2, centroid_callback);
			if ((centroid<=thresh2)&&(centroid>=thresh1)){cit = 1;return;}
			else if (centroid>thresh2){cir = 1;}
			//else if ((centroid>thresh2)&&(centroid<thresh1)){cit = 0;} //also goes in the compute_centroid fn
		}
		return;
	}



void pwm(short speed_index1, short speed_index2)
{
	short ip = 0;
	if (speed_index1 > 1023)
	{  speed_index1 = 1023;}
	if (speed_index2 > 1023)
	{	speed_index2 = 1023;}
	if (speed_index1 < -1023)
	{  speed_index1 = -1023;}
	if (speed_index2 < -1023)
	{	speed_index2 = -1023;}  //truncates indices at 1023 and -1023
	
	if(speed_index1 >= 0){
		pwmWrite(pwmrf, speed_index1);}
	else{
		ip = (-speed_index1);
		pwmWrite(pwmrr, (-speed_index1));}
		
	if(speed_index2 >= 0){
		pwmWrite(pwmlf, speed_index2);}
	else{
		ip = -speed_index2;
		pwmWrite(pwmlr, (ip);}

} //can probs be put in the actions part of the sm

//enum find_States{initial, notInFrame, right, left} fmstate;

void findGoal()			//the if statement needs to be changed to centroid not in picture
						//and then may be once it is in the frame the bot slowly aligns with it
{
	//switch(fmstate){}
			if(!obj){cip = 0;}			//!obj is when if (data->msg) is not exe'd
			while(cip == 0)
			{
				pwm(alignindexlarge,alignindexsmall);				//may need to use separate indices for finding
				delay(finding_delay);
				pwm(0,0);
				ros::Subscriber width = n.subscribe("objects", 2, centroid_callback);				//run like this or make it into a separate function?
												//probs just write a subscriber thing
				if(!obj){cip = 0;}
					else {cip = 1;}
			}
			return;
}
			
		
enum Move_State {initi, movfwd, movright, movleft, correctionalfwd} state;

unsigned int right_turn_counter = 0;  //counts the number of delays that bot turns right for
									  //then the left turn would use a delay = counter*delay;
void Move()
{
	switch(state)
	{
		case initi:
		if ((centroid<=thresh2)&(centroid>=thresh1))
		{if(!obs){			state = movfwd;			}}
		else {align();}
		break;
		
		case movfwd:
		if(!obs1 && !obs2 && !obs3)
		{			state = movfwd;			}
		else if(obs2 || obs3)
		{			state = movright;		}
		break;
		
		case movright:
		if(obs2 || obs3 || obs4)		//and not obs5
		{			state = movright;		}
		if(!obs2 && !obs3 && !obs4)
		{		state = correctionalfwd;	}
		break;
		
		case correctionalfwd:
		if(!obs1 && !obs2 && !obs3)
		{	//SM_find(i/p being the turns needed to get back on track)
			movleft;}
		if(obs1 || obs2)
		{		state = correctionalfwd;	}
		break;
		
		case movleft:
		if(centroid > thresh1)
		{			case movfwd;			}
		if(centroid <= thresh1)
		{			case = movleft;			}	
		break;
	}
	switch(state)
	{
		case movfwd:
		right_turn_counter = 0;
		if(int(object) == 1){pwm(0,0);}
		else if(int(object) == 2){pwm(512,512);}
		else if(int(object) == 3){pwm(800,800)}
			else{pwm(1023,1023);}
		break;
		
		case movright:						//Imma do the whole obs avoidance thing here, and it may interfere
											//with the inerrupts, should we choose to utilize them
		//do 
		//{
		    pwm(right turn indices);
			delay(right_turn_delay);
		//	right_turn_counter +=1;
			pwm(0,0);
		//	poll the US sensors;
		//} while (obstacles to the front of the machine);
		break;	
		
		case correctionalfwd:
		pwm(700, 700);
		break;
		
		case movleft:
		//while(right_turn_counter >=0){
		findGoal();
		align();
	}	
}
	
	
	
	
int main(void)
{
	wiringPiSetupGpio();
	unsigned char cti = 0;
	
	pinMode(pwmr, PWM_OUTPUT);
	pinMode(pwml, PWM_OUTPUT);
	
	findGoal();
	align();
	ros::Subscriber width = n.subscribe("objects",2,centroid_callback);
	ros::spinOnce();
	while(1)
	{
		ros::Subscriber width = n.subscribe("objects",2,centroid_callback);
		align();
		poll the sensors_HOW?;
		Move();
	}
}
	
