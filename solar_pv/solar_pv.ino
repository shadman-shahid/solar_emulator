#include <math.h>
#define pwmPin 9
#define a 0.0025
#define b 0.5
#define c 0.00288
#define P 10
#define I 0.2
#define D 0.2
#define VOC 12   //Open Circuit voltage
#define ISC 0.5    // Short Circuit Current


float Isc=ISC, Voc=VOC, Sref=1000, Tref=25, Vlimit=0;; //Sref=Solar irradiance Tref= Reference Temperature
float Im=0.85*Isc, Vm=0.9*Voc;
double C1,C2,IL,Icorr, VL, ZL,ZLM, zerror,error, prev_error=0;
int pwm_val=220, Corr_pwm=0;

void setup() 
{
 Serial.begin(9600);
 pinMode(pwmPin,OUTPUT);
 pinMode(5,OUTPUT);
 analogWrite(pwmPin,0);
 analogWrite(5,0);

}

void loop() 
{
  //update_base_param(900,25);
  update_current_data();
  calculate_error();
  tune_PID();
  analogWrite(pwmPin,pwm_val);
  
  Serial.print("Current :");
  Serial.println(IL);
  Serial.print("Voltage :");
  Serial.println(VL);
  Serial.print("Required Current :");
  Serial.println(Icorr);
  Serial.print("Corrected pwm:");
  Serial.println(Corr_pwm);
  Serial.println("");
  delay(1000);

}

void update_base_param(float S, float T){
  float delS, delT;
  delS=(S/Sref)-1;
  delT=T-Tref;
  Voc=VOC*(1-c*delT)*(1+log(1+b*delS));
  Vm=Voc*0.9;
  Isc=ISC*S*(1+a*delT)/Sref;
  Im=Isc*0.85;
  
}


void update_current_data(){
  IL=analogRead(A0);
  IL=IL*5/1024;
  VL=analogRead(A1);
  VL=VL*45/1024;
}

void calculate_error()
{
  ZL=VL/IL;
  ZLM=Vm/Im;
  zerror=ZL-ZLM;
  C2=(Vm/Voc-1)*(log((Isc-Im)/Isc));
  C1=(1-Im/Isc)*exp(-Vm/(C2*Voc));
  Vlimit=C2*Voc*log((1-(0.01/Isc))/C1-1);  
  if (VL>0 && VL<=Vlimit)
  {
    Icorr=Isc*(1-C1*(exp(VL/(C2*Voc))-1));
  }
  else
  {
    Icorr=0.01;     
  }
  
}


void tune_PID(){
    Corr_pwm=pwm_val;
    error=(Icorr-IL)*P+prev_error*D;
    prev_error=error;
    Corr_pwm=pwm_val-error;
    pwm_val=Corr_pwm;
    if (pwm_val>=256)
    {
      pwm_val=250;  
    }
}


