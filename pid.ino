

void pid_control()
{


 angle_error=0-gx;
angle_difference= angle_error - angle_error_old;
//-----------------------------------------------------------------------------------CALCULATE RAW_PID
double pTerm = kp * angle_error;
iTerm += ki * angle_error * dt;   iTerm = constrain(iTerm, -100.0f, 100.0f); 
double dTerm = kd  * (angle_error - angle_error_old)/dt;
angle_error_old=angle_error;
// raw_pid =150;
// raw_pid = pTerm + dTerm+ iTerm;
  raw_pid = pTerm + dTerm;

//-----------------------------------------------------------------------------------REMOVE DEADBAND OF PWM
if (raw_pid < deadband_pwm_p) { if (0<raw_pid ) { raw_pid=deadband_pwm_p;} }
if (raw_pid > deadband_pwm_n) {if (0>raw_pid ) { raw_pid=deadband_pwm_n; } }
pwm_cikis=raw_pid;
if (pwm_cikis>255){ pwm_cikis=255;}
if (pwm_cikis<-255){pwm_cikis=-255;}

      
     
         if(  inside_the_deadzone() ==0){
                        digitalWrite(LED_PIN, 0);inside_the_space_beyond_our_dimension=0;
                    
                        if(raw_pid>=0  ){   //gx is negative
                       digitalWrite(STBY, HIGH); 
                       digitalWrite(AIN1, LOW);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, pwm_cikis);
                       digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, pwm_cikis); }
                       
                         else {
                          pwm_cikis=-1*pwm_cikis;
                          digitalWrite(STBY, HIGH); 
                          digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
                          digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis); }
    
        
          }
          else{
                 digitalWrite(LED_PIN, 1);
                 if (pwm_cikis < 0) {pwm_cikis=-1*pwm_cikis;}
                  if(inside_the_space_beyond_our_dimension==0){
                        switch (pos) {

                          //  case 0:increase_speed_from_positive_moment(pwm_cikis);break;
                            case 1:decrease_speed_from_positive_moment(pwm_cikis); break;
                            case 2: decrease_speed_from_negative_moment(pwm_cikis);break;
                          //  case 3:increase_speed_from_negative_moment(pwm_cikis);break;
                            default: break;

                        }
                   }
               }


      sprint(gx,raw_pid,pos,pwm_cikis);
      
//      
//                              #ifdef ROBOT    
//                        
//                               Serial.print(" gx = ");          Serial.print(gx);              Serial.print("\t");
//                               //    Serial.print(" kp = ");          Serial.print(kp);              Serial.print("\t");
//                                //     Serial.print(" ki = ");          Serial.print(ki);              Serial.print("\t");
             //                  Serial.print(gx);    Serial.print(" ");     Serial.print(gx);    Serial.print(" ");     Serial.print(raw_pid);             Serial.print("\t");  
//                               Serial.print(" pwm_cikis = ");     Serial.print(pwm_cikis);     Serial.print("\t");    Serial.print("\t");
//                               Serial.println("\t");
//                                #endif
//                        
              
                     
              
                 
              
              }




void pid_update()
{

    double angle_error=0-gx;
   angle_difference= angle_error - angle_error_old;
//-----------------------------------------------------------------------------------CALCULATE RAW_PID
 double pTerm = kp * angle_error;
 iTerm += ki * angle_error * dt;   iTerm = constrain(iTerm, -100.0f, 100.0f); 
 double dTerm = kd  * (angle_error - angle_error_old)/dt;
 angle_error_old=angle_error;
 raw_pid = pTerm + dTerm+ iTerm;
  
}

void decrease_speed_from_positive_moment(int spd ){
      for(int i=spd; i>0;i--){
               digitalWrite(STBY, HIGH); 
    
                digitalWrite(AIN1, LOW);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, i);
                 digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, i); 
                  delay(1);
                 }
               
      inside_the_space_beyond_our_dimension=1;
                }
      

void increase_speed_from_positive_moment(int spd ){
   for(int i=0; i<spd;i++){
   digitalWrite(STBY, HIGH); 
    digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, i);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, i); 
    delay(1); 
    }
          //   
 inside_the_space_beyond_our_dimension=1;      
}
void decrease_speed_from_negative_moment(int spd ){
   for(int i=spd; i>0;i--){
       digitalWrite(STBY, HIGH); 
       digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, i);
       digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, i); 
       delay(1);  
    }
         
inside_the_space_beyond_our_dimension=1;                
}

void increase_speed_from_negative_moment(int spd ){
   for(int i=0; i<spd;i++){
    digitalWrite(STBY, HIGH); 
  digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, i);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, i); 
             delay(1);    
 inside_the_space_beyond_our_dimension=1;         
}
} 


 
uint8_t inside_the_deadzone( ){
  uint8_t aaa=1;
     if( ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || ( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir ) )  )  {
      aaa=0;
     }
    return aaa;
}



void realise()
{


  /* açının - oldugu yönden freefall yaparken 0
   * açının - oldugu yönden düzeltme  yaparken 1
   * açının + oldugu yönden freefall yaparken 3
   * açının + oldugu yönden düzeltme  yaparken 2
 
   */

 aeod= gx - aeo;    // 5-10 =-5 çıktı
 aeo=gx;            // şimdiki gx degeri kaydedildi.
pos=5;                //
  
     
if(gx<0){if(aeod<0){ aeod=aeod*-1;if(aeod>0.5){    pos=0;} } 
    else if(aeod>0){if(aeod>0.5){ pos=1;} }   }
                

 if(gx>0){if(aeod<0){ aeod=aeod*-1;if(aeod>0.5){    pos=2;} } 
    else if(aeod>0){if(aeod>0.5){ pos=3;} }   }
                
     




 // Serial.print(pos);   Serial.print(" ");   Serial.print(gx);    Serial.print(" ");   Serial.println(inside_the_space_beyond_our_dimension);  
  
  
}



void sprint(float a,float b ,float c ,float d)
{
    Serial.print(a); 
      Serial.print(" "); 
        Serial.print(b); 
        Serial.print(" ");  
        Serial.print(c); 
      Serial.print(" "); 
        Serial.println(d); 

  
}




