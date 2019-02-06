

void pid_control()
{

gx=gx-set_noktasi;

  giris=gx;
  float angle_error=0-gx;

  
  


 float pTerm = kp * angle_error;
 iTerm += ki * angle_error * dt;   iTerm = constrain(iTerm, -1.0f, 1.0f); 
 float dTerm = kd  * (angle_error - angle_error_old)/dt;
 angle_error_old=angle_error;
 
 float PIDValue = pTerm + iTerm + dTerm;
              
 
  pwm_degis=0;
                    
                // float raw_pid=0;
                 float raw_pid = pTerm + iTerm + dTerm;
                 
                // raw_pid=cikis;
             
                    cikis_max=kp*(32-set_noktasi) +  ki*32*10/100;  ;  cikis_max=cikis_max+20; //şimdi bizim robot max 32 derece egiliyor. buna göre max cıkısı hesaladık.
                    cikis_min=kp*(-32+set_noktasi)   + ki*-32*10/100   ; cikis_min=cikis_min-20;
//3589
                     //şöyle bir sıkıntımız vardı. pwm 50 den küçük oldugunda motorlara bir etkisi yoktu bu sebeble c
                     //cıkıs degerini map yaparken  -120 ile 0 arası  -50 ile -255 e map olacak şekilde ve 
                     // 0 ile 120 arası  50 ile 255 e map olacak şekilde ayarladım. işte burda ki 50 degerini yükledigim 
                     //degişkenin ismi initial_speed.

                        
//                      if(cikis>0){
//                    cikis3 =   ( cikis* 200/ (cikis_max-0)) +initial_speed ;  }  // p=2, i=1, d=0.5;   için initial_speed  86  bulundu,
//                      if(cikis<=0){
//                    cikis3 =  (  cikis* 200/ (0-cikis_min) ) -initial_speed ;  }
//                    

//                    if(cikis2>0){
//                     cikis3 =  cikis2+30   ;}
//                     else{
//                     cikis3 =  cikis2-30   ;}
            
             /************MAKS VALUE BULMA*******************/



             
                       pwm_cikis=raw_pid;
                              if (pwm_cikis>255){
                              pwm_cikis=255;}
                              if (pwm_cikis<-255){
                              pwm_cikis=-255;}

      
      if( ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || ( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir ) )  )       {   //-5 ile +5 arasında ise   veya  -35 ten küçük ise veya +35ten büyük ise
         
                         if(raw_pid>=0  )
           {
                      
                       digitalWrite(STBY, HIGH); digitalWrite(AIN1, LOW);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, pwm_cikis);
                         digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, pwm_cikis);
             }
           else {
                 tasima=-1*pwm_cikis;
                  pwm_cikis=tasima;
                 digitalWrite(STBY, HIGH); digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
                   digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis);
                 }
        
      }
       else{
          pwm_cikis=1;
            digitalWrite(STBY, LOW); //disable standby
                                digitalWrite(AIN1, LOW);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
                                  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis);
}




                              
//  //   -5 ile +5 arasında ise   veya  -35 ten küçük ise veya +35ten büyük ise
//       if( ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || ( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir ) )  )       {
//     // if(   ! (              ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || ( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir ) )  )   )    {
//      //  if(  ! ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || !(( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir )) )  )       {
//                  
//                                               if(raw_pid>=0  )
//                                                  {
//                
//                                                  digitalWrite(STBY, HIGH); digitalWrite(AIN1, LOW);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, pwm_cikis);
//                                                  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, pwm_cikis);
//                                                }
//                                                else {
//                                                   tasima=-1*pwm_cikis;
//                                                   pwm_cikis=tasima;
//                                                  digitalWrite(STBY, HIGH); digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
//                                                  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis);
//                                                 }
//
//                                                 
//                        
////                                               }
////                                               else
////                                               {      pwm_cikis=2;}
//
//                                                   
//                      }
//                      else
//                      { 
//                        
//                        sira=0;
//                        pwm_cikis=1;
//                        
//                       // Serial.print("+-"); 
//                          digitalWrite(STBY, LOW); //disable standby
//                                  digitalWrite(AIN1, LOW);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
//                                  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis);
//                        //  mpu.resetFIFO();
//
//                      }
      
      
      
      
                            #ifdef ROBOT    
                        
                               Serial.print(" gx = ");          Serial.print(gx);              Serial.print("\t");
                                   Serial.print(" kp = ");          Serial.print(kp);              Serial.print("\t");
                                //     Serial.print(" ki = ");          Serial.print(ki);              Serial.print("\t");
                             //   Serial.print(" gy = ");          Serial.print(gy);              Serial.print("\t");
                             //    Serial.print(" gz = ");          Serial.print(gz);              Serial.print("\t");
                               Serial.print(" raw_pid = ");         Serial.print(raw_pid);             Serial.print("\t");  
                            //   Serial.print(" cikis2 = ");         Serial.print(cikis2);           Serial.print("\t");  
                              //    Serial.print(" cikis3 = ");         Serial.print(cikis3);            Serial.print("\t"); 
                                //           Serial.print(" ames = ");         Serial.print(aci_max_eksi_sinir);            Serial.print("\t"); 
                          //  Serial.print(" zaman_deg  = ");  Serial.print(zaman_degisimi);  Serial.print("\t");
                               Serial.print(" pwm_cikis = ");     Serial.print(pwm_cikis);     Serial.print("\t");    Serial.print("\t");
                            
                              // Serial.print(" tplam_dizi = ");    Serial.print(toplam_dizi);     Serial.print("\t");
                           //  Serial.print(" cikis max = ");    Serial.print(cikis_max);     Serial.print("\t");
                            //   Serial.print(" cikis min = ");    Serial.print(cikis_min);     Serial.print("\t");
                             //    Serial.print(" initial_speed = ");    Serial.print(initial_speed);     Serial.print("\t");
                               // Serial.print(" p = ");    Serial.print(p);     Serial.print("\t");
                           //     Serial.print(" pot:");    Serial.print(tune);     Serial.print("\t");
                             //      Serial.print(" pot2:");    Serial.print(tune2);     Serial.print("\t");
                                  //  Serial.print(" sensorValue2");    Serial.print(sensorValue2);     Serial.print("\t");
                           //     Serial.print(" hata =  ");         Serial.print(e);               Serial.print("\t");
                        //        Serial.print(" Dhata  =  ");       Serial.print(Dhata);           Serial.print("\t");
                        //        Serial.print("\t");
                        //         Serial.print(" son_sure   = ");    Serial.println(son_sure);
                         //    Serial.print(" zaman_deg  = ");  Serial.print(zaman_degisimi);  Serial.print("\t");
                              Serial.println("\t");
                                #endif
                        
                        
                                        
                    
                    
             
              
                     // delay(1);
              
                 
              
              }
