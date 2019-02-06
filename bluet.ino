void bluetooth(int gx){  //Serial işlemler içerdiginden bu kodu Kullanılırken ana koddaki #define robot kısmı kapatılmalıdır

 sendvalue(gx) ;  //bu fonksyon ile istenilen degişken degeri rakamlara ayrılır ve usart üzerinden gönderilir. 

 // getvalue();  //bu fonksyon deneme aşamasındadır.  p i d degerlerini bluetoot üzerinden arduinoya yüklemeye yarar.
   
 


}

void parse_to_digits(int number)
{

  

  /********
   * 
   * int digit_1000;
   int digit_100;
   int digit_10;
   int digit_1;
   sign_of_number;
   these values must been declared as global variables. 
   * 
   * 
   * 
   */

    int number_3digits;
    int number_2digits;
    int number_1digits;
    int mod2;
    int mod1;
    int mod3;
   // int sign_of_number;
    
    if(number<0){
      number=-1*number;   //take absolute of it  , not neccesary for adc values 
      sign_of_number=1;}
    else{
      sign_of_number=2;
    }

      
      //here is the math calculations . actually  each of the block'S third line are not neccesaryy but doesnt hurt no body. 
      
      /*************************************calculate 1000  digit ************************************/
        mod1=number%1000; //564
        digit_1000=(number-mod1)/1000 ;  //4. digit   1
        number_3digits =(number-(1000*digit_1000))  ;    // 564 
      
       /*************************************calculate 100  digit ************************************/
        mod2=number_3digits % 100;  //  64
        digit_100=(  number_3digits-mod2   )/100;   // 3. digit   5
       number_2digits =(number_3digits-(100*digit_100)) ;    //2. hane digit_10iz digit_10ayı
      
      
       /*************************************calculate 10 digit ************************************/
      
      mod3=number_2digits % 10;  //63
      digit_10=(  number_2digits-mod3   )/10 ;   //2. hane
      number_1digits= (number_2digits-(10*digit_10))  ;  //ı  
      
       /*************************************calculate 1  digit ************************************/
      digit_1=number_1digits;
      

  
}


int digit_dec2ascii(int decimal_digit){

decimal_digit=+48;
return decimal_digit;
  

}

void sendvalue(int value)
{

  //  packet demo  :  35 0 52 51 20   which is equal to #+432
    int asci_digit100= 0; 
     int asci_digit10=  0;
      int asci_digit1=  0;
   
      
    parse_to_digits(value);   

    Serial.write(35);   // #
  

   
     asci_digit100=  digit_100+48;  //rakamlar ascii ye çevrildi
     asci_digit10=  digit_10+48;
     asci_digit1=  digit_1+48;


  Serial.write (sign_of_number);     
  Serial.write (asci_digit100);     
    Serial.write (asci_digit10);
    Serial.write (asci_digit1); 

   // Serial.println();
}

void getvalue()
{
   
   if(Serial.available()>0)  
   {

    while(Serial.read()!=200){  //ilk deger 200 olucak her zaman 
      int deger0=Serial.read();   
      Serial.write(deger0);
      p= deger0;
      
    }
   }

  

 //Serial.write(gx);
}











