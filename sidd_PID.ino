#include<Servo.h>
#include<Wire.h>


//Servo Objects :
Servo left_Arm;
Servo right_Arm;

//Joystick Variables :
int Joystick;

//PWM Variables :
int left_PWM;
int right_PWM;


//Gyro Variables :
float Gyro_raw_angleX, Gyro_raw_angleY, Gyro_raw_angleZ;
float Gyro_raw_error_X, Gyro_raw_error_Y;
float Gyro_AngleX, Gyro_AngleY;
float Gyro_error = 0;

//Acceleraton Variables :
float Accel_raw_angleX, Accel_raw_angleY, Accel_raw_angleZ;
float Accel_angle_error_X, Accel_angle_error_Y;
float Accel_AngleX, Accel_AngleY;
float Accel_error = 0;

//Angle Variables :
float Total_AngleX;
float Total_AngleY;

//Time Vairables :
float elapsedTime, timePrev, time;
float Rad_To_Deg = 180/3.141592654;

//PID Varibales :
float PID;
float error, Previous_error;
float PID_p = 0;
float PID_i = 0;
float PID_d = 0;

//PID Constants :
float kp = 0.32;
float ki = 0.27;
float kd = 0.41;
float Desired_Angle = 0;

void setup() {
  // put your setup code here, to run once:
  left_Arm.attach(5,1000,2000);
  right_Arm.attach(9,1000,2000);

  Serial.begin(9600);
  Wire.begin();

  //================================================================== -: SECTION 01 :- ==================================================================//
                     //In this Section we calculate Acceleration, Gyro and Total angle


                          
                          ////////// Configure The MPU6050 Module //////////
          
   //Power Management:
     Wire.beginTransmission(0b1101000);   //This is the I2C address of MPU6050 (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
     Wire.write(0x6B);                   //Access the register 6B - Power Management (Sec. 4.28, "register data sheet")
     Wire.write(0b00000000);             // Setting Sleep register to 0.(Required; see Note on p. 9
     Wire.endTransmission();
    

   //Gyroscope Configeration: 
     Wire.beginTransmission(0b1101000);    //I2C address of MPU6050
     Wire.write(0x1B);                     //Accessing the register 1B
     Wire.write(0b00000000);             //Setting the gyro full scale range +/- 250 ./S [degrees per second]
     Wire.endTransmission();

    //Accelerometer Configeration:
      Wire.beginTransmission(0b1101000);  //I2C address of MPU6050
      Wire.write(0x1C);                   //Accessing the register 1C
      Wire.write(0b00010000);           //Setting the Accelerometer full scale range +/- 8g
      Wire.endTransmission();

  


                   //-------------------------- Calculate The Gyro_Raw_Error --------------------------//
                
                                 //Here We Calculate The Gyro Raw Data Before We Move To The Loop.
                                      //* We make mean of 200 values, to calculate the gyro raw error.

 if(Gyro_error==0)
  {
     for(int i=0; i<200; i++)
       {
        //recodGyroRegisters:
          Wire.beginTransmission(0b1101000);           //I2C address of MPU6050
          Wire.write(0x43);                           //Startig the ragister for gyro readings
          Wire.endTransmission();
          Wire.requestFrom(0b1101000,6);              //Request for all gyro registers (43 to 48)
          while(Wire.available() < 6);
          Gyro_raw_angleX = Wire.read()<<8|Wire.read(); //Store first two X-axis bytes into Gyro_raw_angleX
         // Gyro_raw_angleY = Wire.read()<<8|Wire.read(); //Store middle two Y-axis bytes into Gyro_raw_angleY
      //    Gyro_raw_angleZ = Wire.read()<<8|Wire.read(); // Store last two Z-axis bytes into Gyro_raw_angleZ
         
       //Calculate Gyro Raw Error:
         Gyro_raw_error_X =  Gyro_raw_error_X + (Gyro_raw_angleX/131.0); //X-axis
        // Gyro_raw_error_Y =  Gyro_raw_error_Y + (Gyro_raw_angleY/131.0); //Y-axis
         
    if(i==199)
    {
      Gyro_raw_error_X = Gyro_raw_error_X/200; //X-axis
    //  Gyro_raw_error_Y = Gyro_raw_error_Y/200; //Y-axis
      Gyro_error=1;
   }
  }
 }//End of Gyro error Calculation




    //------------------------------ Calculate The Accel_angle_Error ------------------------------//
    
                          //Here We Calculate The Gyro Raw Data Before We Move To The Loop.
                               //*  We make mean of 200 values, to calculate the gyro raw error.

 if(Accel_error==0)
 {
 for(int a=0; a<200; a++)
  {
    //record Acceleratopn registers:
      Wire.beginTransmission(0b1101000);              //I2C address of MPU6050
      Wire.write(0x3B);                               //Starting the register for accel readings
      Wire.endTransmission();
      Wire.requestFrom(0b1101000,6);                   //Request for all accel registers ( 3B to 40)
      while(Wire.available() < 6);
      Accel_raw_angleX = (Wire.read()<<8|Wire.read())/4096.0;  //Store first two X-axis bytes into Accel_raw_angle
    //  Accel_raw_angleY = (Wire.read()<<8|Wire.read())/4096.0; //Store middle two Y-axis bytes into Accel_raw_angleY
    //  Accel_raw_angleZ = (Wire.read()<<8|Wire.read())/4096.0;  // Store last two Z-axis bytes into Accel_raw_angleZ

   //Calculate Accel Angle Error:
     Accel_angle_error_X = Accel_angle_error_X + ((atan((Accel_raw_angleY)/sqrt(pow((Accel_raw_angleX),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg));  //X-axis
    
   //  Accel_angle_error_Y = Accel_angle_error_Y + ((atan(-1*(Accel_raw_angleX)/sqrt(pow((Accel_raw_angleY),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg));//Y-axis

   if(a==199)
     {
        Accel_angle_error_X = Accel_angle_error_X/200; //X-axis
    //    Accel_angle_error_Y = Accel_angle_error_Y/200; //Y-axis
        Accel_error=1;
  }
 }

}
}

void loop() {
  // put your main code here, to run repeatedly:
  ////////// Calculate The Elapsed Time //////////
             timePrev = time;   // the previous time is stored before the actual time read  
                 time = millis();  //Actual Time Read in mili Seconds
          elapsedTime = (time - timePrev)/1000;     //Calculate elapsed Time     


      //------------------ Calculate The Gyro Angle :------------------//

     //Record Gyro Registers:
       Wire.beginTransmission(0b1101000);           //I2C address of MPU6050
       Wire.write(0x43);                           //Startig the ragister for gyro readings
       Wire.endTransmission();
       Wire.requestFrom(0b1101000,6);              //Request for all gyro registers (43 to 48)
       while(Wire.available() < 6);
       Gyro_raw_angleX = Wire.read()<<8|Wire.read(); //Store first two X-axis bytes into Gyro_raw_angleX
   //    Gyro_raw_angleY = Wire.read()<<8|Wire.read(); //Store middle two Y-axis bytes into Gyro_raw_angleY
  //     Gyro_raw_angleZ = Wire.read()<<8|Wire.read(); // Store last two Z-axis bytes into Gyro_raw_angleZ

    //Calculate The Gyro_Raw_Angle:
      Gyro_raw_angleX = (Gyro_raw_angleX/131.0) - Gyro_raw_error_X; //Gyro_Angle_X  
   //   Gyro_raw_angleY = (Gyro_raw_angleY/131.0) - Gyro_raw_error_Y; //Gyro_Angle_Y

    //Calculate The Gyro Angle:
            //Now we integrate the raw value in degrees per second in order to obtain the gyro angle.
               //* If we multiplied degrees/second * degrees we get "degrees".
               
      Gyro_AngleX = Gyro_raw_angleX*elapsedTime; //X-axis
      
   //   Gyro_AngleY = Gyro_raw_angleY*elapsedTime; //Y-axis


     //------------------------- Calculate The Accel Angle :-------------------------//

         //record Accel Registers:
            Wire.beginTransmission(0b1101000);              //I2C address of MPU6050
            Wire.write(0x3B);                               //Starting the register for accel readings
            Wire.endTransmission();
            Wire.requestFrom(0b1101000,6);                   //Request for all accel registers ( 3B to 40)
            while(Wire.available() < 6);
            Accel_raw_angleX = (Wire.read()<<8|Wire.read())/4096.0; //Store first two X-axis bytes into Accel_raw_angleX
        //    Accel_raw_angleY = (Wire.read()<<8|Wire.read())/4096.0; //Store middle two Y-axis bytes into Accel_raw_angleY
          //  Accel_raw_angleZ = (Wire.read()<<8|Wire.read())/4096.0; // Store last two Z-axis bytes into Accel_raw_angleZ


       //Calculate The Accel Angle:
               //Now in order to get Accel Angles we use Euler's formula with Acceleration values.
               //*And we substract the error value.

         Accel_AngleX = (atan((Accel_raw_angleY)/sqrt(pow((Accel_raw_angleX),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg) - Accel_angle_error_X;     //X-axis

     //    Accel_AngleY = (atan(-1*(Accel_raw_angleX)/sqrt(pow((Accel_raw_angleY),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg) -  Accel_angle_error_Y; //Y-axis
    
               
         //--------------------- Total Angle :--------------------------//

                //Calculate The Total Angle:
                  Total_AngleX = 0.98 *(Total_AngleX + Gyro_AngleX) + 0.02*Accel_AngleX; //Xaxis
   
               //   Total_AngleY = 0.98 *(Total_AngleY + Gyro_AngleY) + 0.02*Accel_AngleY; //y-axis


                  
              //--------------------- Now Print The Data On Serial Monitor ------------------//
                         /*
                            Serial.print(" Gyro Data (deg)= ");
                            Serial.print(" Gyro_AngleX = ");
                            Serial.print(Gyro_AngleX);
                            Serial.print(" Gyro_AngleY = ");
                            Serial.println(Gyro_AngleY);
                         
                            Serial.print("  Accel Data (deg)= ");
                            Serial.print(" Accel_AngleX = ");
                            Serial.print(Accel_AngleX);
                            Serial.print(" Accel_AngleY = ");
                            Serial.println(Accel_AngleY);                    

                           
                            Serial.print("  Total Angle Data (deg)= ");                        
                            Serial.print("  Total_AngleX = ");                    
                            Serial.print(Total_AngleX); 
                            Serial.print("  Total_AngleY = ");                    
                            Serial.println(Total_AngleY);
                            */    


           //----------------------------- PID CONTROLLER -----------------------------//

     //First We Calculate The error:
       error = Total_AngleX - Desired_Angle;   //Calculate the roll error 
  

    //Now Calculate The Proportional Value of Pitch and Roll PID:
       PID_p = kp*(error);   // Now Multiply Proportional **
    

   //Now Calculate The Intigral Value of Pitch and Roll PID:
    if(-3 < error < 3) {
      PID_i =  PID_i + (ki*error);   // To intigrate we just sum the previous intigral Value with the error
    }
     
 //Now Calculate The Derivative Value of Roll and Pitch PID:
  /*The Last part is derivative control works on how fast or speed the error is 
   * changing.As we know the speed is the amount of error produced in a certain 
   * amount of time and divided by that time.The Formula is:
   * Speed (s) = (e)/ elapsed time, where (e) is actual error which is 
   * obtained by subtracting the error by previous error*/
     PID_d = kd*((error - Previous_error)/elapsedTime);


  //Now Calculate The Final PID Value:
    /*Final PID value is the sum of Proportioal Intigral
     * and Derivative part of PID*/
     PID = PID_p + PID_i + PID_d;


//====================================================================== -: SECTION 04 :- ======================================================================//
                                                              //In This Section we calculate the PWM width & send the signal to ESCs.


     //Now lets set and map pwm signals :
       Joystick = analogRead(A7);
       Joystick = map(Joystick,0,1023,0,255);

     //Now we need to convert jostick integer values into float values: (for calculation purpose)
       Joystick = (float)Joystick;
            //Serial.println(Jostick);

            
      //Now calulate Required PWM width :
        left_PWM  = Joystick - PID;
        right_PWM = Joystick + PID;

       //Now we need to convert pwm float values into integer values: 
         left_PWM  = (int)left_PWM;
         right_PWM = (int)right_PWM;

        /*We know that we can supply minimum PWM signal is o and maximum is 255
     * but after the PWM width calculation PWM signal is exceeding the maximum
     * value so we need to adjust it and we also need to adjust the minimum value 
     * because it is not accurate */
       if(left_PWM >= 255)
         { left_PWM = 255;}
       if(left_PWM < 5)
         { left_PWM = 0;}

       if(right_PWM >= 255)
         { right_PWM = 255;}
       if(right_PWM < 5)
         { right_PWM = 0;}


     // Now Send The Signal To The ESCs:
       /*In this last part of the coding we send the
        * calculated PWM width signal to all the ESCs.
        * we send the signal to the ESCs in CCW order 
        * of the arms & it starts from R_F_ARM */

    /* We made some modifications in code for better control by triggering the PWM
     *  means if the push button is on send the Calculated PWM signal to the esc
     *  otherwise send low value to turn off he motor.
     *  Here Throttle[5] is ButtonState */
        if(Joystick > 5){     
           left_Arm.write(left_PWM);
           right_Arm.write(right_PWM);
       
         } else {   
            left_Arm.write(0);
            right_Arm.write(0);
         }
     
        // Now we store the error in previous error because it nedded for calculation process.  
           Previous_error = error;
       
          
     //--------------------- Now Print The Calculted PWM Width On Serial Monitor ---------------------//

                         Serial.print(" left_Arm = ");
                         Serial.print(left_PWM);
                         Serial.print(" right_Arm = ");
                         Serial.println(right_PWM);
                         
}
