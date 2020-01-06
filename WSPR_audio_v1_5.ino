/* 

WPSR audio signal source to control a SSb transmitter for WSPR beacon operation.

 Generates a WSPR message from call locator and power to a symbol/tone  
 and generates sinewaves with a PWM and software DDS

 ‘On-the-fly’ GPS generation of grid square location combined with the generation of compound 
 callsigns with a 6 digit locator provides for flexible mobile or portable WSPR operation.

Acknowlegements:
 The WSPR special message alorithm was derived from Fortran and C files found the K1JT WSPR source code. 
 Portions of the WSPR message algorithm is the work of Andy Talbot, G4JNT. Portions of the GPS receive 
 code were influenced by Igor Gonzalez Martin's Arduino tutorial. The PWM alorithm is a modified version 
 of DH3JO's wsprgen PWM alogorith
 
 
 Copyright (C) 2013,  Gene Marcus W3PM GM4YRE
 
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 8 September 2013 version 1.1 -  K1FM supplied bug fix to add #include <SerialClass.h> statement that 
                                 allows compilation on Apple Mac machines.
                                
12 October 2013 version 1.2  -   Detach 1pps interrupt during transmit to eliminate short audio droputs.
                                 Time display replaced by "Transmit" during transmit periods. 
                                 
28 October 2013 vewrsion 1.3 -   Corrected bug in grid square calculation algorithm.

25 February 2014 version 1.4 -   Changed message type 2 transmit order. Initial transmission is now 
                                 6 digit locator data. Changed because WSPRnet.org will append last
                                 transmitted 6 digit location when compound callsign is identified. 
                                 
18 May 2015 version 1.5 -        Modified sine flash memory variable to be compatible withe Arduino 1.6.x
                                 Removed unused #include statements.
 _________________________________________________________________________________________________________
 
 UNO Digital Pin Allocation
 D0  GPS RX
 D1  
 D2  1PPS GPS input
 D3  
 D4  
 D5  
 D6  
 D7  
 D8  CW LED
 D9  
 D10 TX PTT
 D11 WSPR Audio out
 D12 
 D13 
 A0/D14 LCD D7
 A1/D15 LCD D6
 A2/D16 LCD D5
 A3/D17 LCD D4
 A4/D18 LCD enable
 A5/D19 LCD RS 
 _______________________________________________________________________________________
 */

#include <LiquidCrystal.h>


//_________________________Enter home callsign and grid square below:_____________________
char call3[13] = "W3PM"; //e.g. "W3PM" or "GM4YRE"
char locator2[7] = "EM64";  // Use 4 character locator e.g. "EM64"

//_________________________Enter portable/mobile callsign below:__________________________
char call4[13] = "W3PM/M"; //e.g. "W3PM/M" or "W4/GM4YRE"
/*
Note:
- Upper or lower case characters are acceptable.
- Compound callsigns may use up to a three letter/number combination prefix followed by a “/”. 
  A one letter or two number suffix may be used preceded by a “/”.
*/

//_________________________Enter power level below:_______________________________________
byte ndbm = 27; // Min = 0 dBm, Max = 43 dBm, steps 0,3,7,10,13,17,20,23,27,30,33,37,40,43


//_________________________Enter audio output frequency in Hz below:______________________
int audioFreq = 1520; // Use any audio frequency between 1400 and 1600 Hz

//_________________________Enter even minute to begin transmission:________________________
byte txTime = 0;  //Select even minute to transmit. Enter either 0,2,4,6, or 8.


// DDS freq table for audio output frequency ( 2^32 * freq in Hz / (16 MHz/510) / 256)
// Constant is nominally 534.77 adjust for actual clock frequency
const unsigned long mt[4] = 
 {
  (audioFreq*534.77-(783.34*2)),
  (audioFreq*534.77-783.34),
  (audioFreq*534.77),
  (audioFreq*534.77+783.34) 
 };

// CW dot length in terms of Timer1 interrupt period 
const byte CWdot = 1;

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(19,18,17,16,15,14);


//! Macro that clears all Timer/Counter1 interrupt flags.
#define CLEAR_ALL_TIMER1_INT_FLAGS    (TIFR1 = TIFR1)


const char SyncVec[162] = {
  1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,
  1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,
  0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,
  0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0
};

// table of 256 sine values / one sine period / stored in flash memory
const PROGMEM  uint8_t sine256[]  = { 
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124
};

/*
Load Morse code send data. 
 The first five bits are data representing the Morse character. 
 
 1 = dit
 0 = dah
 
 The last three bits represent the number of bits in the Morse character.
 */
 
const byte Morse[37] = {
  B11111101,	//	0		
  B01111101,	//	1	
  B00111101,	//	2		
  B00011101,	//	3		
  B00001101,	//	4	
  B00000101,	//	5		
  B10000101,	//	6		
  B11000101,	//	7		
  B11100101,	//	8		
  B11110101,	//	9		
  B01000010,	//	A		
  B10000100,	//	B		
  B10100100,	//	C		
  B10000011,	//	D		
  B00000001,	//	E		
  B00100100,	//	F		
  B11000011,	//	G		
  B00000100,	//	H		
  B00000010,	//	I		
  B01110100,	//	J		
  B10100011,	//	K		
  B01000100,	//	L		
  B11000010,	//	M		
  B10000010,	//	N		
  B11100011,	//	O		
  B01100100,	//	P		
  B11010100,	//	Q		
  B01000011,	//	R		
  B00000011,	//	S		
  B10000001,	//	T		
  B00100011,	//	U		
  B00010100,	//	V		
  B01100011,	//	W		
  B10010100,	//	X		
  B10110100,	//	Y		
  B11000100,	//	Z
  B00000000     //      sp		
};

int led2Pin = 8;          // LED pin 
int t1Pin = 4;            // test pin interrupt        
int t2Pin = 5;            // test pin main      
int pttPin=10;
int state=0;
int GPSpin = 0;            // GPS RX PIN
int byteGPS=-1;
char buffer[300] = "",call2[13],locator[7];
char StartCommand[7] = "$GPGGA",StartCommand2[7] = "$GPRMC";
volatile unsigned long phaccu;  // soft DDS phase accu
volatile unsigned long mm;      // soft DDS frequency word
volatile unsigned int sycnt;
volatile unsigned int tcnt;   // tonespacing timer
volatile byte icnt,icnt2;
volatile byte ii,i,j,type_flag,type2_flag;
volatile byte InhibitFlag = 0,GPSinhibitFlag = 0; // 1 will inhibit transmitter
volatile byte CWcount,MorseChar,MorseBit,CharFlag,ByteMask,MorseBitCount;
volatile byte SpaceCount,WordSpaceFlag;
byte c[11],sym[170],symt[170],symbol[162],calltype;
byte msg_type = 1,txTime2,temp = 1;
int IndiceCount=0,StartCount=0,counter=0;
int indices[13],sat1,sat10;
int second=1,minute=1,minute1=1,hour=0;
int Lat10,Lat1,NS,Lon100,Lon10,Lon1,EW,validGPSflag;
int dlon,mLon10,mLon1,mdLon1,dlat,mLat10,mLat1,mdLat1;
int nadd,nc,n,ntype,MsgLength,bb;
char grid4[5],grid6[7],call1[7],cnt1;
char GPSlocator[7],CWmsg[22];
unsigned long t1,ng,n2,m1,n1,cc1;
long MASK15=32767,ihash;


//******************************************************************
// Clock - GPS 1PPS interrupt routine used as master timekeeper
// Called every second by GPS 1PPS on pin 20
void PPSinterrupt()
{
  second++ ;
  if (second == 60) {
    minute++ ;
    second=0 ;
  }
  if (minute == 60) {
    hour++;
    minute=0 ;
  }
  if (hour == 24) {
    hour=0 ;
  }
  displaytime();
  if(second == 0 & txTime != minute1 & txTime+1 != minute1 )
   {
    calcGridSquare();
    wsprGenCode();
    }
  if(TIMSK1 == 2 & second == 0)
  {
   CWmessage();
   LCDupdate();
  }
 // if(minute%2 == 0 & second == 2) //Used for testing
  if(txTime == minute%10 & second == 2) transmit();       
  if(calltype == 2 & txTime2 == minute%10 & second == 2) 
 { 
  msg_type = !msg_type;
  transmit();
 }
}


//******************************************************************
// Timer1 Overflow Interrupt Vector
// used for CW timing 
ISR(TIMER1_COMPA_vect) 
 // CW transmit routine begins here:
      {
        if(validGPSflag == 0)for (i=0;i<4;i++)CWmsg[i] = 'E';
        MsgLength = (strlen(CWmsg));
        if (MorseChar>MsgLength-1)
        {
          CWcount=0;
          MorseChar=0;
          MorseBit=0;
          CharFlag=0;
        }
        else
          if((CWmsg[MorseChar] >= 97) & (CWmsg[MorseChar] <= 122)) 
          {
            temp = CWmsg[MorseChar]-87;          
          }
          else
            if((CWmsg[MorseChar] >= 65) & (CWmsg[MorseChar] <= 90))
            { 
              temp = CWmsg[MorseChar] - 55;  
            }
            else
              temp = CWmsg[MorseChar] - 48; 
        ByteMask = B00000111;
        MorseBitCount = ByteMask & Morse[temp]; 
        if(CWmsg[MorseChar] == 32)
        {
          WordSpace(); 
        }
        else
          if(bitRead(Morse[temp],(7-MorseBit)) == HIGH)
          {
            Dah();
          }
          else
          {
            Dit();
          }
        if (CharFlag >= 1)
        {
          CharSpace(); 
        }
    }

//******************************************************************
// Timer2 Interrupt Service at 31372.550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : 8 microseconds ( inclusive push and pop)
ISR(TIMER2_OVF_vect) {
  
  sbi(PORTD,4);       // set PORTD,4 high to observe timing with a oscope
  if (tcnt++ >= 21411)// Corrected for actual clock frequency. Nominal = 21417
  { 
    tcnt=0;          // set tone spacing flag at 1,4648 Hz
    sycnt++;
  }
  phaccu=phaccu+mm;   // soft DDS phase accu with 24 bits
  icnt=phaccu >> 16;  // upper 8 bits for pwm modulator
  OCR2A=pgm_read_byte_near(sine256 + icnt);

  bb=sym[sycnt];
  mm=mt[bb];  
  if(sycnt >= 162)
  {
    digitalWrite(pttPin,0);
    TIMSK1 = 2;        // CW Timer1 Interrupt enable
    TIMSK2 = 0;        // Disable Timer2 
    attachInterrupt(0, PPSinterrupt, RISING); // Turn 1pps timer on
  }
  cbi(PORTD,4);      // set PORTD,4
}

void setup()
{
  //Set up Timer1A (CW timing)
  TCCR1B = 0;        //Disable timer during setup
  TIMSK1 = 2;        //Timer1 Interrupt enable
  TCCR1A = 0;        //Normal port operation, Wave Gen Mode normal
  TCCR1B = 12;       //Timer prescaler to 256 - CTC mode
  OCR1A = 10000;     //Adjust CW speed (increase OCR1A to decrease CW speed)
  
    //Set up Timer2 (31372.549 Hz clock = 16MHz/510)
  TIMSK2 = 0;        //Disable timer during setup
  TCCR2A = 161;      //PWM mode phase correct
  TCCR2B = 1;        //Timer prescaler to 1
  
  pinMode(led2Pin, OUTPUT); 
  pinMode(t1Pin, OUTPUT);
  pinMode(t2Pin, OUTPUT);
  pinMode(pttPin, OUTPUT);
  pinMode(11, OUTPUT);   // Audio output

  // set up the LCD for 16 columns and 2 rows 
  lcd.begin(16, 2); 
  
  // Turn on LCD and display default message 
  lcd.display();
  lcd.setCursor(0,1);
  lcd.print("No Data");
  lcd.setCursor(15,1);
  lcd.print("!");   

  //Set up GPS pin
  pinMode(GPSpin, INPUT);
  // digitalWrite(GPSpin, HIGH); // internal pull-up enabled
  
  // Set GPS input to 4800 baud
  Serial.begin(4800);

  // Set 1PPS pin 2 for external interrupt input
  attachInterrupt(0, PPSinterrupt, RISING);
  
  for (int i = 0; i < 13; i++) call2[i] = call3[i]; // Start with home callsign
  for (int i = 0; i < 7; i++) locator[i] = locator2[i]; // Start with home location

  // WSPR message calculation
  wsprGenCode();  

  tcnt=0;
  mm=0;
}


void loop()
{
  if(TIMSK2 == 0) GPSprocess();
} 


/*
_______________________________________________________________
 
 GPS timing process starts here
 _______________________________________________________________
 */
void GPSprocess()
{
  byte pinState = 0;
  byteGPS=Serial.read();         // Read a byte of the serial port
  if (byteGPS == -1) {           // See if the port is empty yet
    delay(100); 
  } 
  else {
    buffer[counter]=byteGPS;        // If there is serial port data, it is put in the buffer
    counter++;                      
    if (byteGPS==13){            // If the received byte is = to 13, end of transmission
      IndiceCount=0;
      StartCount=0;
      for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPGGA
        if (buffer[i]==StartCommand[i-1]){
          StartCount++;
        }
      }
      if(StartCount==6){      // If yes, continue and process the data
        for (int i=0;i<300;i++){
          if (buffer[i]==','){    // check for the position of the  "," separator
            indices[IndiceCount]=i;
            IndiceCount++;
          }
          if (buffer[i]=='*'){    // ... and the "*"
            indices[12]=i;
            IndiceCount++;
          }
        }
        // Load time data
        temp = indices[0];
        hour = (buffer[temp+1]-48)*10 + buffer[temp+2]-48;
        minute1 = buffer[temp+4]-48;
        minute = (buffer[temp+3]-48)*10 + buffer[temp+4]-48;
        second = (buffer[temp+5]-48)*10 + buffer[temp+6]-48; 
        // Load latitude and logitude data          
        temp = indices[1];
        Lat10 = buffer[temp+1]-48;
        Lat1 = buffer[temp+2]-48;
        mLat10 = buffer[temp+3]-48;
        mLat1 = buffer[temp+4]-48;
        mdLat1 = buffer[temp+6]-48;
        temp = indices[2];
        NS = buffer[temp+1];
        temp = indices[3];
        Lon100 = buffer[temp+1]-48;
        Lon10 = buffer[temp+2]-48;
        Lon1 = buffer[temp+3]-48; 
        mLon10 = buffer[temp+4]-48;
        mLon1 = buffer[temp+5]-48; 
        mdLon1 = buffer[temp+7]-48;        
        temp = indices[4];
        EW = buffer[temp+1];
        temp = indices[5];
        validGPSflag = buffer[temp+1]-48;
        temp = indices[6];
        sat10 = buffer[temp+1]-48;
        sat1 = buffer[temp+2]-48;        
      }

      else
      {
        IndiceCount=0;
        StartCount=0;
        for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPRMC
          if (buffer[i]==StartCommand2[i-1]){
            StartCount++;
          }
        }       
        if(StartCount==6){      // If yes, continue and process the data
          for (int i=0;i<300;i++){
            if (buffer[i]==','){    // check for the position of the  "," separator
              indices[IndiceCount]=i;
              IndiceCount++;
            }
            if (buffer[i]=='*'){    // ... and the "*"
              indices[12]=i;
              IndiceCount++;
            }
          }
          // Load time data
          temp = indices[0];
          hour = (buffer[temp+1]-48)*10 + buffer[temp+2]-48;
          minute1 = buffer[temp+4]-48; 
          minute = (buffer[temp+3]-48)*10 + buffer[temp+4]-48;
          second = (buffer[temp+5]-48)*10 + buffer[temp+6]-48;
          temp = indices[1]; 
          if (buffer[temp+1] == 65){
            validGPSflag = 1; 
          }
          else
          {
            validGPSflag = 0;
          }        
          // Load latitude and logitude data          
          temp = indices[2];
          Lat10 = buffer[temp+1]-48;
          Lat1 = buffer[temp+2]-48;
          mLat10 = buffer[temp+3]-48;
          mLat1 = buffer[temp+4]-48;
          mdLat1 = buffer[temp+6]-48;
          temp = indices[3];
          NS = buffer[temp+1];
          temp = indices[4];
          Lon100 = buffer[temp+1]-48;
          Lon10 = buffer[temp+2]-48;
          Lon1 = buffer[temp+3]-48;
          mLon10 = buffer[temp+4]-48;
          mLon1 = buffer[temp+5]-48; 
          mdLon1 = buffer[temp+7]-48;         
          temp = indices[5];
          EW = buffer[temp+1];
        }
      }

      if(validGPSflag ==1)GPSinhibitFlag = 0;
      else
      {
        GPSinhibitFlag = 1;
      }
      counter=0;                  // Reset the buffer
      for (int i=0;i<300;i++){    //  
        buffer[i]=' '; 
      }

    }
  }
}

//******************************************************************
void displaytime()
{
  lcd.setCursor(0,0);
  if (hour < 10) lcd.print ("0");
  lcd.print (hour);
  lcd.print (":");
  if (minute < 10) lcd.print ("0");
  lcd.print (minute);
  lcd.print (":");
  if (second < 10) lcd.print ("0");
  lcd.print (second);
  lcd.print (" "); 

  return;  
}


//******************************************************************
void wsprGenCode()
{
  for(i=0;i<13;i++){if(call2[i] == 47)calltype=2;};
  if(calltype == 2) type2();
  else
  {
  for(i=0;i<7;i++){call1[i] = call2[i]; };
  for(i=0;i<5;i++){
  grid4[i] = locator[i];
  };
  packcall();
  packgrid();
  n2=ng*128+ndbm+64;
  pack50();
  encode_conv();
  interleave_sync();
  }
}

//******************************************************************
void type2()
{
  if(msg_type == 0)
  {
  packpfx();
  ntype=ndbm + 1 + nadd;
  n2 = 128*ng + ntype + 64;
  pack50();
  encode_conv();
  interleave_sync();  
  }
  else
  {
    hash();
    for(ii=1;ii<6;ii++)
    {
      call1[ii-1]=locator[ii];
    };
    call1[5]=locator[0];
    packcall();
    ntype=-(ndbm+1);
    n2=128*ihash + ntype +64;
    pack50();
    encode_conv();
    interleave_sync();
  };

}

//******************************************************************
void packpfx()
{
  char pfx[3];
  int Len;
  int slash;

  for(i=0;i<7;i++)
  {
    call1[i]=0;
  };
  Len = strlen(call2);
  for(i=0;i<13;i++)
  {
    if(call2[i] == 47) slash = i;
  };
  if(call2[slash+2] == 0)
  {//single char add-on suffix
    for(i=0;i<slash;i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    nadd=1;
    nc=int(call2[slash+1]);
    if(nc>=48 && nc<=57) n=nc-48;
    else if(nc>=65 && nc<=90) n=nc-65+10;
    else if (nc>=97 && nc<=122) n=nc-97+10;
    else n=38;
    ng=60000-32768+n;
  }
  else
    if(call2[slash+3] == 0)
    {
     for(i=0;i<slash;i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    n=10*(int(call2[slash+1])-48) +  int(call2[slash+2])-48;
    nadd=1;
    ng=60000 + 26 + n;
    }
    else
    {
     for(i=0;i<slash;i++)
    {
     pfx[i] = call2[i];
    };
    if (slash == 2)
    {
     pfx[2] = pfx[1];
     pfx[1] = pfx[0];
     pfx[0] = ' ';
    };
    if (slash == 1)
    {
     pfx[2] = pfx[0];
     pfx[1] = ' ';
     pfx[0] = ' ';
    };
     ii=0;
     for(i=slash+1;i<Len;i++)
    {
      call1[ii] = call2[i];
      ii++;
     };
    packcall();
    ng=0;
    for(i=0;i<3;i++)
    {
     nc=int(pfx[i]);
    if(nc>=48 && nc<=57) n=nc-48;
    else if(nc>=65 && nc<=90) n=nc-65+10;
    else if (nc>=97 && nc<=122) n=nc-97+10;
    else n=36;     
    ng=37*ng+n;
    };
    nadd=0;
    if(ng >= 32768)
    {
     ng=ng-32768;
     nadd=1; 
    };
   }
 }

//******************************************************************
void packcall()
{
  // coding of callsign
  if (chr_normf(call1[2]) > 9) 
  {
    call1[5] = call1[4];
    call1[4] = call1[3]; 
    call1[3] = call1[2];
    call1[2] = call1[1];
    call1[1] = call1[0];
    call1[0] = ' ';
  }

  n1=chr_normf(call1[0]);
  n1=n1*36+chr_normf(call1[1]);
  n1=n1*10+chr_normf(call1[2]);
  n1=n1*27+chr_normf(call1[3])-10;
  n1=n1*27+chr_normf(call1[4])-10;
  n1=n1*27+chr_normf(call1[5])-10;
}

//******************************************************************
void packgrid()
{
  // coding of grid4
  ng=179-10*(chr_normf(grid4[0])-10)-chr_normf(grid4[2]);
  ng=ng*180+10*(chr_normf(grid4[1])-10)+chr_normf(grid4[3]);
}

//******************************************************************
void pack50()
{
  // merge coded callsign into message array c[]
  t1=n1;
  c[0]= t1 >> 20;
  t1=n1;
  c[1]= t1 >> 12;
  t1=n1;
  c[2]= t1 >> 4;
  t1=n1;
  c[3]= t1 << 4;
  t1=n2;
  c[3]= c[3] + ( 0x0f & t1 >> 18);
  t1=n2;
  c[4]= t1 >> 10;
  t1=n2;
  c[5]= t1 >> 2;
  t1=n2;
  c[6]= t1 << 6;
}

//******************************************************************
//void hash(string,len,ihash)
void hash()
{
  int Len;
  uint32_t jhash;
  int *pLen = &Len;
  Len = strlen(call2); 
  byte IC[12];
  byte *pIC = IC;
  for (i=0;i<12;i++)
  {
    pIC + 1;
    &IC[i];
  }
  uint32_t Val = 146;
  uint32_t *pVal = &Val;
  for(i=0;i<Len;i++)
  {
    IC[i] = int(call2[i]);
  };
  jhash=nhash_(pIC,pLen,pVal);
  ihash=jhash&MASK15;
  return;
}
//******************************************************************
// normalize characters 0..9 A..Z Space in order 0..36
char chr_normf(char bc ) 
{
  char cc=36;
  if (bc >= '0' && bc <= '9') cc=bc-'0';
  if (bc >= 'A' && bc <= 'Z') cc=bc-'A'+10;
  if (bc >= 'a' && bc <= 'z') cc=bc-'a'+10;  
  if (bc == ' ' ) cc=36;

  return(cc);
}


//******************************************************************
// convolutional encoding of message array c[] into a 162 bit stream
void encode_conv()
{
  int bc=0;
  int cnt=0;
  int cc;
  unsigned long sh1=0;

  cc=c[0];

  for (int i=0; i < 81;i++) {
    if (i % 8 == 0 ) {
      cc=c[bc];
      bc++;
    }
    if (cc & 0x80) sh1=sh1 | 1;

    symt[cnt++]=parity(sh1 & 0xF2D05351);
    symt[cnt++]=parity(sh1 & 0xE4613C47);

    cc=cc << 1;
    sh1=sh1 << 1;
  }
}

//******************************************************************
byte parity(unsigned long li)
{
  byte po = 0;
  while(li != 0)
  {
    po++;
    li&= (li-1);
  }
  return (po & 1);
}

//******************************************************************
// interleave reorder the 162 data bits and and merge table with the sync vector
void interleave_sync()
{
  int ii,ij,b2,bis,ip;
  ip=0;

  for (ii=0;ii<=255;ii++) {
    bis=1;
    ij=0;
    for (b2=0;b2 < 8 ;b2++) {
      if (ii & bis) ij= ij | (0x80 >> b2);
      bis=bis << 1;
    }
    if (ij < 162 ) {
      sym[ij]= SyncVec[ij] +2*symt[ip];
      ip++;
    }
  }
}

/*
-------------------------------------------------------------------------------
lookup3.c, by Bob Jenkins, May 2006, Public Domain.

These are functions for producing 32-bit hashes for hash table lookup.
hashword(), hashlittle(), hashlittle2(), hashbig(), mix(), and final() 
are externally useful functions.  Routines to test the hash are included 
if SELF_TEST is defined.  You can use this free for any purpose.  It's in
the public domain.  It has no warranty.

You probably want to use hashlittle().  hashlittle() and hashbig()
hash byte arrays.  hashlittle() is is faster than hashbig() on
little-endian machines.  Intel and AMD are little-endian machines.
On second thought, you probably want hashlittle2(), which is identical to
hashlittle() except it returns two 32-bit hashes for the price of one.  
You could implement hashbig2() if you wanted but I haven't bothered here.

If you want to find a hash of, say, exactly 7 integers, do
  a = i1;  b = i2;  c = i3;
  mix(a,b,c);
  a += i4; b += i5; c += i6;
  mix(a,b,c);
  a += i7;
  final(a,b,c);
then use c as the hash value.  If you have a variable length array of
4-byte integers to hash, use hashword().  If you have a byte array (like
a character string), use hashlittle().  If you have several byte arrays, or
a mix of things, see the comments above hashlittle().  

Why is this so big?  I read 12 bytes at a time into 3 4-byte integers, 
then mix those integers.  This is fast (you can do a lot more thorough
mixing with 12*3 instructions on 3 integers than you can with 3 instructions
on 1 byte), but shoehorning those bytes into integers efficiently is messy.
-------------------------------------------------------------------------------
*/

//#define SELF_TEST 1

//#include <stdio.h>      /* defines printf for tests */
//#include <time.h>       /* defines time_t for timings in the test */
//#ifdef Win32
//#include "win_stdint.h"	/* defines uint32_t etc */
//#else
//#include <stdint.h>	/* defines uint32_t etc */
//#endif

//#include <sys/param.h>  /* attempt to define endianness */
//#ifdef linux
//# include <endian.h>    /* attempt to define endianness */
//#endif

#define HASH_LITTLE_ENDIAN 1

#define hashsize(n) ((uint32_t)1<<(n))
#define hashmask(n) (hashsize(n)-1)
#define rot(x,k) (((x)<<(k)) | ((x)>>(32-(k))))

/*
-------------------------------------------------------------------------------
mix -- mix 3 32-bit values reversibly.

This is reversible, so any information in (a,b,c) before mix() is
still in (a,b,c) after mix().

If four pairs of (a,b,c) inputs are run through mix(), or through
mix() in reverse, there are at least 32 bits of the output that
are sometimes the same for one pair and different for another pair.
This was tested for:
* pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
* the base values were pseudorandom, all zero but one bit set, or 
  all zero plus a counter that starts at zero.

Some k values for my "a-=c; a^=rot(c,k); c+=b;" arrangement that
satisfy this are
    4  6  8 16 19  4
    9 15  3 18 27 15
   14  9  3  7 17  3
Well, "9 15 3 18 27 15" didn't quite get 32 bits diffing
for "differ" defined as + with a one-bit base and a two-bit delta.  I
used http://burtleburtle.net/bob/hash/avalanche.html to choose 
the operations, constants, and arrangements of the variables.

This does not achieve avalanche.  There are input bits of (a,b,c)
that fail to affect some output bits of (a,b,c), especially of a.  The
most thoroughly mixed value is c, but it doesn't really even achieve
avalanche in c.

This allows some parallelism.  Read-after-writes are good at doubling
the number of bits affected, so the goal of mixing pulls in the opposite
direction as the goal of parallelism.  I did what I could.  Rotates
seem to cost as much as shifts on every machine I could lay my hands
on, and rotates are much kinder to the top and bottom bits, so I used
rotates.
-------------------------------------------------------------------------------
*/
#define mix(a,b,c) \
{ \
  a -= c;  a ^= rot(c, 4);  c += b; \
  b -= a;  b ^= rot(a, 6);  a += c; \
  c -= b;  c ^= rot(b, 8);  b += a; \
  a -= c;  a ^= rot(c,16);  c += b; \
  b -= a;  b ^= rot(a,19);  a += c; \
  c -= b;  c ^= rot(b, 4);  b += a; \
}

/*
-------------------------------------------------------------------------------
final -- final mixing of 3 32-bit values (a,b,c) into c

Pairs of (a,b,c) values differing in only a few bits will usually
produce values of c that look totally different.  This was tested for
* pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
* the base values were pseudorandom, all zero but one bit set, or 
  all zero plus a counter that starts at zero.

These constants passed:
 14 11 25 16 4 14 24
 12 14 25 16 4 14 24
and these came close:
  4  8 15 26 3 22 24
 10  8 15 26 3 22 24
 11  8 15 26 3 22 24
-------------------------------------------------------------------------------
*/
#define final(a,b,c) \
{ \
  c ^= b; c -= rot(b,14); \
  a ^= c; a -= rot(c,11); \
  b ^= a; b -= rot(a,25); \
  c ^= b; c -= rot(b,16); \
  a ^= c; a -= rot(c,4);  \
  b ^= a; b -= rot(a,14); \
  c ^= b; c -= rot(b,24); \
}

/*
-------------------------------------------------------------------------------
hashlittle() -- hash a variable-length key into a 32-bit value
  k       : the key (the unaligned variable-length array of bytes)
  length  : the length of the key, counting by bytes
  initval : can be any 4-byte value
Returns a 32-bit value.  Every bit of the key affects every bit of
the return value.  Two keys differing by one or two bits will have
totally different hash values.

The best hash table sizes are powers of 2.  There is no need to do
mod a prime (mod is sooo slow!).  If you need less than 32 bits,
use a bitmask.  For example, if you need only 10 bits, do
  h = (h & hashmask(10));
In which case, the hash table should have hashsize(10) elements.

If you are hashing n strings (uint8_t **)k, do it like this:
  for (i=0, h=0; i<n; ++i) h = hashlittle( k[i], len[i], h);

By Bob Jenkins, 2006.  bob_jenkins@burtleburtle.net.  You may use this
code any way you wish, private, educational, or commercial.  It's free.

Use for hash table lookup, or anything where one collision in 2^^32 is
acceptable.  Do NOT use for cryptographic purposes.
-------------------------------------------------------------------------------
*/

//uint32_t hashlittle( const void *key, size_t length, uint32_t initval)
#ifdef STDCALL
uint32_t __stdcall NHASH( const void *key, size_t *length0, uint32_t *initval0)
#else
uint32_t nhash_( const void *key, int *length0, uint32_t *initval0)
#endif
{
  uint32_t a,b,c;                                          /* internal state */
  size_t length;
  uint32_t initval;
  union { const void *ptr; size_t i; } u;     /* needed for Mac Powerbook G4 */

  length=*length0;
  initval=*initval0;

  /* Set up the internal state */
  a = b = c = 0xdeadbeef + ((uint32_t)length) + initval;

  u.ptr = key;
  if (HASH_LITTLE_ENDIAN && ((u.i & 0x3) == 0)) {
    const uint32_t *k = (const uint32_t *)key;         /* read 32-bit chunks */
    const uint8_t  *k8;

    k8=0;                                     //Silence compiler warning
    /*------ all but last block: aligned reads and affect 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      b += k[1];
      c += k[2];
      mix(a,b,c);
      length -= 12;
      k += 3;
    }

    /*----------------------------- handle the last (probably partial) block */
    /* 
     * "k[2]&0xffffff" actually reads beyond the end of the string, but
     * then masks off the part it's not allowed to read.  Because the
     * string is aligned, the masked-off tail is in the same word as the
     * rest of the string.  Every machine with memory protection I've seen
     * does it on word boundaries, so is OK with this.  But VALGRIND will
     * still catch it and complain.  The masking trick does make the hash
     * noticably faster for short strings (like English words).
     */
#ifndef VALGRIND

    switch(length)
    {
    case 12: c+=k[2]; b+=k[1]; a+=k[0]; break;
    case 11: c+=k[2]&0xffffff; b+=k[1]; a+=k[0]; break;
    case 10: c+=k[2]&0xffff; b+=k[1]; a+=k[0]; break;
    case 9 : c+=k[2]&0xff; b+=k[1]; a+=k[0]; break;
    case 8 : b+=k[1]; a+=k[0]; break;
    case 7 : b+=k[1]&0xffffff; a+=k[0]; break;
    case 6 : b+=k[1]&0xffff; a+=k[0]; break;
    case 5 : b+=k[1]&0xff; a+=k[0]; break;
    case 4 : a+=k[0]; break;
    case 3 : a+=k[0]&0xffffff; break;
    case 2 : a+=k[0]&0xffff; break;
    case 1 : a+=k[0]&0xff; break;
    case 0 : return c;              /* zero length strings require no mixing */
    }

#else /* make valgrind happy */

    k8 = (const uint8_t *)k;
    switch(length)
    {
    case 12: c+=k[2]; b+=k[1]; a+=k[0]; break;
    case 11: c+=((uint32_t)k8[10])<<16;  /* fall through */
    case 10: c+=((uint32_t)k8[9])<<8;    /* fall through */
    case 9 : c+=k8[8];                   /* fall through */
    case 8 : b+=k[1]; a+=k[0]; break;
    case 7 : b+=((uint32_t)k8[6])<<16;   /* fall through */
    case 6 : b+=((uint32_t)k8[5])<<8;    /* fall through */
    case 5 : b+=k8[4];                   /* fall through */
    case 4 : a+=k[0]; break;
    case 3 : a+=((uint32_t)k8[2])<<16;   /* fall through */
    case 2 : a+=((uint32_t)k8[1])<<8;    /* fall through */
    case 1 : a+=k8[0]; break;
    case 0 : return c;
    }

#endif /* !valgrind */

  } else if (HASH_LITTLE_ENDIAN && ((u.i & 0x1) == 0)) {
    const uint16_t *k = (const uint16_t *)key;         /* read 16-bit chunks */
    const uint8_t  *k8;

    /*--------------- all but last block: aligned reads and different mixing */
    while (length > 12)
    {
      a += k[0] + (((uint32_t)k[1])<<16);
      b += k[2] + (((uint32_t)k[3])<<16);
      c += k[4] + (((uint32_t)k[5])<<16);
      mix(a,b,c);
      length -= 12;
      k += 6;
    }

    /*----------------------------- handle the last (probably partial) block */
    k8 = (const uint8_t *)k;
    switch(length)
    {
    case 12: c+=k[4]+(((uint32_t)k[5])<<16);
             b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 11: c+=((uint32_t)k8[10])<<16;     /* fall through */
    case 10: c+=k[4];
             b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 9 : c+=k8[8];                      /* fall through */
    case 8 : b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 7 : b+=((uint32_t)k8[6])<<16;      /* fall through */
    case 6 : b+=k[2];
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 5 : b+=k8[4];                      /* fall through */
    case 4 : a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 3 : a+=((uint32_t)k8[2])<<16;      /* fall through */
    case 2 : a+=k[0];
             break;
    case 1 : a+=k8[0];
             break;
    case 0 : return c;                     /* zero length requires no mixing */
    }

  } else {                        /* need to read the key one byte at a time */
    const uint8_t *k = (const uint8_t *)key;

    /*--------------- all but the last block: affect some 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      a += ((uint32_t)k[1])<<8;
      a += ((uint32_t)k[2])<<16;
      a += ((uint32_t)k[3])<<24;
      b += k[4];
      b += ((uint32_t)k[5])<<8;
      b += ((uint32_t)k[6])<<16;
      b += ((uint32_t)k[7])<<24;
      c += k[8];
      c += ((uint32_t)k[9])<<8;
      c += ((uint32_t)k[10])<<16;
      c += ((uint32_t)k[11])<<24;
      mix(a,b,c);
      length -= 12;
      k += 12;
    }

    /*-------------------------------- last block: affect all 32 bits of (c) */
    switch(length)                   /* all the case statements fall through */
    {
    case 12: c+=((uint32_t)k[11])<<24;
    case 11: c+=((uint32_t)k[10])<<16;
    case 10: c+=((uint32_t)k[9])<<8;
    case 9 : c+=k[8];
    case 8 : b+=((uint32_t)k[7])<<24;
    case 7 : b+=((uint32_t)k[6])<<16;
    case 6 : b+=((uint32_t)k[5])<<8;
    case 5 : b+=k[4];
    case 4 : a+=((uint32_t)k[3])<<24;
    case 3 : a+=((uint32_t)k[2])<<16;
    case 2 : a+=((uint32_t)k[1])<<8;
    case 1 : a+=k[0];
             break;
    case 0 : return c;
    }
  }

  final(a,b,c);
  return c;
}

//uint32_t __stdcall NHASH(const void *key, size_t length, uint32_t initval)


/*
//******************************************************************
// Alternate grid square calculator 
// (may create errors near cardinal points due to float rounding)
//******************************************************************

void calcGridSquare()
{ 
  float latitude, longitude;
  longitude = (Lon100*100+Lon10*10+Lon1)+((mLon10*10+mLon1)/60)+mdLon1/600;
  latitude = (Lat10*10+Lat1)+((mLat10*10+mLat1)/60)+mdLat1/600;
  if (EW == 69) longitude = longitude + 180;
  if (EW == 87)longitude=180-longitude;
  if (NS == 78)latitude = latitude + 90;
  if (NS == 83)latitude = 90-latitude;
  GPSlocator[0] = 65 + int(longitude / 20);
  GPSlocator[1] = 65 + int(latitude / 10);
  GPSlocator[2] = 48 + int((int(longitude) % 20)/2);
  GPSlocator[3] = 48 + int((int(latitude) % 10)/1);
  GPSlocator[4] = 65 + int((longitude - (int(longitude/2)*2)) / 0.08333);
  GPSlocator[5] = 65 + int((latitude - (int(latitude/1)*1)) / 0.04166); 
  if(strcmp(locator, GPSlocator)  != 0)  
  {
   for (int i = 0; i < 7; i++)locator[i] = GPSlocator[i];
   for (int i = 0; i < 4; i++)
   {
    if(locator[i] != locator2[i]) // Compare current location with home location
    {
     for (int j=0; j<13; j++) 
     {
       call2[j] = call4[j]; //Away from home, use portable/mobile callsign
       i = 4;
     }
    }
   }
  }
}
  */

//******************************************************************
void calcGridSquare()
{
  unsigned long latitude, longitude;
  float temp3,tempLat,tempLong;
  longitude = Lon100*100000000L+Lon10*10000000L+Lon1*1000000L+mLon10*100000L+mLon1*10000L;
  latitude = Lat10*10000000L+Lat1*1000000L+mLat10*100000L+mLat1*10000L;
  tempLong=longitude;
  tempLong=1000000*int(tempLong/1000000)+ ((tempLong-1000000*int(tempLong/1000000))/0.6)+int(mdLon1*100000/60); 
  if (EW == 69)tempLong=(tempLong)+180000000;
  if (EW == 87)tempLong=180000000-(tempLong);
  tempLat=latitude;
  tempLat=1000000*int(tempLat/1000000)+((tempLat-1000000*int(tempLat/1000000))/0.6)+int(mdLat1*100000/60);   
  if (NS==78)tempLat=tempLat+90000000;
  if (NS==83)tempLat=90000000-tempLat;
  GPSlocator[0]=(65+int(tempLong/20000000));
  GPSlocator[1]=(65+int(tempLat/10000000));
  temp3=tempLong-(20000000*int(tempLong/20000000)); 
  GPSlocator[2]=(48+int(temp3*10/20/1000000)); 
  temp3=tempLat-(10000000*int(tempLat/10000000));  
  GPSlocator[3]=(48+int(temp3/1000000)); 
  temp3=(tempLong/2000000)-(int(tempLong/2000000));
  GPSlocator[4]=(65+int(temp3*24));
  temp3=(tempLat/1000000)-(int(tempLat/1000000));
  GPSlocator[5]=(65+int(temp3*24));
  if(strcmp(locator, GPSlocator)  != 0)  
  {
   for (int i = 0; i < 7; i++)locator[i] = GPSlocator[i];
   for (int i = 0; i < 4; i++)
   {
    if(locator[i] != locator2[i]) // Compare current location with home location
    {
     for (int j=0; j<13; j++) 
     {
       call2[j] = call4[j]; //Away from home, use portable/mobile callsign
       i = 4;
     }
    }
   }
  }
}

//******************************************************************
void Dah()
{
  digitalWrite(led2Pin, HIGH);
  CWcount++;
  if(CWcount > (CWdot*4))
  {
      digitalWrite(led2Pin, LOW);
    SpaceCount++;
    if(SpaceCount > CWdot)
    {
      CWcount=0;
      MorseBit++;
      SpaceCount=0;
    }
  }
  if(MorseBit > MorseBitCount-1)
  {
    MorseBit=0;
    MorseChar++;
    CharFlag=1;
  }
  return;
}

//******************************************************************
void Dit()
{
  digitalWrite(led2Pin, HIGH);
  CWcount++;
  if(CWcount > CWdot)
  {
    digitalWrite(led2Pin, LOW);
    SpaceCount++;
    if(SpaceCount > CWdot)
    {
      CWcount=0;
      MorseBit++;
      SpaceCount=0;
    }
  }
  if(MorseBit > MorseBitCount-1)
  {
    MorseBit=0;
    MorseChar++;
    CharFlag=1;
  }
  return;
}

//******************************************************************
void CharSpace()
{
  digitalWrite(led2Pin, LOW);
  CharFlag++;
  if(CharFlag > CWdot*6)
  {
    CharFlag=0;
    MorseBit=0;
    CWcount=0;
    SpaceCount=0;
  }
  return;
}


//******************************************************************
void WordSpace()
{
  digitalWrite(led2Pin, LOW);
  CharFlag=0;
  WordSpaceFlag++;
  if(WordSpaceFlag > (CWdot*8))
  {
    MorseChar++;
    WordSpaceFlag=0;
    CWcount=0;
    MorseBit=0;
  }
  return;
}

//******************************************************************
void CWmessage()
{
        CWmsg[0] = ' ';
        for (i=1;i<7;i++)CWmsg[i] = locator[i-1];
        for (i=7;i<10;i++)CWmsg[i] = ' ';
        CWmsg[10] = hour/10+48;
        CWmsg[11] = hour%10+48;
        CWmsg[12] = minute/10+48;
        CWmsg[13] = minute%10+48;
        for (i=14;i<17;i++)CWmsg[i] = ' ';
        CWmsg[17] = 'S';
        CWmsg[18] = sat1+48;
        for (i=19;i<21;i++)CWmsg[i] = ' ';
}


//******************************************************************
void LCDupdate()
{
        lcd.clear();
        lcd.setCursor(9,0); 
        lcd.print("Idle");
        lcd.setCursor(14,0);
        lcd.print(sat10);
        lcd.print(sat1);
        lcd.setCursor(0,1);
        lcd.print(locator);
        lcd.print(" ");
        lcd.print(Lat10);
        lcd.print(Lat1);
        lcd.write(NS);
        lcd.print(" ");
        lcd.print(Lon100);
        lcd.print(Lon10);
        lcd.print(Lon1);
        lcd.write(EW);
        if(validGPSflag != 0)lcd.write("*");
        else lcd.write("!");
}

//******************************************************************
// Determine if it is time to transmit. If so, determine if it is
// time to transmit the QRSS or the WSPR message

void transmit()
{
   if(validGPSflag != 0)
   {
     calcGridSquare();
     wsprGenCode();
     lcd.setCursor(0,0); 
     lcd.print("Transmit     "); 
     digitalWrite(pttPin,1);
     digitalWrite(led2Pin, HIGH);
     txTime2 = txTime + 2;
     if (txTime2 == 10) txTime2 = 0;
     detachInterrupt(0); // 1pps intrrupt disable
     TIMSK1 = 0;         //CW Timer1 Interrupt disable
     tcnt=0;  
     sycnt =0;
     TIMSK2 = 1;         // Enable Timer2
   }
}
     


