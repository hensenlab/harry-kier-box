/*
  ADCDAC 
  (C) Copyright 2023-2024 ELD/LION, Leiden University

  Written by       : Harry Visser & Arno van Amersfoort
  Target compiler  : Arduino IDE
  Target platform  : Teensy 4.1
  Dependencies     : spi.h
  Initial date     : June 20, 2024
  Last modified    : June 28, 2024
  
  // ***** Input ADC0
  // ***** Output DAC0=referentie  ; DAC1=2*f0  ; DAC2=fase -180 ... +180 =-1.8 ...+1.8 V   ; =modulus  ; 
  
*/

#include <Arduino.h>
#include <IntervalTimer.h>
#include <adc_dac_io.h>

ADCDAC        adc_dac;                          // Uit adc_dac_io.h bibliotheek
IntervalTimer sample_timer;                     // De timer routine

int32_t       ADC[ADC_COUNT];                   // Array van gemeten integer waarden ADC[0] ..ADC[3] 
int32_t       DAC[DAC_COUNT];

EventResponder g_SPI0Event;
EventResponder g_SPI1Event;
ADCDAC adc_dac(g_SPI0Event, g_SPI1Event);

// ################################################################################################################
// ***** Door de gebruiker te specificeren instellingen: *****

// ***** Referentie bron *****
float f0=3e3 ;                                 	// Hertz frekentie 1 Hz ... 20 kHz. De frekwentie is verondersteld konstnt te blijven.
float refampl=5;                              	// Volt amplitude
float refoffset=0. ;                            // Volt DC offset -10.<=refoffset+refampl<=10
int   CosOfBlok=0 ;                             // 0=cosinus ; 1=blokgolf

// ***** Lockin instelling *****
int   harmonische=0 ;     	                // detektie frekwentie = f0*(1+harmonische)   0<=harmonische<=4.   Bedenk fref*nharm < 20 kHz
float gain=1. ;                              	// 1...1e5
float ruisbandbreedte=1 ;                   	// 1e-3...1e3 Hertz  
int   nRC=2 ;             	                // 1=6 dB/oktaaf ; 2=12 dB/oktaaf, gebruikelijke instelling
                                                // enkel RC filter is van nut binnen regellussen.
float framehoek=0. ;                            // graden, stand van het detektie frame t.o.v. fo*(1+harmonische)
float OutputoffsetI=0. ;                        // Volt
float OutputoffsetQ=0. ;                        // Volt
// ################################################################################################################

// ***** Globale variabelen en konstanten in het programma *****
const float pi= 3.141592653589793 ;		// Zonder pi geen goniometrie
const float fs=1.e5 ;                           // Door de hardware bepaalde bemonsteringsfrekwentie.     

// ***** Timingszaken *****
// Volatile maakt gebruik van de volatile variabele in de interrupt routine mogelijk. Starter voor het achtergrond programma.
volatile int  teller ;			        // afteller van het aantal interrupts om 1 sec af te tellen
const int     presetteller=fs/1;               	// output naar monitor, elke 1/1 sec

// ***** Lock-in en CORDIC referentie oscillator *****
float input=0.0 ,                               // Signaal input
      framerotatie[2] ,                         // t.b.v. draaiing van referentieframe.
      LockinOutput[4];                          // float  I-Q ,  modulus en fase respektievelijk element 0,1,2,3
double k1,k2 ;                                  // Omgewerkte tijdkonstante van de lock-in. zie setup


// ***** T.b.v. CORDIC oscillator f0 en de eerste 4 harmonischen *****
float Retabel[5],                               // Reele delen van de roterende eenheidsvektoren
      Imtabel[5],                               // Imaginaire delen van de roterende eenheidsvektoren
      refrotatie[2],                            // t.b.v. Cordic referentie f0 generatie
      
      excitatie;                                // excitatie, het extern beschikbare referentie signaal via DAC0



// **************************************************************************************
void Lockin(float Retabel[5],                   // Reele delen van de roterende eenheidsvektoren
            float Imtabel[5],                   // Imaginaire delen van de roterende eenheidsvektoren
            int harmonische,                    // fdetektie=f0*(1+harmonische)
            float faserotatie[2],               // De fasedraaing t.o.v. van de referentie
            float LockinOutput[4],              // Infase, Quadratuur , modulus en fase
            float input ){                      // Het ingangssignaal van de lock-in

  float I,Q ;                                 	// Infase en Quadratuur output van de lock-in
  static double cbuf0,cbuf1,                    // Buffers voor het I kanaal, het geheugen van het RC filter ekwivalent
                sbuf0,sbuf1 ;                   // Buffers voor het Q kanaal, het geheugen van het RC filter ekwivalent
  double  a ;                                   // dummy variabele

  
  input=input*2.*gain ;                         // kompensatie voor (1/sqrt(2))^2, de versterking van de lock-ins.
  
  // ***** Infase detektie *****
  a=double(Retabel[harmonische]*input) ;  	// ***** input* cos (f0*(1+harmonische)) *****
 
  cbuf0=cbuf0*k2+a*k1 ;                         // 1st RC filter
  cbuf1=cbuf1*k2+cbuf0*k1 ;                     // 2de RC filter
  if (nRC==1){I=float(cbuf0) ; }                // Van buffer naar float I.
  else       {I=float(cbuf1) ; }                // Van buffer naar float I, ingeval van dubbel RC filter.
  
  // ***** Quadratuur detektie *****
  a=double(Imtabel[harmonische]*input) ;        // ***** input* sin (f0*(1+harmonische)) *****
  
  sbuf0=sbuf0*k2+a*k1 ;                         // 1st RC filter
  sbuf1=sbuf1*k2+sbuf0*k1 ;                     // 2de RC filter 
  if (nRC==1) {Q=float(sbuf0) ; }               // Van buffer naar float Q.
  else        {Q=float(sbuf1) ; }               // Van buffer naar float Q, ingeval van dubbel RC filter.
                 
  // ***** referentie-frame rotatie wordt op I en Q uitgangen uitgevoerd. ******
  a= framerotatie[0]*I+framerotatie[1]*Q ;
  Q=-framerotatie[1]*I+framerotatie[0]*Q ;
  I=a ; 
  
  // ***** Opslaan van de resulaten in het array LockinOutput *****
  LockinOutput[0]=I ;                           // In fase
  LockinOutput[1]=Q ;                           // Quadratuur
  LockinOutput[2]=sqrt(I*I+Q*Q) ;               // Modulus
  LockinOutput[3]=1.8/pi*atan2(Q,I) ;          // fasehoek in graden -1.8 .. 1.8 Volt
  
} ; // Lockin

// **************************************************************************************
void CORDIC(float Retabel[5],   		// Reele delen van de roterende eenheidseenheidsvektoren
            float Imtabel[5],                   // Imaginaire delen van de roterende eenheidseenheidsvektoren
            float refrotatie[2]){  		// vektorrotatie in radialen per bemonsteringsinterval

// COordinate Rotating DIgital Computer 1959 Jack Volder
// Gegeven tijdstappen 10 microsec is er toch 6 cijfers resolutie in de opgewekte frekwentie mogelijk door gebruik van dit vektorrotatie algoritme.
// De absolute fase van de (co-)sinus wordt niet gebruikt, alleen het inkrement per iteratie slag.

// De berekeningen duren in de Teensy processor 132 nsec 2024-09-26, compiler instelling: fastest en Link Time Optimization

  float a,b,                                    // dummy float variabelen, eigenlijk dus flummy's
        korrektie;                              // T.b.v. modulus==1 stabilisator
  int   harmonische ;                           // Slechts voor lokaal gebruik
  
  // De tussenresultaten bij de generatie van harmonischen hebben elk een eigen naam. Hun naam is hun funktie.
  float cosf0,cos2f0,cos3f0,cos4f0,cos5f0,
        sinf0,sin2f0,sin3f0,sin4f0,sin5f0;
   
   
	// ***** De rotatie-matrix vermenigvuldigd met f0 eenheidsvektor in Retabel[0],Imtabel[0] geeft de nieuwe waarden voor Retabel[0],Imtabel[0*****
  
	a= refrotatie[0]*Retabel[0]+refrotatie[1]*Imtabel[0] ;
	b=-refrotatie[1]*Retabel[0]+refrotatie[0]*Imtabel[0] ;

	// Modulus, nominaal 1.000. Stabilisatie van de modulus van de f0 eenheidsvektor is onmisbaar om afrondingsfoutakkumulatie (scrabble!) tegen te gaan.
	korrektie=(3.-a*a-b*b)/2. ;                 	// korrektie=1.-(1.-modulus*modulus)/2. is 1 min halve fout in modulus
	cosf0=a*korrektie ; sinf0=b*korrektie ;     	// Fase geinkrementeerde vektor met modulus==1
                                 
	Retabel[0]=cosf0 ; 
	Imtabel[0]=sinf0 ;  
	
	
  	// ****** Harmonischen generatie *****
	// Dit gebeurt d.m.v. goniometrische uitdrukkingen om fase stabiliteit tussen harmonische en de f0 referentie te garanderen
	
  	harmonische=1 ;																// f0*(1+harmonische)=2*f0
	// 2*fref
	cos2f0=2.*cosf0*cosf0-1.;           Retabel[harmonische]=cos2f0 ;
	sin2f0=2.*sinf0*cosf0   ;           Imtabel[harmonische]=sin2f0 ;

	harmonische++ ;                               // f0*(1+harmonische)=3*f0
	// 3* fref
	cos3f0=cosf0*cos2f0-sinf0*sin2f0 ;  Retabel[harmonische]=cos3f0 ;
	sin3f0=sinf0*cos2f0+cosf0*sin2f0 ;  Imtabel[harmonische]=sin3f0 ; 

	harmonische++ ;                               // f0*(1+harmonische)=4*f0
	// 4* fref
	cos4f0=2.*cos2f0*cos2f0-1.;         Retabel[harmonische]=cos4f0 ;
	sin4f0=2.*sin2f0*cos2f0 ;           Imtabel[harmonische]=sin4f0 ;

	harmonische++ ;                               // f0*(1+harmonische)=5*f0
	// 5*fref
	cos5f0=cosf0*cos4f0-sinf0*sin4f0 ;  Retabel[harmonische]=cos5f0 ;
	sin5f0=sinf0*cos4f0+cosf0*sin4f0 ;  Imtabel[harmonische]=sin5f0 ;
     
} // CORDIC

// *************************************************************************************

void trigger_adcs()                             // Interrupt routine
{
  adc_dac.start_conversion();
}

void SPI1EventResponder(EventResponderRef event_responder) 
{
  adc_dac.parse_adc_data(ADC);
  adc_dac.check_for_adc_overload(ADC);
}

void SPI0EventResponder(EventResponderRef event_responder) 
{
  digitalWriteFast(_DAC_SYNC, HIGH);
}

// **************************************************************************************

void setup()
{
  int   k ;
  float flummy ;                                // flummy is een float dummy, bummy is een Boolean dummy enz

  Serial.begin(140000);                         // Seriele kommunikatie 140000 Baud prepareren.
  delay(500) ;               // Er blijkt enige wachttijd noodzakelijk. Het waarom is onbekend bij mij.
  flummy=2.*pi*f0/fs ;				// Het fase-inkrement voor het CORDIC vektor rotatie algoritme dient in radialen gegeven te worden.
  refrotatie[0]=cos(flummy) ;            	// Deze cosinus en sinus behoeven nu niet bij elke bemonstering opnieuw berekend te worden.
  refrotatie[1]=sin(flummy) ; 
  
  // ***** Initialiseer alle roterende vektoren met reele term=1. en imaginaire term=0. *****
  for (k=0 ; k<=4 ; k++){
    Retabel[k]=1. ; 
    Imtabel[k]=0. ;
  }

  // Rotatie van het detektie assenstelsel en korrektie van InterneLooptijd
  flummy=framehoek*pi/180. ;                    // Van graden naar radialen
  // flummy=flummy+2*pi*f0*InterneLooptijd;   	// Interne looptijd is hiermee gekompenseerd.
  framerotatie[0]=cos(flummy) ;			// De rotatie van de detektie assenstelsel wordt op de I Q uitgangen toegepast. 
  framerotatie[1]=sin(flummy) ;                                

  // De filter tijdkonstante van de lock-in.
  // ***** De konstanten k1 en k2 moeten van het type double zijn omdat de verhouding    *****
  // ***** fs/f0 wel tot 1e5 kan oplopen en floats hebben maar 6 cijfers in hun mantisse *****
  // tijdkonstante van een enkelvoudig digitaal RC filter tau=1/(fs*k1)
  // Handiger is de specifikatie van de ruisbandbreedte. Dat is van nut bij Signaal/Ruis verhouding (SNR) bepalingen.
  
  k1=double(ruisbandbreedte*2./(pi*fs)) ;       // enkel RC filter ruisbandbreedte=pi/(2*tau)=fs*k1*pi/2
  if (nRC==2) {k1=k1*2. ;}                      // bij dubbel RC filter verdubbelt de waarde van k1 
  k2=1.-k1 ; 
  teller=presetteller;

  adc_dac.init();
  sample_timer.begin(trigger_adcs, SAMPLE_INTERVAL);  // ***** Start de timer, elke 10 microsec een interrupt *****

  g_SPI0Event.attachImmediate(&SPI0EventResponder);
  g_SPI1Event.attachImmediate(&SPI1EventResponder);

}

float i2u(int32_t i) {
  return -float(i)/8388608*11.7/1.2 ;  // 1.2 schalingsfout door filter en ingangsversterker??
}

int32_t u2i(float u) {
  if (u>10.){u=10.;} else if(u<-10.){u=-10.;} // Overrun bescherming
  return int(u/11.7*8388608*1.2);   // Van float schalen en dan naar integer 
}

// ********************************************************************************************
void loop()

 
{
  float U ;

  //if (adc_dac.adc_conversion())                 // Wacht tot adc_conversion==true 
  {
    while (!adc_dac.adc_busy()); {              // Haal alle ADC metingen binnen
      adc_dac.get_adcs(); }                  // resultaten in integer array g_adc_results[]

    //enter your code here
        
    U=i2u(ADC[0]);

    input=U;                                    // Input signaal
      
    Lockin(Retabel,Imtabel,harmonische,framerotatie,LockinOutput,input) ;  // 510 nsec
      
    // Laat de CORDIC oscillator de fase-rotatie op de vektor in Retabel[0] en Imtabel[0] toepassen
    // Tevens worden de harmonischen 2*f0,3*f0,4*f0 en 5*f0 berekend.
    CORDIC(Retabel,Imtabel,refrotatie) ;      	// 132 nsec 2024-9-26       
      

    excitatie=Retabel[0] ;			// Gebruik cos(f0) voor excitatie
    if (CosOfBlok==1){				// Blokgolf generatie
        if (excitatie>=0.) {excitatie=+1.;} 
        else               {excitatie=-1.;}     // Blokgolf, signum operator in C
     }
           
    excitatie=refampl*excitatie+refoffset ;   	// Vervul gebruikerswensen
    
    // Breng de diverse signalen naar hun DACs
    U=excitatie ; 				// f0
    DAC[0]=u2i(U);
    
    U=Retabel[1] ; 				// 2*f0
    DAC[1]=u2i(U);

    // TODO ???
    // U=50; //LockinOutput[3];  			// Fase in graden  -1.8 .. 1.8 Volt
    // if (U>10.){U=10.;} else if(U<-10.){U=-10.;} // Overrun bescherming
    // DAC[2]=int(-U*8388608 );
    
    U= LockinOutput[2]*10; 			// Modulus
    DAC[1]=u2i(U);
    
    teller=teller-1;
    if (teller<=0){ teller=presetteller;

      Serial.print("Modulus=") ;Serial.print(LockinOutput[2]*10) ; Serial.print("   Fase="); Serial.println(LockinOutput[3]*100);
    }


    adc_dac.put_dacs(DAC);                      // Bij eerst volgende bemonstering aan de DACs toegevoerd
  } // if (adc_dac.adc_conversion()) 
} // loop
