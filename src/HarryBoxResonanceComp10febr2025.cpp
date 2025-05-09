/* HarryBoxResonanceComp10febr2025

  ADCDAC 
  (C) Copyright 2023-2024 ELD/LION, Leiden University

  Written by       : Harry Visser & Arno van Amersfoort
  Target compiler  : Arduino IDE
  Target platform  : Teensy 4.1
  Dependencies     : spi.h
  Initial date     : June 20, 2024
  Last modified    : June 28, 2024
  Per 2025-02-03 Resonantie kompensatie werkt. Kier Heeck
  2025-02-10     Kompensator routine kompakter geschreven
  
  ***** In setup ***** 
  regel 290 kan het type kompensatoren gekozen worden
  regel 297 t/m 303 specifikatie van resonantie frekwenties en Q-faktoren
  De gespecificeerde resonanties bestaan in mijn hardware demo
  .
*/

#include <Arduino.h>
#include <IntervalTimer.h>
#include <adc_dac_io.h>

EventResponder g_SPI0Event;
EventResponder g_SPI1Event;
ADCDAC adc_dac(g_SPI0Event, g_SPI1Event);
IntervalTimer sample_timer;
const float   fs=1e5 ;					                // De 100 kHz bemonsteringsfrekwentie zoals gebruikelijk is in de HarryBox.

int32_t ADC[ADC_COUNT];                   			// Array van gemeten integer waarden ADC[0] ... ADC[3] 
int32_t DAC[DAC_COUNT];													// Array met integer outputwaarden   DAC[0] ... DAC[3]

// ******************************************************************************************************************************************************
struct ARMA {                                   // Dit datatytpe omvat 1 bikwadatische breuk in z. Maximaal 1 komplex poolpaar + 1 komplex nulpunten paar
	struct ARMA *nextarmablok ;              			// Linked List van struct ARMA, elke struct is een bikwadratische filtersektie.
	double *koeffeindptr ;                   			// Pointer die op het adres van de laatste koefficient van de koefficientvektor staat en afteld naar 0de koefficient
	double *x0ptr ;                          			// Pointer op meest recente input in de toestandsvektor.
	double *y1ptr ;                          			// Pointer op meest recente output van dit ARMA blok in de toestandsvektor.
	double toestand[5] ;                     			// Toestandsvektor, 5 elementen in een cirkulair buffer met als pointers x0ptr voor input en y1ptr voor vorige filteroutput.   
	double koeff[5] ;                        			// Koefficientvektor, 5 elementen in een vektor.
} ;
typedef struct ARMA ARMABLOK;										// Het blijkt handig om de voorgaande struct met 1 bikwadratische breuk als een blok te zien
  
struct ARMAfilter {                  						// De gehele keten van de 2de orde ARMA sekties.
 	double H0 ;                             			// Schaalfaktor van het gehele ARMAfilter, dat zijn alle struct ARMA tesamen.
  ARMA   armablok ;                         		// Eerste en altijd aanwezige ARMAblok in de linked list van ARMAblokken.
}  ;
typedef struct ARMAfilter *ARMAFILTER ;					// Een ARMA filter wordt in het volgende programma steeds als gedefinieerd als b.v.   ARMAFILTER mijnARMAfilter 

// ***** De digitale filters van dienst, ze kunnen tot een enkele gekombineerd worden *****
ARMAFILTER  ResComp1,ResComp2,ResComp3,ResComp4,ResComp5,ResComp6,ResComp7,ResComp8 ; 

typedef unsigned int CARDINAL ;									// Geen verwarring meer over wat voor inter type bedoeld wordt

// *************************************************************************
void Create(ARMAFILTER &armafilter){
	armafilter=NULL ;
}
// *************************************************************************
void Dispose(ARMAFILTER &armafilter){
	free(armafilter) ;
}
// *************************************************************************
void Allokeer(	ARMAFILTER &armafilter,
								CARDINAL   nARMA){
		
	// Doet allokatie en initialiseert alle pointers, zodat die naar de juiste naamloze variablen wijzen

	CARDINAL allokatie,k ;														// t.b.v geheugen allokatie en als index
	ARMABLOK *armaptr;																// Pointer naar het ARMAblok van dienst
	
  // ***** Te allokeren: ARMAFILTER +(nARMA-1)*ARMABLOK *****
  allokatie=sizeof(struct ARMAfilter)+(nARMA-1)*(sizeof(ARMABLOK)) ;  	// nARMA-1, omdat eerste blok al in het ARMAFILTER struct opgenomen is
  armafilter=(ARMAFILTER)malloc(allokatie) ; 				// Allokeer geheugen voor het gehele digitale filter
  
  armafilter->H0=1.;																// De waarde van de schaalfaktor H0 wordt dient door gebruiker de gewenste waarde gegeven worden
	//   ****** armafilter->nARMAblok=nARMA;					// Gebruik en nut van nARMAblok is nog niet helder 2024-06-26, leek handig om snel te zien hoe groot het filter is.
	armaptr=&(armafilter->armablok) ;  								// Wijst naar eerste ARMAblok in de allokatie
  
	for (k=1 ; k<=nARMA ; k=k+1) {                  	// Initialiseer de pointers in elk ARMA blok in de linked list
  	armaptr->koeffeindptr=&(armaptr->koeff[4]);			// Dit het stopadres bij de vektorinprodukt berekening
		armaptr->x0ptr       =&(armaptr->toestand[0]) ;	// Positie waar het eerste monster, x(0), in geplaatst zal worden
		armaptr->y1ptr       =&(armaptr->toestand[2]) ;	// Positie waar de uitkomst y(0) van de eerste konvolutie geplaatst zal worden,
																										// maar tot dat moment nog steeds x(2) bevat. Na die plaatsing het y(0) nu y(1)
		armaptr->toestand[0]=0. ;                  			// Wis de toestandsvektor, op de domste manier
		armaptr->toestand[1]=0. ;
		armaptr->toestand[2]=0. ;
		armaptr->toestand[3]=0. ;
		armaptr->toestand[4]=0. ;
    if (k<nARMA){
			armaptr->nextarmablok=armaptr+1; 					// Enkelvoudige Linked List van ARAMAblokken, wijst volgende ARAMblok aan.
			armaptr=armaptr->nextarmablok ;						// Ga naar volgende ARMAblok
  	}
  	else {
    	armaptr->nextarmablok=NULL ;              // Einde van de reeks ARMAblokken markeren
  	}
	} 
} // AllokeerARMAFilter
// *************************************************************************
void  Exerceer( ARMAFILTER armafilter,
								float      datumin, 
								float      &datumuit) {

	// Input datum altijd in som plaatsen. Dat is namelijk ook de methode van datum doorgifte tussen de ARMA blokken onderling.

	double som,*koefptr ;
	ARMABLOK *armaptr;
	
 	som=double(datumin) ;												  // Konversie van float naar double
	armaptr=&(armafilter->armablok) ;             // Pointer op eerste ARMAblok

	while (true) {  															// Volg de enkelvoudig linked list langs alle ARMAblokken in armafilter
    
		koefptr=&(armaptr->koeff[0]) ;              // Koefficientpointer op onder handen zijnde ARMA koefficienten
		*(armaptr->x0ptr)=som ;                     // Input opnemen in toestandsvektor van het onderhanden zijnde ARMA blok
  
		// ***** Berekening van het vektor inwendig-produkt van koefficienten-vektor en toestandsvektor *****
																								// De toestandsvektor bevind zich in het cirkulaire buffer .
		som=0. ;																		// Totalisator voor het vektor inwendig produkt
		while (true)  {  
      
			som=som+*koefptr * *(armaptr->x0ptr)  ;   // Het inwendig vektor produkt wordt hier opgebouwd
			if (koefptr==(armaptr->koeffeindptr)) {
			  break;																	// ***** De 5 koeffienten hebben hun rol vervult *****
			}
			// ***** Cirkulair buffer zaken voor x0ptr inkrement *****
			armaptr->x0ptr=armaptr->x0ptr+1;      		// Inkrementeer x0ptr
			if (armaptr->x0ptr==armaptr->koeff)  { 		// In x0ptr buiten het blok toestand?
				armaptr->x0ptr=armaptr->toestand ;			// Zet dan x0ptr weer aan het begin
			}
			koefptr=koefptr+1 ;                   		// Volgende koefficient   sizeof(double)
		} // while , 1 enkel inprodukt is nu berekend
																								// ***** som= vektor inwendigprodukt en is nu berekend. *****
																								
		*(armaptr->y1ptr)=som ;                 		// som in toestandsvektor van huidige ARMAblok opnemen. 
																								// Impliciet transport van som naar volgende ARMAblok

		// ***** Cirkulair buffer zaken voor y1ptr dekrement *****
		armaptr->y1ptr=armaptr->y1ptr-1 ; 					// Dekrementeer y1ptr
		if (armaptr->y1ptr<armaptr->toestand) {			// Is y1ptr buiten het blok toestand geraakt?
			armaptr->y1ptr=armaptr->koeff-1 ;					// Zet dan y1ptr op het einde van toestand
		}
		if ((armaptr->nextarmablok==NULL)) {
		  break ;   																// ***** Alle ARMAblokken zijn verwerkt *****
		}  
		armaptr=armaptr->nextarmablok ;             // Volgend ARMA blok onder handen nemen   
	} // while(true)  
  som=som*armafilter->H0 ;                 			// Schaling, indien nodig
  // ***** Einde berekeningen voor het digitale filter *****

	datumuit=float(som) ;													// Konversie van double naar float

}
// *************************************************************************
void Kompensator(ARMAFILTER &armafilter,				// Gelegenheidsaanvulling ter demo van digitale filters
								 int        keuze,
								 float      fs ,        				// Moet in alle gevallen bekend zijn
								 float      f0 ,
								 float      Qfaktor){ ; 
                       

// Hier moet mijn varargin probleem nog opgelost worden, Alleen keuze 3 funktioneert
// varargin parameters f1 en f2
// Deze demo geeft de keuze: keuze=0 : LowPass    f1=>f0
//                           keuze=1 : HighPass   f1<=f0  
//                           keuze=2 : BandPass   f1<=f0 ; f2>=f0  
//                           keuze=3 : AllPass    -  

// uitbreiding van de parameterlijst van de procedure Kompensator met f1 en f2 is noodzakelijk
// Methode van Matlab varargin   gebruiken?? pointer naar een array van pointers waarin de eerste een pointer naar een integer is, nargin of nargout 
// Matlab boekhoudt de datatype apart van de pointers. In de boekhouding van variabele namen wordt aan de naam het type gekoppeld, dimensies e.d.

// ***** All internal variables in the ARMA fliter are of the type "double" in order *****
// ***** to allow for a large ratio between signal frequency and sampling frequency. *****

// ***** In deze routine wordt het ARMA filter gekreeerd *****

 const double	pi = 3.141592653589793 ;					// It appears in each case
 
 float 			  f1, f2  ;
 const double Qcomp =double(1.)/double(sqrt(2.)) ;
 double 			b, c, p,m,q, r,Q;
 ARMABLOK     *armaptr ;												// Wijst naar het feitelijk ARMA filter, de struct ARMA. 
 																								// Daar wonen de koefficienten, toestandsvektor en de interne pointers
 																								// In deze Kompensator routine worden de ARMA koefficienten berekend en in het ARMA filter geplaatst.
 																								
 CARDINAL     nARMAblok;												// Een miniem restje MODULA-2 
 
 f0=fs/pi*tan(pi*f0/fs) ;   										// prewarp, om met name de frekwenties richting 30 kHz een korrektie te geven.
 
 
 // *****  Allokatie van het gekozen tytpe ARMA filter ******
 nARMAblok=1 ;
 switch (keuze){
 	case 2:
 		nARMAblok=2 ;																	// Het bandfilter heeft nog een tweede bi-kwadratisch blok
 	break ;
 }
 
 Allokeer(armafilter,nARMAblok) ;
 armaptr=&(armafilter->armablok);  								// Wijst eerste van de linked list van ARMA blokken aan
 
 // ***** Berekening van de hulpvariabelen *****
 Q=double(Qfaktor) ;
 p=double(fs)/(pi*double(f0)) ;
			
	switch(keuze){ 
		case 0:																				// ****** Low Pass voorlopig f1=f0 ****** 
				f1= f0;																		// In reality is f1=>f0 allowed
				b = double(fs)/(pi*double(f1));
				q = double(1.)+b/Qcomp+b*b ;	
		break;

		case 1:																				// ***** High Pass voorlopig f1=f0 *****	    
				f1=f0 ;																		// In reality is f1=<f0 allowed
				b=double(fs)/(pi*double(f1));
				m=double(f0/f1) ; m=m*m ;
				q=double(1.)+b/Qcomp+b*b ;
		break ;

		case 2:																				// ***** Band Pass voorlopig f1=f0 en f2=f0 *****     
			 f1=f0 ;																		// In reality is f1=<f0 allowed, bandwith extension
			 f2=f0 ;																		// In reality is f2=>f0 allowed, bandwith extension
			 b=double(fs)/(pi*double(f1));
			 c=double(fs)/(pi*double(f2));
			 m=double(f2*f0/f1)/(pi*double(f1)) ;
			 q=double(1.)+b/Qcomp+b*b ;
			 r=double(1.)+c/Qcomp+c*c ;
		break ;

		case 3:																				// ***** All Pass *****
			q=double(1.)+double(p)+p*p ;								// De nieuwe noemer, geen Qcomp
		break ;
	} // switch(keuze)
 
  // ***** Substitueer de koefficienten in het ARMA filter *****
	switch (keuze){
		case 0:
		case 1:
		case 2:
			armaptr->koeff[0] = (double(1.)+p/Q+p*p)        /q ;
			armaptr->koeff[1] = (double(2.)-double(2.)*p*p) /q ;
			armaptr->koeff[2] = (double(1.)-p/Q+p*p)        /q ;
			armaptr->koeff[3] =-(double(2.)-double(2.)*b*b) /q ;
			armaptr->koeff[4] =-(double(1.)-b/Qcomp+b*b)    /q ;
		break;
    case 3:
    	armaptr->koeff[0]= (double(1.)+p/Q+p*p)        /q ;
			armaptr->koeff[1]= (double(2.)-double(2.)*p*p) /q ;
		  armaptr->koeff[2]= (double(1.)-p/Q+p*p)        /q ;
		  armaptr->koeff[3]=-(double(2.)-double(2.)*p*p) /q ;
    	armaptr->koeff[4]=-(double(1.)-p+p*p)          /q ;  // Please remark, no Qcomp in this expression. It is invisible, it's value is 1!
    	break;
  }
 	switch (keuze){
 		case 2:																				// Het bandfilter heeft nog een tweede bi-kwadratisch blok
					armaptr=armaptr->nextarmablok;				  // Het volgende ARMAblok binnen het filter gaat nu gevuld worden
				    
				  armaptr->koeff[0]= m/r;
				  armaptr->koeff[1]= double(0.) ;
			    armaptr->koeff[2]= -m/r ;
		     	armaptr->koeff[3]=-(double(2.)-double(2.)*c*c)/r ;
		     	armaptr->koeff[4]=-(double(1.)-c/Qcomp+c*c)   /r ;
    break;
 	}
  armafilter->H0 = double(1.) ; 								// In elk geval van deze demos is de schaalfaktor==1.

}  // Kompensator
// *******************************************************************************************************************************************************
void trigger_adcs()                             // ***** Interrupt routine *****
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
// *******************************************************************************************************************************************************
void setup()
{
  int keuze ;
  
  Serial.begin(140000) ;												// De serieele snelheid in Bit/sec
  delay(500) ;																	// Er blijkt enige wachttijd noodzakelijk. Het waarom is onbekend bij mij.
  // Serial.print() en Serial.println()  werden bij proeven gebruikt

   // ***** Keuze type kompensator  *****
   
   // ***** keuze=0    LowPass ; 		*****
   // ***** keuze=1    HighPass ; 	*****
   // ***** keuze=2    BandPass; 		*****
   // ***** keuze=3    AllPass; 		*****
   
  keuze=3 ; // het aanbevolen 'altijd goed' model

  // Parameters van de Kompensator: (ARMAFILTER,keuze, fs,f0,Q)
  // Allokatie van de ARMAfilters wordt binnen Kompensatie aangeroepen
  
  // ***** De te kompenseren resonanties en hun Q-faktoren *****

  Kompensator(ResComp1,keuze,fs, 2540, 18) ;
  Kompensator(ResComp2,keuze,fs, 3500,  1.8) ; // Geeft grote blop een vlakke top
  Kompensator(ResComp3,keuze,fs, 4995, 30) ;   // Blijkt temperatuur gevoelig te zijn
  Kompensator(ResComp4,keuze,fs, 7780, 45) ;
  Kompensator(ResComp5,keuze,fs, 8450, 35) ;
  Kompensator(ResComp6,keuze,fs,12100, 65) ;    
  Kompensator(ResComp7,keuze,fs,15500, 60) ;
  
  adc_dac.init();																// Wat zou dat nou doen?
  sample_timer.begin(trigger_adcs, SAMPLE_INTERVAL);  // Start de timer, elke 10 microsec een interrupt
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

// *****************************************************************************
void loop()																			// Het Hoofdprogramma
 
{
	float U; 

//   if (adc_dac.adc_conversion())                 // Wacht tot adc_conversion==true 
//   {
	while (!adc_dac.adc_busy());              // Haal alle ADC metingen binnen en zet DAC data klaar voor volgende interrupt.

	//   adc_dac.get_adcs(ADC); }                  // resultaten in integer array g_adc_results[]
	adc_dac.get_adcs();

	//enter your code here
		
	// U=-float(ADC[0])/8388608*11.7/1.2 ;         // 1.2 schalingsfout door filter en ingangsversterker??
	U=i2u(ADC[0]);

	// ***** Doe de resonantieonderdrukking *****
	// Een array van kompensatoren lijkt handig
	Exerceer(ResComp1,U,U);        
	Exerceer(ResComp2,U,U); 
	Exerceer(ResComp3,U,U); 
	Exerceer(ResComp4,U,U); 
	Exerceer(ResComp5,U,U); 
	Exerceer(ResComp6,U,U);
	Exerceer(ResComp7,U,U);
		
	U=50.*U;																		// Zonder deze vermenigvuldiging is de output onhandig klein (voor de skoop)

	// end of code

	// if (U>10.){U=10.;} else if(U<-10.){U=-10.;} // Overrun bescherming
	// DAC[0]=int(-U/11.7*8388608 );
	DAC[0]=u2i(U);

	adc_dac.put_dacs(DAC);                      // Wordt bij eerst volgende bemonstering aan de DACs toegevoerd
//    } // if (adc_dac.adc_conversion()) 
} // loop
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
