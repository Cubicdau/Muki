#include <Arduino.h>
#include <SPI.h>
#include <utility/VS10xx.h>

/**********************************************************
 * private Definitionen
 **********************************************************/


/**********************************************************
 * Konstruktor
 **********************************************************/
VS10XX::VS10XX()
{
}

/**********************************************************
 * öffentliche Methoden
 **********************************************************/
/*---------------------------------------------------------
 Name:           begin
 
 Beschreibung:   initialisiert den MP3-Decoder            
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
void VS10XX::begin(void)
{
  pinMode(VS10XX_DREQ, INPUT);
  digitalWrite(VS10XX_DREQ, HIGH);
  pinMode(VS10XX_RST, OUTPUT);
  pinMode(VS10XX_DCS,OUTPUT );
  pinMode(VS10XX_CS, OUTPUT);
  pinMode(VS10XX_MUTE, INPUT);
  digitalWrite( VS10XX_MUTE, LOW );

  SPI.begin();

  //INIT VS1011
  Reset();
}
/*---------------------------------------------------------
 Name:           PutInReset
 
 Beschreibung:   versetzt den MP3-Decoder in Reset
 				setzt Reset-Pin auf Low
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
void VS10XX::PutInReset(void)
{
  digitalWrite( VS10XX_RST, LOW );
}
/*---------------------------------------------------------
 Name:           ReleaseFromReset
 
 Beschreibung:   entlässt den MP3-Decoder aus dem Reset
 				setzt Reset-Pin auf High
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/** Release the MP3 player chip from reset */
void VS10XX::ReleaseFromReset(void)
{
  digitalWrite( VS10XX_RST, HIGH );
}
/*---------------------------------------------------------
 Name:           SelectControl
 
 Beschreibung:   wählt den MP3-Decoder im Befehlsmodus an
 				setzt Control Chip-Select auf Low
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/** Control Chip Select (for accessing SPI Control/Status registers) */
/** Pull the VS10xx Control Chip Select line Low */
void VS10XX::SelectControl(void)
{
  digitalWrite( VS10XX_CS, LOW );
}
/*---------------------------------------------------------
 Name:           DeselectControl
 
 Beschreibung:   wählt den Befehlsmodus des MP3-Decoder ab
 				setzt Control Chip-Select auf High
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/** Pull the VS10xx Control Chip Select line High */
void VS10XX::DeselectControl(void)
{
  digitalWrite( VS10XX_CS, HIGH );
}
/*---------------------------------------------------------
 Name:           SelectData
 
 Beschreibung:   wählt den MP3-Decoder im Datenmodus an
 				setzt Control Chip-Select auf Low
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/** Pull the VS10xx Data Chip Select line Low */
void VS10XX::SelectData(void)
{
  digitalWrite( VS10XX_DCS, LOW );
}
/*---------------------------------------------------------
 Name:           DeselectData
 
 Beschreibung:   wählt den Datenmodus des MP3-Decoder ab
 				setzt Control Chip-Select auf High
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/** Pull the VS10xx Data Chip Select line High */
void VS10XX::DeselectData(void)
{
  digitalWrite( VS10XX_DCS, HIGH );
}
/*---------------------------------------------------------
 Name:           Check_DREQ
 
 Beschreibung:   prüft ob MP3-Decoder bereit ist neue Daten zu Empfangen,
 				ließt DREG-Pin ein.
 
 Eingänge:       void      
 
 Ausgang:        unsigned char
 					Status des DREQ-Pins
 ---------------------------------------------------------*/
/** Data Request: Player asks for more data */
unsigned char VS10XX::Check_DREQ(void)
{
  return digitalRead( VS10XX_DREQ );
}
/*---------------------------------------------------------
 Name:           SetMute
 
 Beschreibung:   schaltet den Ausgang des MP3-Decoders auf Stumm,
 				setzt MUTE-Pin auf Low
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/** Mute the MP3 player chip */
void VS10XX::SetMute(void)
{
  pinMode(VS10XX_MUTE, OUTPUT);
  digitalWrite( VS10XX_MUTE, LOW );
}
/*---------------------------------------------------------
 Name:           UnsetMute
 
 Beschreibung:   beendet die Stummschaltung des MP3-Decoders,
 				setzt MUTE-Pin auf High
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/** Release the MP3 player chip from reset */
void VS10XX::UnsetMute(void)
{
  pinMode(VS10XX_MUTE, OUTPUT);
  digitalWrite( VS10XX_MUTE, HIGH );
}
/*---------------------------------------------------------
 Name:           Reset
 
 Beschreibung:   versetzt den MP3-Decoder hardwaremaßit in Reset
 				und initialisiert diesen anschliessend neu.
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/**
 * \brief Reset VS10xx
 **/
void VS10XX::Reset()
{
  SPI.setClockDivider( SPI_CLOCK_DIV16 );
  PutInReset();
  delay( 1 );

  /* Send dummy SPI byte to initialize SPI */
  SPI.transfer( 0xFF );

  /* Un-reset MP3 chip */
  DeselectControl();
  DeselectData();
  ReleaseFromReset();
  SetVolume( 0xff, 0xff ); //OFF

  /* Set clock register, doubler etc. */
  WriteRegister( VS10XX_SPI_CLOCKF, 0x30, 0xD4 ); // 25Mhz

  /* Wait for DREQ */
  while( !( Check_DREQ() ) )
  {
    ; /* Do nothing while waiting for DREQ = 1 */
  }

  /* Slow sample rate for slow analog part startup */
  WriteRegister( VS10XX_SPI_AUDATA, 0, 10 ); /* 10 Hz */
  delay( 100 );

  /* Switch on the analog parts */
  SetVolume( 0xfe, 0xfe );
  WriteRegister( VS10XX_SPI_AUDATA, 31, 64 ); /* 8kHz */
  SetVolume( 0,0 ); //Max
  SoftReset();

  SPI.setClockDivider( SPI_CLOCK_DIV4 );
}
/*---------------------------------------------------------
 Name:           SoftReset
 
 Beschreibung:   löst einen Reset des MP3-Decoders in Software aus.
 				Zwischen einzelnen Sounddateinen ausführen.
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/**
 * \brief Soft Reset of VS10xx (Between songs)
 **/
void VS10XX::SoftReset()
{
  /* Soft Reset of VS10xx */
  WriteRegister( VS10XX_SPI_MODE, 0x08, 0x04 ); /* Newmode, Reset, No L1-2 */

  delay( 10 ); /* One millisecond delay */
  while( !( Check_DREQ() ) ) /* wait for startup */
  {
    ; /* Do nothing while waiting for DREQ = 1 */
  }

  /* Set clock register, doubler etc. */
  /* (XTALI/2000) in Hz (+0x8000 if clockdoubling) */
  WriteRegister( VS10XX_SPI_CLOCKF, 0x30, 0xD4 ); // 25MHz

  delay( 10 ); /* One millisecond delay */

  /* Send null bytes to data interface */
  SelectData();
  SPI.transfer( 0 );
  SPI.transfer( 0 );
  SPI.transfer( 0 );
  SPI.transfer( 0 );
  DeselectData();
}

/*---------------------------------------------------------
 Name:           Send
 
 Beschreibung:   sendet ein Datenbyte an den MP3-Decoder
 
 Eingänge:       unsigned char databyte
 					Datenbyte welches gesendet werden soll
 
 Ausgang:        void
 ---------------------------------------------------------*/
/**
 * \brief Sends one Byte to VS10xx
 *
 * \param databyte to be send
 **/
void VS10XX::Send( unsigned char databyte )
{
  SelectData(); /* Select Data-transfer */
  SPI.transfer( databyte );
  DeselectData(); /* Deselect Data-transfer */
}
/*---------------------------------------------------------
 Name:           Send2048Zeros
 
 Beschreibung:   sendet 2048 NULLEN zum MP3-Decoder,
 				um internen Puffer zu leeren
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/**
 * \brief Sends 2048 Zeros to VS10xx
 * to shift out all data in buffer
 **/
void VS10XX::Send2048Zeros()
{
  SelectData(); /* Select Data-transfer */
  for( unsigned int i = 0; i < 2048; i++)
  {
    while( !( Check_DREQ() ) )      /* Wait for DREQ = 1 */
    {
      ;
    }
    SPI.transfer( 0x00 );
  }
  DeselectData(); /* Deselect Data-transfer */
}
/*---------------------------------------------------------
 Name:           Send32
 
 Beschreibung:   sendet einen Block aus 32 Datenbytes zum MP3-Decoder.
 				Decoder erwartet Blöcke aus je 32 Byte
 
 Eingänge:       unsigned char* pBuffer
 					Pointer auf den Datenpuffer zum Senden
 
 Ausgang:        unsigned char*
 					Pointer auf den Datenpuffer nach dem Senden
 ---------------------------------------------------------*/
/**
 * \brief Sends 32Byte-Block to VS10xx
 *
 * \param pBuffer <pointer to databuffer>
 *
 * \return new Pointerposition of buffer
 **/
unsigned char* VS10XX::Send32( unsigned char* pBuffer )
{

  SelectData(); /* Select Data-transfer */
  while( !( Check_DREQ() ) )      /* Wait for DREQ = 1 */
  {
    VS10XX_FREETIME;
  }

  /* Send 32 octets of disk block data to VS10xx */
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  SPI.transfer( *pBuffer++ );
  DeselectData(); /* Deselect Data-transfer */
  return pBuffer;
}
/*---------------------------------------------------------
 Name:           WriteRegister
 
 Beschreibung:   schreiben eines Registers im MP3-Decoder
 				ein Register ist 16-Bit breit
 
 Eingänge:       unsigned char addressbyte
 					Adresse des Registers
 				unsigned char  highbyte
 					erstes Byte des Registerinhalts
 				unsigned char  lowbyte
 					zweites Byte des Registerinhalts
 
 Ausgang:        void
 ---------------------------------------------------------*/
/**
 * \brief Write VS10xx register
 *
 * \param addressbyte <register address>
 * \param highbyte <highbyte of 16bit data>
 * \param lowbyte <lowbyte of 16bit data>
 **/
void VS10XX::WriteRegister( unsigned char  addressbyte, unsigned char  highbyte, unsigned char  lowbyte )
{ 
  SelectControl(); 
  SPI.transfer( VS10XX_WRITE_COMMAND );
  SPI.transfer( addressbyte );
  SPI.transfer( highbyte );
  SPI.transfer( lowbyte );
  DeselectControl();
} 
/*---------------------------------------------------------
 Name:           ReadRegister
 
 Beschreibung:   lesen eines Registers im MP3-Decoder
 				ein Register ist 16-Bit breit
 
 Eingänge:       unsigned char addressbyte
 					Adresse des Registers
 
 Ausgang:        unsigned int
 					gelesener Registerinhalt
 ---------------------------------------------------------*/
/**
 * \brief Read the 16-bit value of a VS10xx register
 *
 * \param addressbyte <register address>
 *
 * \return Received data from register
 **/
unsigned int VS10XX::ReadRegister( unsigned char addressbyte )
{
  unsigned int resultvalue = 0;
  SelectControl();
  SPI.transfer( VS10XX_READ_COMMAND );
  SPI.transfer( addressbyte );
  //SPI.transfer( 0xff );
  resultvalue = SPI.transfer( 0x00 );
  //SPI.transfer( 0xff );
  resultvalue |= SPI.transfer( 0x00 );
  DeselectControl();
  return resultvalue;
}
/*---------------------------------------------------------
 Name:           SetVolume
 
 Beschreibung:   Lautstärkeeinstellung des MP3-Decoders setzen
 				0 = lauteste Einstellung
 				254 = leiseste Einstellung
 				255 = deaktivieren der Ausgangsstufe des MP3-Decoders
 
 Eingänge:       unsigned char  leftchannel
 					Lautstärke für den linken Ausgangskanal
 				unsigned char  rightchannel 
 					Lautstärke für den rechten Ausgangskanal
 
 Ausgang:        void
 ---------------------------------------------------------*/
/**
 * \brief Set VS10xx Volume Register
 *
 * \param leftchannel <value for left volume>
 * \param rightchannel <value for right volume>
 **/
void VS10XX::SetVolume( unsigned char  leftchannel, unsigned char  rightchannel )
{
  WriteRegister( VS10XX_SPI_VOL, leftchannel, rightchannel );
}
/*---------------------------------------------------------
 Name:           SineTest
 
 Beschreibung:   Ausgabe eines Sinus-Testtons starten.
 				Ausgabe stoppen mit Reset/Softreset des MP3-Decoders.
 
 Eingänge:       void      
 
 Ausgang:        void
 ---------------------------------------------------------*/
/**
 * \brief VS10xx Sine Test Function
 *
 * Good getting started example
 **/
void VS10XX::SineTest()
{

  /* Reset MP3 chip */
  PutInReset();       /* Pull xRESET low -> hardware reset */
  delay( 100 );            /* 100 ms delay */

  /* Send dummy SPI byte to initialize SPI of Atmel microcontroller */
  SPI.transfer( 0xFF );

  /* Un-reset MP3 chip */
  DeselectControl();  /* Pull xCS high    */
  DeselectData();     /* Pull xDCS high   */
  ReleaseFromReset(); /* Pull xRESET high */
  delay( 100 );            /* 100 ms delay     */


  /* VS10xx Application Notes, chapter 4.8 ---------------------------------*/
  /* As an example, let's write value 0x0820 to register 00 byte by byte    */
  SelectControl();    /* Now SPI writes go to SCI port                   */
  SPI.transfer( 0x02 );      /* Send SPI Byte, then wait for byte to be sent.   */
  SPI.transfer( 0x00 );      /* 0x02 was WRITE command, 0x00 is register number */
  SPI.transfer( 0x08 );      /* This byte goes to MSB                           */
  SPI.transfer( 0x20 );      /* ..and this is LSB. (0x20=Allow Test Mode)       */
  DeselectControl();  /* Now SPI writes don't go to SCI port             */

  while( !( Check_DREQ() ) )      /* Wait for DREQ = 1                               */
  {
    ; 			 /* Do nothing while waiting for DREQ = 1           */
  }

  /* Send a Sine Test Header to Data port                                   */
  SelectData();       /* Now SPI writes go to SDI port                   */

  SPI.transfer( 0x53 );      /* - This is a special VLSI Solution test header - */
  SPI.transfer( 0xef );      /* - that starts a sine sound. It's good for     - */
  SPI.transfer( 0x6e );      /* - testing your code, the chip and also for    - */
  SPI.transfer( 0xA8 );      /* - seeing if your MP3 decoder was manufactured - */
  SPI.transfer( 0x00 );      /* - by VLSI Solution oy. ------------------------ */
  SPI.transfer( 0x00 );
  SPI.transfer( 0x00 );
  SPI.transfer( 0x00 );
  DeselectData();

  //  delay( 500 );           /* 500 ms delay */
  //
  //  /* Stop the sine test sound */
  //  SelectData();
  //  SPI.transfer( 0x45 );
  //  SPI.transfer( 0x78 );
  //  SPI.transfer( 0x69 );
  //  SPI.transfer( 0x74 );
  //  SPI.transfer( 0x00 );
  //  SPI.transfer( 0x00 );
  //  SPI.transfer( 0x00 );
  //  SPI.transfer( 0x00 );
  DeselectData();

  delay( 500 );            /* 500 ms delay */
}