#ifndef VS10XX_h
#define VS10XX_h


/**********************************************************
 * Definitionen
 **********************************************************/
/** 
 * Unter dieser Definition kann eine Funktion abgelegt werden,
 * welche w?hrend des wartens auf den MP3-Decoder ausgef?hrt wird.
 **/
#ifndef VS10XX_FREETIME
#define VS10XX_FREETIME
#endif
/******************************************************************************
 * Definiere Port und Pins f?r VS10xx
 ******************************************************************************/
#define VS10XX_DREQ		3
#define VS10XX_RST		8 
#define VS10XX_DCS		7  
#define VS10XX_CS		6
#define VS10XX_MUTE		5



/******************************************************************************
 * VS10xx SCI Command bytes
 ******************************************************************************/
/** VS10xx SCI Write Command byte is 0x02 */
#define VS10XX_WRITE_COMMAND 0x02
/** VS10xx SCI Read Command byte is 0x03 */
#define VS10XX_READ_COMMAND 0x03

/******************************************************************************
 * VS10xx Register
 ******************************************************************************/
#define VS10XX_SPI_MODE          0x00   /**< VS10xx register */
#define VS10XX_SPI_STATUS        0x01   /**< VS10xx register */
#define VS10XX_SPI_BASS          0x02   /**< VS10xx register */
#define VS10XX_SPI_CLOCKF        0x03   /**< VS10xx register */
#define VS10XX_SPI_DECODE_TIME   0x04   /**< VS10xx register */
#define VS10XX_SPI_AUDATA        0x05   /**< VS10xx register */
#define VS10XX_SPI_WRAM          0x06   /**< VS10xx register */
#define VS10XX_SPI_WRAMADDR      0x07   /**< VS10xx register */
#define VS10XX_SPI_HDAT0         0x08   /**< VS10xx register */
#define VS10XX_SPI_HDAT1         0x09   /**< VS10xx register */
#define VS10XX_SPI_AIADDR        0x0a   /**< VS10xx register */
#define VS10XX_SPI_VOL           0x0b   /**< VS10xx register */
#define VS10XX_SPI_AICTRL0       0x0c   /**< VS10xx register */
#define VS10XX_SPI_AICTRL1       0x0d   /**< VS10xx register */
#define VS10XX_SPI_AICTRL2       0x0e   /**< VS10xx register */
#define VS10XX_SPI_AICTRL3       0x0f   /**< VS10xx register */

#define VS10XX_SM_DIFF          0x01    /**< VS10xx register */
#define VS10XX_SM_JUMP          0x02    /**< VS10xx register */
#define VS10XX_SM_RESET         0x04    /**< VS10xx register */
#define VS10XX_SM_OUTOFWAV      0x08    /**< VS10xx register */
#define VS10XX_SM_PDOWN         0x10    /**< VS10xx register */
#define VS10XX_SM_TESTS         0x20    /**< VS10xx register */
#define VS10XX_SM_STREAM        0x40    /**< VS10xx register */
#define VS10XX_SM_PLUSV         0x80    /**< VS10xx register */
#define VS10XX_SM_DACT          0x100   /**< VS10xx register */
#define VS10XX_SM_SDIORD        0x200   /**< VS10xx register */
#define VS10XX_SM_SDISHARE      0x400   /**< VS10xx register */
#define VS10XX_SM_SDINEW        0x800   /**< VS10xx register */
#define VS10XX_SM_ADPCM         0x1000  /**< VS10xx register */
#define VS10XX_SM_ADPCM_HP      0x2000  /**< VS10xx register */

static unsigned int playState;
#define idle		0
#define playback	1
#define pause		2
/**********************************************************
 * Klasse VS10XX
 **********************************************************/
class VS10XX
{
public:
  VS10XX();
  void begin( void );
  void Reset( void );
  void SoftReset( void );

  void UnsetMute( void );
  void SetMute( void );
  void SetVolume( unsigned char leftchannel, unsigned char rightchannel );

  void Send( unsigned char databyte );
  unsigned char* Send32( unsigned char* pBuffer );
  void Send2048Zeros( void );

  void WriteRegister( unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte );
  unsigned int ReadRegister( unsigned char addressbyte );
  void playpause ( void);
  void getState( void );
  void SineTest( void );

private:
  void PutInReset( void );
  void ReleaseFromReset( void );
  void SelectControl( void );
  void DeselectControl( void );
  void SelectData( void );
  void DeselectData( void );
		static void feed();
  unsigned char Check_DREQ( void );
};


extern VS10XX VS1011;

#endif /* VS10XX_h */
