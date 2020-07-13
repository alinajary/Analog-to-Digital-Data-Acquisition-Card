
#include <asf.h>
#include "conf_usb.h"
#include "ui.h"
#include "uart.h"


// ====================== UART Requirement ====================== 
#define USART_SERIAL                     &USARTE0
#define USART_SERIAL_BAUDRATE            115200
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false
// =================== END UART Requirement ======================



// ====================== ADC Requirement ====================== 
#define MY_ADC_A    ADCA
#define MY_ADC_B    ADCB
#define MY_ADC_CH ADC_CH1

#define ADCA_CH0    (1U << 0)       /**< ADC A channel 0. */
#define ADCA_CH1    (1U << 1)       /**< ADC A channel 1. */
#define ADCA_CH2    (1U << 2)       /**< ADC A channel 2. */
#define ADCA_CH3    (1U << 3)       /**< ADC A channel 3. */

#define ADCB_CH0    (1U << 0)       /**< ADC B channel 0. */
#define ADCB_CH1    (1U << 1)       /**< ADC B channel 1. */
#define ADCB_CH2    (1U << 2)       /**< ADC B channel 2. */
#define ADCB_CH3    (1U << 3)       /**< ADC B channel 3. */

void putdecint (unsigned int num)
{
	uint8_t c1= num%10;
	uint8_t c2=(num/10)%10;
	uint8_t c3=(num/100)%10;
	uint8_t c4=(num/1000)%10;
	uint8_t c5=(num/10000)%10;

	udi_cdc_putc(c5 + '0');
	udi_cdc_putc(c4 + '0');
	udi_cdc_putc(c3 + '0');
	udi_cdc_putc(c2 + '0');
	udi_cdc_putc(c1 + '0');
}
/*
static void adc_init(void)
{
	 struct adc_config adc_conf;
	 struct adc_channel_config adcch_conf;

	 adc_read_configuration(&MY_ADC_A, &adc_conf);
	 adcch_read_configuration(&MY_ADC_A, MY_ADC_CH, &adcch_conf);

	 adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,	 ADC_REF_AREFA); // ADC_REF_AREFA
	 adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	 adc_set_clock_rate(&adc_conf, 200000UL);

	 adcch_set_input(&adcch_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1);

	 adc_write_configuration(&MY_ADC_A, &adc_conf);
	 adcch_write_configuration(&MY_ADC_A, MY_ADC_CH, &adcch_conf);
}
*/

static void adc_a_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	
	adc_read_configuration(&MY_ADC_A, &adc_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,	 ADC_REF_AREFA); // ADC_REF_AREFA
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&MY_ADC_A, &adc_conf); 
	
	adcch_read_configuration(&MY_ADC_A, ADCA_CH0, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_A, ADCA_CH0, &adcch_conf);
		
	adcch_read_configuration(&MY_ADC_A, ADCA_CH1, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN2, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_A, ADCA_CH1, &adcch_conf);
	
	adcch_read_configuration(&MY_ADC_A, ADCA_CH2, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN3, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_A, ADCA_CH2, &adcch_conf);
	
	adcch_read_configuration(&MY_ADC_A, ADCA_CH3, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN4, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_A, ADCA_CH3, &adcch_conf);

}


static void adc_b_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	
	adc_read_configuration(&MY_ADC_B, &adc_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,	 ADC_REF_AREFB); // ADC_REF_AREFB
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&MY_ADC_B, &adc_conf);
	
	adcch_read_configuration(&MY_ADC_B, ADCB_CH0, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_B, ADCB_CH0, &adcch_conf);
	
	adcch_read_configuration(&MY_ADC_B, ADCB_CH1, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN4, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_B, ADCB_CH1, &adcch_conf);
	
	adcch_read_configuration(&MY_ADC_B, ADCB_CH2, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN5, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_B, ADCB_CH2, &adcch_conf);
	
	adcch_read_configuration(&MY_ADC_B, ADCB_CH3, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN6, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC_B, ADCB_CH3, &adcch_conf);
}


static inline uint16_t adc_read_cmplt(ADC_t *adc, uint8_t ch_mask)
{
	uint8_t index = 0;

	//irqflags_t flags = cpu_irq_save();
	adc->CTRLA |= ch_mask << ADC_CH0START_bp;
	//cpu_irq_restore(flags);

	while (adc_get_interrupt_flag(adc, ch_mask) != ch_mask);
	adc_clear_interrupt_flag(adc, ch_mask);

	Assert(ch_mask & ((1 << ADC_NR_OF_CHANNELS) - 1));
	if (!(ch_mask & 0x03)) {
		index += 2;
		ch_mask >>= 2;
	}
	if (!(ch_mask & 0x01)) {
		index++;
	}

	return ((ADC_CH_t *)(&adc->CH0 + index))->RES;
}

// ==================== EDN ADC Requirement ====================


// ======================================== DAC Requirement ===============================
#define OUTPUT_DAC    DACB 
#define RATE_OF_CONVERSION    500

__always_inline static void wait_for_timer(void)
{
	#if !XMEGA_E
	do { } while (!(TCC0.INTFLAGS & TC0_OVFIF_bm));
	TCC0.INTFLAGS = TC0_OVFIF_bm;
	#else
	do { } while (!(TCC4.INTFLAGS & TC4_OVFIF_bm));
	TCC4.INTFLAGS = TC4_OVFIF_bm;
	#endif
}
// ===================================== END DAC Requirement =============================


void spi_init_module(void);
void spi_init_pins(void);

static volatile bool main_b_cdc_enable = false;
//static int data_buf_send[64]= {21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84};
static int data_buf_send[64]= {'#','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','1','2','3','4','5','6','7','8','9','0','@'};
//static int data_buf_send[32]= {'#','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','A','B','C','D','E'};


/*! \brief Main function. Execution starts here.
 */
int main(void)
{
//uint8_t value_send ;
int i, j, ii, pack_count;
uint8_t data_buffer[4] = {0xf0, 0xff, 0xaa, 0x00};
//uint8_t data_buffer1 = 0xaa;
uint16_t result;
uint8_t result_lo, result_hi;

unsigned char var_vol;
char xor_output, xor_input;
unsigned int adc_value[8]={0,0,0,0,0,0,0,0};
unsigned int adc_value_a1,adc_value_a2,adc_value_a3,adc_value_a4=0 ;
unsigned int adc_value_b1,adc_value_b2,adc_value_b3,adc_value_b4=0 ;
uint16_t dac_val0, dac_val1 ;
uint8_t 	rx_buf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		// receive data buf
iram_size_t buf_read_size =0;

uint8_t NumByteRec;


//________________added by ghiasi______________
float f_dac_val0, f_dac_val1;
double f_adc_value;

// float gain_dac= 0.9945;  
// float offs_dac= 2;
float gain_dac0= 0.6354;		
float offs_dac0= 4;

float gain_dac1= 0.6311;
float offs_dac1= -3;

float gain  = 1.036;//1.03;	// 1.031 // 1.0438;// .0035; //1.042;
int offset  = 190 ;		//176; //168;      //165;


//float var_gain[6]={1.3646, 1.3487, 1.3236, 1.3018, 1.276, 1.2596};
//float var_offs[6]={1503, 1428, 1325, 1232, 1132, 1061};
//					5		6		7		8		9		10   Volt
float var_gain[6]={1.33,	1.3192,	1.3031,	1.2867,	1.2675,	1.2522};
float var_offs[6]={1297,	1238,	1170,	1097,	1036,	976};


float gain_var;
float offs_var;


float gain_10volt= 1.033 ; //0.9847 ;
int offs_10volt= 177  ;//137; //74.961 ;

float gain_12v  = 1.045; 
int offset_12v  = 197;

float gain_3v  = 1.048;
int offset_3v  = 198;

uint8_t index_bit=0 ;

//_____________________________________________



static usart_rs232_options_t USART_SERIAL_OPTIONS = {
	.baudrate = USART_SERIAL_BAUDRATE,
	.charlength = USART_SERIAL_CHAR_LENGTH,
	.paritytype = USART_SERIAL_PARITY,
	.stopbits = USART_SERIAL_STOP_BIT
};



struct spi_device spi_device_conf = { .id = IOPORT_CREATE_PIN(PORTC, 4) };
pack_count =0 ;


struct dac_config conf;



irq_initialize_vectors();
cpu_irq_enable();
	
sysclk_init(); 

//sysclk_enable_peripheral_clock(USART_SERIAL);
ioport_configure_pin(IOPORT_CREATE_PIN(PORTE, 3), IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);

 // ============================ UART init ============================
 //usart_init_rs232(USART_SERIAL, &USART_SERIAL_OPTIONS);
 //ioport_configure_pin(IOPORT_CREATE_PIN(PORTE, 3), IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
 //ioport_configure_pin(IOPORT_CREATE_PIN(PORTE, 2), IOPORT_DIR_INPUT);
 // ========================== END UART init ==========================
 
//board_init();
//sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_SPI );
	
// Initialize the sleep manager
//sleepmgr_init();


// Start USB stack to authorize VBus monitoring
udc_start();

	
for (i=0;i<1000;i++) for (j=0;j<1000;j++) ;

spi_init_pins();
spi_init_module();

//================================== DAC INITIAL ============================================
// Initialize the dac configuration.
	dac_read_configuration(&OUTPUT_DAC, &conf);

	dac_set_conversion_parameters(&conf, DAC_REFSEL_AREFA_gc, DAC_ADJ_RIGHT);
	dac_set_active_channel(&conf, DAC_CH0 | DAC_CH1, 0);
	dac_set_conversion_trigger(&conf, 0, 0);
#if XMEGA_DAC_VERSION_1
	dac_set_conversion_interval(&conf, 10);
	dac_set_refresh_interval(&conf, 20);
#endif

	dac_write_configuration(&OUTPUT_DAC, &conf);
	dac_enable(&OUTPUT_DAC);

	dac_wait_for_channel_ready(&OUTPUT_DAC, DAC_CH0 | DAC_CH1);
	dac_set_channel_value(&OUTPUT_DAC, DAC_CH0, 0);
	dac_set_channel_value(&OUTPUT_DAC, DAC_CH1, 0);
	dac_wait_for_channel_ready(&OUTPUT_DAC, DAC_CH0 | DAC_CH1);

//#if !XMEGA_E
	// Configure timer/counter to generate events at conversion rate.
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_TC0);
	TCC0.PER = (sysclk_get_per_hz() / RATE_OF_CONVERSION) - 1;

	// Configure event channel 0 to generate events upon T/C overflow.
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_EVSYS);
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc;

	// Start the timer/counter.
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
//================================ END DAC INITIAL ===========================================
	
	
//================================ END ADC INITIAL ===========================================
//adc_init();
adc_a_init();
adc_b_init();
adc_enable(&MY_ADC_A);
adc_enable(&MY_ADC_B);
//================================ END ADC INITIAL ===========================================


     var_vol = 0x04F; // set to 3.3V
     dac_val0 = 1000;
     dac_val1 = 4000;

	  PORTE.OUT= 0x00; 
     udi_cdc_putc('#') ; // OK
//================================

// for test led green 
//sysclk_init();
//ioport_configure_pin(IOPORT_CREATE_PIN(PORTE, 3), IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);

while (true) {
	
	//PORTE.OUTCLR= 0x08;
	
	
	//sleepmgr_enter_sleep();
	
	//udi_cdc_putc('S');   // for test
	xor_output = 0;
	xor_input  = 0xA0;   //  
	
		for (ii=0; ii<20 ; ii++ )
		{
			rx_buf[ii]=0;
		}

	 //=================================== Get Command from LabView ===================================
	 //iram_size_t udi_cdc_read_buf(void* buf, iram_size_t size)
	 //buf_read_size= udi_cdc_read_buf(rx_buf, 20);
	 ii=0;
	 
	 while ( ii < 9 )
	 { 
	  if (udi_cdc_is_rx_ready() ) 
		{
		
			rx_buf[ii] = udi_cdc_getc();
			xor_input = xor_input ^ rx_buf[ii] ;
			ii++ ;
		 }
		}
		
	/*
	// for test
	udi_cdc_putc('N') ; // OK
	for (ii=0; ii<10 ; ii++ )
	{
		udi_cdc_putc(rx_buf[ii]);
	}
	
	//udi_cdc_putc(xor_input);
	udi_cdc_putc('Q') ; // OK
	*/
	
	// check packet
	// AB ADCEN DACEN VarVol DAC01L DAC01H DAC02L DAC02H XOR BB
	//xor_input = xor_input  ^ 0xBB ;
	if ( (rx_buf[0] == 0xAB) && (rx_buf[8] == 0xBB) ) //   && (xor_input==0))
	{
	 // packet is correct
	 udi_cdc_putc('#') ; // OK
	 udi_cdc_putc('K') ; // OK
	 
	//============================== DAC Setting  ==============================
	// set DAC value
	dac_val0 =  rx_buf[4] + (((uint16_t)rx_buf[3]) <<8) ;
	dac_val1 =  rx_buf[6] + (((uint16_t)rx_buf[5]) <<8) ;
	
	f_dac_val0=(dac_val0*gain_dac0)-offs_dac0;
	f_dac_val1=(dac_val1*gain_dac1)-offs_dac1;

	f_dac_val0 = (f_dac_val0 <0) ? 0 : f_dac_val0 ;
	f_dac_val1 = (f_dac_val1 <0) ? 0 : f_dac_val1 ;

	dac_val0=(uint16_t) f_dac_val0;
	dac_val1=(uint16_t) f_dac_val1;
				  
	//while ( DAC_Channel_DataEmpty( &DACA, CH0 ) == 0 ) {}
	//delay_us(100);
	//DAC_Channel_Write( &DACB, dac_val0, CH0 );
	//while ( DAC_Channel_DataEmpty( &DACA, CH1 ) == 0 ) {}
	//delay_us(100);
	//DAC_Channel_Write( &DACB, dac_val1, CH1 );
	
	// Wait for channels to get ready for new values, then set the value 
	wait_for_timer();
	//for (i=0; i< 100; i++)	for (j=0; j< 100; j++) ; // delay some time
	dac_set_channel_value(&OUTPUT_DAC, DAC_CH0, dac_val0);
	dac_set_channel_value(&OUTPUT_DAC, DAC_CH1, dac_val1);
	//============================ END DAC Setting  ============================


//================================= send data on SPI  =================================
    // Send command to spi micro for set variable voltag e value
	
	/*_________commented by ghiasi__________	
    var_vol =  rx_buf[2];
	_______________________________________*/
	
	/*
    PORTC.OUT=PORTC.OUT&0xEF;  // spi cs enable
    value=spic_master_tx_rx(0x11);
    value=spic_master_tx_rx(var_vol);
    PORTC.OUT=PORTC.OUT|0x10;  // spi cs disable
    delay_us(100);
	*/
		//________________added by ghiasi______________	
	switch (rx_buf[2])
	{
		case 5 :    var_vol =  0x11;    gain_var=var_gain[0]; offs_var=var_offs[0];
		break;
		case 6 :    var_vol =  0x39;    gain_var=var_gain[1]; offs_var=var_offs[1];
		break;
		case 7 :    var_vol =  0x55;    gain_var=var_gain[2]; offs_var=var_offs[2];
		break;
		case 8 :    var_vol =  0x6A;    gain_var=var_gain[3]; offs_var=var_offs[3];
		break;
		case 9 :    var_vol =  0x7A;    gain_var=var_gain[4]; offs_var=var_offs[4];
		break;
		case 10:    var_vol =  0x87;    gain_var=var_gain[5]; offs_var=var_offs[5];
		break;
		
		default:    var_vol =  0x11;    gain_var=var_gain[0]; offs_var=var_offs[0];
	};
	//_____________________________________________
		
	spi_select_device(&SPIC, &spi_device_conf);  // Enable SPI CS (Active Low)
	spi_write_single(&SPIC, 0x11);
	while (!spi_is_rx_full(&SPIC)) {	}
	spi_write_single(&SPIC,var_vol);
	while (!spi_is_rx_full(&SPIC)) {	}
	//for (i=0; i< 100; i++)	for (j=0; j< 100; j++) ; // delay some time
	spi_deselect_device(&SPIC, &spi_device_conf);   // Disable SPI CS (Active Low)
	//for (i=0; i< 1000; i++)	for (j=0; j< 100; j++) ; // delay some time
//=============================== END send data on SPI  ===============================
	
	}
else    //  packet is wrong
  {
	   udi_cdc_putc('@') ; // error
	   udi_cdc_putc(xor_input);
   }
   	
	//PORTE.OUTSET= 0x08; 
//============================== Send Data (ADC) to PC with USB ================================

     udi_cdc_putc(0xAB);
     // send ADC0-7 value to USB
	 
	  //==================== READ ADC A ====================

		f_adc_value  = adc_read_cmplt(&MY_ADC_A, ADCA_CH0);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH0);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH0);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH0);
		f_adc_value /=4;
		f_adc_value = f_adc_value * gain ;
		if (f_adc_value < offset) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offset;
		adc_value[0] = (uint16_t)f_adc_value ;
		//------------------------------------------------
		f_adc_value  = adc_read_cmplt(&MY_ADC_A, ADCA_CH1);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH1);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH1);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH1);
		f_adc_value /=4;
		f_adc_value = f_adc_value * gain ;
		if (f_adc_value<offset) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offset;
		adc_value[1] = (uint16_t)f_adc_value ;
		//------------------------------------------------
		
		f_adc_value  = adc_read_cmplt(&MY_ADC_A, ADCA_CH2);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH2);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH2);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH2);
		f_adc_value /=4;
		f_adc_value = f_adc_value * gain_12v ;
		if (f_adc_value<offset_12v) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offset_12v;
		adc_value[2] = (uint16_t)f_adc_value ;
		//------------------------------------------------
		
		f_adc_value  = adc_read_cmplt(&MY_ADC_A, ADCA_CH3);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH3);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH3);
		f_adc_value += adc_read_cmplt(&MY_ADC_A, ADCA_CH3);
		f_adc_value /=4;
		f_adc_value = f_adc_value * gain_3v ;
		if (f_adc_value<offset_3v) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offset_3v;
		adc_value[3] = (uint16_t)f_adc_value ;
		
		
		//==================== READ ADC B ====================

		//_______________________+-10 Vol_________________________
		f_adc_value  = adc_read_cmplt(&MY_ADC_B, ADCA_CH0);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH0);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH0);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH0);
		f_adc_value /=4;
		
		f_adc_value = f_adc_value * gain_10volt ;
		if (f_adc_value<offs_10volt) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offs_10volt;

		adc_value[4] = (uint16_t)f_adc_value ;		
		//________________________Var Vol_______________________________
		f_adc_value  = adc_read_cmplt(&MY_ADC_B, ADCA_CH1);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH1);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH1);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH1);
		f_adc_value /=4;
		
		f_adc_value = f_adc_value * gain_var ;
		if (f_adc_value<offs_var) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offs_var;
		
		adc_value[5] = (uint16_t)f_adc_value ;
	
		//------------------------------------------------
		
		f_adc_value  = adc_read_cmplt(&MY_ADC_B, ADCA_CH2);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH2);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH2);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH2);
		f_adc_value /=4;
		f_adc_value = f_adc_value * gain ;
		if (f_adc_value<offset) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offset;
		adc_value[6] = (uint16_t)f_adc_value ;
		//------------------------------------------------
		
		f_adc_value  = adc_read_cmplt(&MY_ADC_B, ADCA_CH3);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH3);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH3);
		f_adc_value += adc_read_cmplt(&MY_ADC_B, ADCA_CH3);
		f_adc_value /=4; 
		f_adc_value = f_adc_value * gain ;
		if (f_adc_value<offset) 	 f_adc_value=0;
		else 						f_adc_value = f_adc_value - offset;
		adc_value[7] = (uint16_t)f_adc_value ;
				 
/*
adc_value[4] = adc_read_cmplt(&MY_ADC_B, ADCB_CH0);
adc_value[5] = adc_read_cmplt(&MY_ADC_B, ADCB_CH1);
adc_value[6] = adc_read_cmplt(&MY_ADC_B, ADCB_CH2);
adc_value[7] = adc_read_cmplt(&MY_ADC_B, ADCB_CH3);
*/
		//------------------------------------------------
	

	for (i=0; i < 8; i++ )
     {
		 index_bit =  rx_buf[1] >> i ;
		 if ((index_bit & 0x01)==0) 	 {adc_value[i] =0;	 }
	 }
	 

	  for (i=0; i < 8; i++ )
     {
		 
// 		putdecint (adc_value[i]);
// 		udi_cdc_putc(' ');
		 udi_cdc_putc(adc_value[i]>>8);
	     udi_cdc_putc(adc_value[i]);		 
	     xor_output = xor_output ^ adc_value[i] ;
	     xor_output = xor_output ^ (adc_value[i]>>8) ;
	     // delay_ms(100);
     }
    // End send ADC0-7 value to uart

     udi_cdc_putc(xor_output);
     udi_cdc_putc(0xBB);
//============================ End Send Data (ADC) to PC with USB ==============================


	} // End While

}


void spi_init_pins(void)
{
	//ioport_configure_port_pin(&PORTC, PIN1_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT);
	
	ioport_configure_port_pin(&PORTC, PIN4_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT);
	ioport_configure_port_pin(&PORTC, PIN5_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT);
	ioport_configure_port_pin(&PORTC, PIN6_bm, IOPORT_DIR_INPUT);
	ioport_configure_port_pin(&PORTC, PIN7_bm, IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT);
}

void spi_init_module(void)
{
	struct spi_device spi_device_conf = { .id = IOPORT_CREATE_PIN(PORTC, 4)
	};

	spi_master_init(&SPIC);
	spi_master_setup_device(&SPIC, &spi_device_conf, SPI_MODE_0, 100000, 0);
	spi_enable(&SPIC);
}


void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
		return;
	ui_process(udd_get_frame_number());
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	ui_powerdown();
}

void main_remotewakeup_lpm_disable(void)
{
	ui_wakeup_disable();
}

void main_remotewakeup_lpm_enable(void)
{
	ui_wakeup_enable();
}
#endif

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
	uart_open(port);
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	uart_close(port);
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		ui_com_open(port);
	}else{
		// Host terminal has close COM
		ui_com_close(port);
	}
}

