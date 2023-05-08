/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

/*
	Code adjusted by Eva Zwetsloot (2023)
		- Adjustments include a rewritten version of the handleRFCommands()

*/

#include "mirf.h"
#include "nRF24L01.h"
#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )


// Flag which denotes transmitting mode
volatile uint8_t PTX;

void mirf_init() 
// Initializes pins as interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
    // Define CSN and CE as Output and set them to default
    //DDRB |= ((1<<CSN)|(1<<CE));
    mirf_CE_hi;
    mirf_CSN_hi;

	mirf_config();
}


void mirf_config() 
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
{

	uint8_t temp[3];

	// power down
	mirf_config_register(CONFIG, 0x0D);

	// address width
	mirf_config_register(SETUP_AW, 0x01);

	// tx address
	temp[0] = (rfAddress>>8)&0xFF;
	temp[1] = rfAddress & 0xFF;
	temp[2] = 0x00;
	mirf_write_register(TX_ADDR, temp, 3);	

	// rx address => same as tx address for auto ack
	mirf_write_register(RX_ADDR_P0, temp, 3);

	// enable auto ack for pipe0
	mirf_config_register(EN_AA, 0x01);

	// enable pipe0
	mirf_config_register(EN_RXADDR, 0x01);

	// 500µs (+ 86µs on-air), 2 re-transmissions
	mirf_config_register(SETUP_RETR, 0x12);

    // select RF channel
    mirf_config_register(RF_CH,40);

	// RX payload size; it isn't needed because the dynamic payload length is activated for ACK+PAYLOAD feature
    mirf_config_register(RX_PW_P0, PAYLOAD_SIZE);

	// enable extra features
    mirf_CSN_lo;
    SPI_Write_Byte(NRF_ACTIVATE);
    SPI_Write_Byte(0x73);
    mirf_CSN_hi;
	
	// enable dynamic payload for pipe0
	mirf_config_register(NRF_DYNPD, 0x01);

	// enable payload with ACK and dynamic payload length
	mirf_config_register(NRF_FEATURE, 0x06);
		
	// power up; enable crc (2 bytes); prx; max_rt, tx_ds enabled
	mirf_config_register(CONFIG, 0x0F);	

    // Start receiver 
    //PTX = 0;        // Start in receiving mode
    //RX_POWERUP;     // Power up in receiving mode
    //mirf_CE_hi;     // Listening for pakets
}

void mirf_set_RADDR(uint8_t * adr) 
// Sets the receiving address
{
    mirf_CE_lo;
    mirf_write_register(RX_ADDR_P0,adr,5);
    mirf_CE_hi;
}

void mirf_set_TADDR(uint8_t * adr)
// Sets the transmitting address
{
	mirf_write_register(TX_ADDR, adr,5);
}

/*
#if defined(__AVR_ATmega8__)
SIGNAL(SIG_INTERRUPT0) 
#endif // __AVR_ATmega8__
#if defined(__AVR_ATmega168__)
SIGNAL(SIG_PIN_CHANGE2) 
#endif // __AVR_ATmega168__  
// Interrupt handler 
{
    uint8_t status;   
    // If still in transmitting mode then finish transmission
    if (PTX) {
    
        // Read MiRF status 
        mirf_CSN_lo;                                // Pull down chip select
        status = spi_fast_shift(NOP);               // Read status register
        mirf_CSN_hi;                                // Pull up chip select

        mirf_CE_lo;                             // Deactivate transreceiver
        RX_POWERUP;                             // Power up in receiving mode
        mirf_CE_hi;                             // Listening for pakets
        PTX = 0;                                // Set to receiving mode

        // Reset status register for further interaction
        mirf_config_register(STATUS,(1<<TX_DS)|(1<<MAX_RT)); // Reset status register
    }
}
*/

uint8_t mirf_data_ready() 
// Checks if data is available for reading
{
    if (PTX) return 0;
    uint8_t status;
    // Read MiRF status 
    mirf_CSN_lo;                                // Pull down chip select
    status = SPI_Write_Byte(NOP);               // Read status register
    mirf_CSN_hi;                                // Pull up chip select
    return status & (1<<RX_DR);

}

uint8_t rx_fifo_is_empty() {
	
	uint8_t fifo_status = 0;

	mirf_read_register(FIFO_STATUS, &fifo_status, 1);
	
	return (uint8_t)(fifo_status&0x01);
}

void flush_rx_fifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_RX);
    mirf_CSN_hi;

}

void mirf_get_data(uint8_t * data) 
// Reads mirf_PAYLOAD bytes into data array
{
    mirf_CSN_lo;                               		// Pull down chip select
    SPI_Write_Byte( R_RX_PAYLOAD );            		// Send cmd to read rx payload
    SPI_ReadWrite_Block(data,data,PAYLOAD_SIZE); 	// Read payload
    mirf_CSN_hi;                               		// Pull up chip select
    mirf_config_register(STATUS,(1<<RX_DR));   		// Reset status register
}

void mirf_config_register(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Byte(value);
    mirf_CSN_hi;
}

void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(R_REGISTER | (REGISTER_MASK & reg));
    SPI_ReadWrite_Block(value,value,len);
    mirf_CSN_hi;
}

void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len) 
// Writes an array of bytes into inte the MiRF registers.
{
    mirf_CSN_lo;
    SPI_Write_Byte(W_REGISTER | (REGISTER_MASK & reg));
    SPI_Write_Block(value,len);
    mirf_CSN_hi;
}


void mirf_send(uint8_t * value, uint8_t len) 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    while (PTX) {}                  // Wait until last paket is send

    mirf_CE_lo;

    PTX = 1;                        // Set to transmitter mode
    TX_POWERUP;                     // Power up
    
    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CSN_lo;                    // Pull down chip select
    SPI_Write_Byte( W_TX_PAYLOAD ); // Write cmd to write payload
    SPI_Write_Block(value,len);   // Write payload
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CE_hi;                     // Start transmission
}

void writeAckPayload(unsigned char *data, unsigned char size) {

	unsigned char k = 0;

	flushTxFifo();

    mirf_CSN_lo;

	SPI_Write_Byte(NRF_W_ACK_PAYLOAD_P0);

	for(k=0; k<size; k++) {
		SPI_Write_Byte(data[k]);
	}	

    mirf_CSN_hi;


}

uint8_t readPayloadWidthFromTopFifo() {
	uint8_t pWidth = 0;

    mirf_CSN_lo;
    SPI_Write_Byte(NRF_R_RX_PL_WID);
	pWidth = SPI_Write_Byte(NOP); 	// not specified in the datasheet but the "NRF_R_RX_PL_WID" has a parameter,
									// we need to send a NOP to receive the actual payload size
    mirf_CSN_hi;
	
	return pWidth;
}

uint8_t readPayloadWidthFromPipe0() {
	uint8_t pWidth = 0;

	mirf_read_register(RX_PW_P0, &pWidth, 1);
	
	return pWidth;
}

void flushTxFifo() {

    mirf_CSN_lo;
    SPI_Write_Byte(FLUSH_TX);
    mirf_CSN_hi;

}

bool handleRFCommands() {
	
	unsigned int i = 0;
	unsigned int id_neigh;
	unsigned int id_start = 0;
	unsigned int r_speed_temp = 0;
	unsigned int l_speed_temp = 0;
	bool out;
	
	out = false;

	if(mirf_data_ready()) {
		
		out = true;

		rfFlags |= 0x02;

		// clear irq status
		mirf_config_register(STATUS, 0x70);

		if(rx_fifo_is_empty()) {
			return;
		}

		mirf_get_data(rfData);
		flush_rx_fifo();

		//usartTransmit(rfData[0]);

		if(rfDebugMode==1) {

			writeAckPayload(ackPayload, 16);
			
		} else {		
			
			if ((rfData[3]&0b10000000)==0b10000000){
				// initialisation of the positions in x_hat
				id_start = 0x0F & rfData[3];
								
				if ((rfData[3]&0b01010000)==0b00000000){
					// type 1
					if(id_start == 0){
						id = rfData[0];
						ROBOTS = rfData[4];
						filtered_theta = (((signed int) (rfData[2]<<8) | (unsigned char) rfData[1]))/573.0; //to radians
						theta = filtered_theta;
						
						Xpos_array[id] = ((signed int) (rfData[6]<<8) | (unsigned char) rfData[5]);
						Ypos_array[id] = ((signed int) (rfData[8]<<8) | (unsigned char) rfData[7]);
						
						error_radius = ((signed int) (rfData[10]<<8) | (unsigned char) rfData[9]);
						u_max = ((signed int) (rfData[12]<<8) | (unsigned char) rfData[11]);
						
						//Set all variables related to Xpos
						xPos_fixed = Xpos_array[id];
						yPos_fixed = Ypos_array[id];
						xPos = Xpos_array[id];
						yPos = Ypos_array[id];
						position[0] = Xpos_array[id];
						position[1] = Ypos_array[id];
						fixed_position[0] = Xpos_array[id];
						fixed_position[1] = Ypos_array[id];
					}else{
						id_start = (id_start-1)*3;
						
						Xpos_array[id_start] = ((signed int) (rfData[1]<<8) | (unsigned char) rfData[0]);
						Ypos_array[id_start] = ((signed int) (rfData[4]<<8) | (unsigned char) rfData[2]);
						
						Xpos_array[id_start+1] = ((signed int) (rfData[6]<<8) | (unsigned char) rfData[5]);
						Ypos_array[id_start+1] = ((signed int) (rfData[8]<<8) | (unsigned char) rfData[7]);
						
						Xpos_array[id_start+2] = ((signed int) (rfData[10]<<8) | (unsigned char) rfData[9]);
						Ypos_array[id_start+2] = ((signed int) (rfData[12]<<8) | (unsigned char) rfData[11]);
									
					}
					
				}else if ((rfData[3]&0b01010000)==0b00010000){
					// type 2
					
						id_start = id_start*5;
					
						L[id_start] = ((signed int) (rfData[1]<<8) | (unsigned char) rfData[0]);
						L[id_start+1] = ((signed int) (rfData[4]<<8) | (unsigned char) rfData[2]);
						
						L[id_start+2] = ((signed int) (rfData[6]<<8) | (unsigned char) rfData[5]);
						L[id_start+3] = ((signed int) (rfData[8]<<8) | (unsigned char) rfData[7]);
						
						L[id_start+4] = ((signed int) (rfData[10]<<8) | (unsigned char) rfData[9]);
						L[id_start+5] = ((signed int) (rfData[12]<<8) | (unsigned char) rfData[11]);
					
				}else if ((rfData[3]&0b01010000)==0b01000000){
					// type 3
					
						id_start = id_start*5;
					
						if (currentSelector == 13 || currentSelector == 12){
							Bx[id_start] = ((signed int) (rfData[1]<<8) | (unsigned char) rfData[0]);
							Bx[id_start+1] = ((signed int) (rfData[4]<<8) | (unsigned char) rfData[2]);
											
							Bx[id_start+2] = ((signed int) (rfData[6]<<8) | (unsigned char) rfData[5]);
							Bx[id_start+3] = ((signed int) (rfData[8]<<8) | (unsigned char) rfData[7]);
											
							Bx[id_start+4] = ((signed int) (rfData[10]<<8) | (unsigned char) rfData[9]);
							Bx[id_start+5] = ((signed int) (rfData[12]<<8) | (unsigned char) rfData[11]);
						} else{
							B[0][id_start] = ((signed int) (rfData[1]<<8) | (unsigned char) rfData[0]);
							B[0][id_start+1] = ((signed int) (rfData[4]<<8) | (unsigned char) rfData[2]);
														
							B[0][id_start+2] = ((signed int) (rfData[6]<<8) | (unsigned char) rfData[5]);
							B[0][id_start+3] = ((signed int) (rfData[8]<<8) | (unsigned char) rfData[7]);
														
							B[0][id_start+4] = ((signed int) (rfData[10]<<8) | (unsigned char) rfData[9]);
							B[0][id_start+5] = ((signed int) (rfData[12]<<8) | (unsigned char) rfData[11]);	
						}
					
				} else if ((rfData[3]&0b01010000)==0b01010000){
					// type 4
					
						id_start = id_start*5;
						
						if (currentSelector == 13 || currentSelector == 12){
							By[id_start] = ((signed int) (rfData[1]<<8) | (unsigned char) rfData[0]);
							By[id_start+1] = ((signed int) (rfData[4]<<8) | (unsigned char) rfData[2]);
											
							By[id_start+2] = ((signed int) (rfData[6]<<8) | (unsigned char) rfData[5]);
							By[id_start+3] = ((signed int) (rfData[8]<<8) | (unsigned char) rfData[7]);
											
							By[id_start+4] = ((signed int) (rfData[10]<<8) | (unsigned char) rfData[9]);
							By[id_start+5] = ((signed int) (rfData[12]<<8) | (unsigned char) rfData[11]);
						} else{
							B[1][id_start] = ((signed int) (rfData[1]<<8) | (unsigned char) rfData[0]);
							B[1][id_start+1] = ((signed int) (rfData[4]<<8) | (unsigned char) rfData[2]);
						
							B[1][id_start+2] = ((signed int) (rfData[6]<<8) | (unsigned char) rfData[5]);
							B[1][id_start+3] = ((signed int) (rfData[8]<<8) | (unsigned char) rfData[7]);
						
							B[1][id_start+4] = ((signed int) (rfData[10]<<8) | (unsigned char) rfData[9]);
							B[1][id_start+5] = ((signed int) (rfData[12]<<8) | (unsigned char) rfData[11]);
						}						
						
				}
				
				
			}else{
				// operational packet
				id_neigh = rfData[4];
			
				// Change this to theta in future
				filtered_theta = (((signed int) (rfData[2]<<8) | (unsigned char) rfData[1]))/573.0;
				//resetOrientation();
				
				
				if(calibrateOdomFlag==0) {
					speedr = (rfData[1]&0x7F);	// cast the speed to be at most 127, thus the received speed are in the range 0..127 (usually 0..100),
					speedl = (rfData[2]&0x7F);	// the received speed is then shifted by 3 (x8) in order to have a speed more or less
					// in the same range of the measured speed that is 0..800.
					// In order to have greater resolution at lower speed we shift the speed only by 2 (x4),
					// this means that the range is more or less 0..400.


					if((rfData[1]&0x80)==0x80) {			// motor right forward
						pwm_right_desired = speedr; 		// speed received (0..127) is expressed in 1/5 of mm/s (0..635 mm/s)
						} else {								// backward
						pwm_right_desired = -(speedr);
					}

					if((rfData[2]&0x80)==0x80) {			// motor left forward
						pwm_left_desired = speedl;
						} else {								// backward
						pwm_left_desired = -(speedl);
					}

				}
				
				// Indicate when to leave idle state
				if((rfData[3]&0b01000000)==0b01000000) {
					start_comm = true;
					} else {
					start_comm = false;
				}
			
				
				// Xpos and Ypos needs to be assigned this value and Xpos_array needs to be updated only if triggered + delay
				position[0] = ((signed int) (rfData[i*4+6]<<8) | (unsigned char) rfData[i*4+5]);
				position[1] = ((signed int) (rfData[i*4+8]<<8) | (unsigned char) rfData[i*4+7]);
				
				// Only update the array if data is sent
				if ((rfData[3]&0b00010000)==0b00010000){
					Xpos_array[id_neigh] = ((signed int) (rfData[i*4+10]<<8) | (unsigned char) rfData[i*4+9]);
					Ypos_array[id_neigh] = ((signed int) (rfData[i*4+12]<<8) | (unsigned char) rfData[i*4+11]);										
				}
				
				// reset data when requested
				if ((rfData[0]&0b00000001)==0b00000001){
					reset_flag = true;
					xPos = position[0];		
					yPos = position[1];			
				}
				
			}
			
			if(currentSelector == 2) {
				if(calibrateOdomFlag==0) {
					if((rfData[7]&0b00000001)==0b00000001) {
						calibrateSensors();
						proximityResult[8] = 1023;	// because the first time this value could be low after calibration
						proximityResult[11] = 1023;	// and in that case a false black line will be detected
						calibState = CALIBRATION_STATE_FIND_THRS_0;
						calibVelIndex = 1;
						calibrateOdomFlag = 1;
					}
				}
			}
			
						
			// read and handle the remaining bytes of the payload (at the moment not used)


			// write back the ack payload
			ackPayload[0] = packetId&0xFF;

			switch(packetId) {
				case 3:
					ackPayload[1] = proximityResult[0]&0xFF;
					ackPayload[2] = proximityResult[0]>>8;
					ackPayload[3] = proximityResult[1]&0xFF;
					ackPayload[4] = proximityResult[1]>>8;
					ackPayload[5] = proximityResult[2]&0xFF;
					ackPayload[6] = proximityResult[2]>>8;
					ackPayload[7] = proximityResult[3]&0xFF;
					ackPayload[8] = proximityResult[3]>>8;
					ackPayload[9] = proximityResult[5]&0xFF;
					ackPayload[10] = proximityResult[5]>>8;
					ackPayload[11] = proximityResult[6]&0xFF;
					ackPayload[12] = proximityResult[6]>>8;
					ackPayload[13] = proximityResult[7]&0xFF;
					ackPayload[14] = proximityResult[7]>>8;
					#ifdef HW_REV_3_1
						ackPayload[15] = CHARGE_ON | (BUTTON0 << 1) | (CHARGE_STAT << 2);
					#else
						ackPayload[15] = CHARGE_ON | (BUTTON0 << 1);
					#endif
					packetId = 4;
					break;

				case 4:
					ackPayload[1] = proximityResult[4]&0xFF;
					ackPayload[2] = proximityResult[4]>>8;
					ackPayload[3] = proximityResult[8]&0xFF;
					ackPayload[4] = proximityResult[8]>>8;
					ackPayload[5] = proximityResult[9]&0xFF;
					ackPayload[6] = proximityResult[9]>>8;
					ackPayload[7] = proximityResult[10]&0xFF;
					ackPayload[8] = proximityResult[10]>>8;
					ackPayload[9] = proximityResult[11]&0xFF;
					ackPayload[10] = proximityResult[11]>>8;
					ackPayload[11] = accX&0xFF;
					ackPayload[12] = accX>>8;
					ackPayload[13] = accY&0xFF;
					ackPayload[14] = accY>>8;
					ackPayload[15] = irCommand;
					packetId = 5;
					break;

				case 5:
					if (r_speed < 0){
						r_speed_temp = (((unsigned int) -r_speed)&0xFF); 
					}else{
						r_speed_temp = 0x80 | (((unsigned int) r_speed)&0xFF);	
					}
					 
					if (l_speed < 0){
						l_speed_temp = (((unsigned int) -l_speed)&0xFF);
					}else{
						l_speed_temp = 0x80 | (((unsigned int) l_speed)&0xFF);
					}					
				
				
					ackPayload[1] = init_time&0xFF;
					ackPayload[2] = init_time>>8;
					ackPayload[3] = init_time>>16;
					ackPayload[4] = init_time>>24;
					ackPayload[5] = comm_time&0xFF;
					ackPayload[6] = comm_time>>8;
					ackPayload[7] = comm_time>>16;
					ackPayload[8] = comm_time>>24;
					ackPayload[9] = r_speed_temp&0xFF;
					ackPayload[10] = turn&0xFF;
					ackPayload[11] = ((signed int) (filtered_theta*573.0))&0xFF;
					ackPayload[12] = ((signed int) (filtered_theta*573.0))>>8;
					ackPayload[13] = count_resets&0xFF;
					ackPayload[14] = count_resets>>8;
					ackPayload[15] = currentSelector;
					packetId = 6;
					break;

				case 6:
					ackPayload[1] = loop_time&0xFF;
					ackPayload[2] = loop_time>>8;
					ackPayload[3] = loop_time>>16;
					ackPayload[4] = loop_time>>24;
					ackPayload[5] = control_time&0xFF;
					ackPayload[6] = control_time>>8;
					ackPayload[7] = count_resets&0xFF;
					ackPayload[8] = count_resets>>8;
					ackPayload[9] = ((unsigned int) speedr)&0xFF;
					ackPayload[10] = ((unsigned int) speedr)>>8;
					ackPayload[11] = ((unsigned int) Ypos_array[ROBOTS-2])&0xFF;
					ackPayload[12] = ((unsigned int) Ypos_array[ROBOTS-2])>>8;	
					ackPayload[13] = ((unsigned int) Xpos_array[ROBOTS-2])&0xFF;
					ackPayload[14] = ((unsigned int) Xpos_array[ROBOTS-2])>>8;
					ackPayload[15] = 0;
					//ackPayload[15] = ((unsigned int)trigger_count)&0xFF;
					packetId = 7;
					break;


				case 7:
					lastTheta = theta;
					ackPayload[1] = ((unsigned int) xPos_fixed)&0xFF;
					ackPayload[2] = ((unsigned int) xPos_fixed)>>8;
					ackPayload[3] = ((unsigned int) yPos_fixed)&0xFF;
					ackPayload[4] = ((unsigned int) yPos_fixed)>>8;
					ackPayload[5] = (trigger_count)&0xFF;
					ackPayload[6] = (trigger_count)>>8;
					ackPayload[7] = turn&0xFF;
					ackPayload[8] = ((unsigned int) Ypos_array[id])&0xFF;
					ackPayload[9] = ((signed int)(lastTheta*573.0))&0xFF;	// radians to degrees => 573 = 1800/PI
					ackPayload[10] = ((signed int)(lastTheta*573.0))>>8;				
					ackPayload[11] = ((unsigned int)xPos)&0xFF;
					ackPayload[12] = ((unsigned int)xPos)>>8;
					ackPayload[13] = ((unsigned int)yPos)&0xFF;
					ackPayload[14] = ((unsigned int)yPos)>>8;
					ackPayload[15] = 0;
					packetId = 6;
					break;

			}
			if ((!turn) && ((rfData[0]&0b00000010)==0b00000010)){
				theta = filtered_theta;
			}
			
			writeAckPayload(ackPayload, 16);

		}

		

	}
	
	return out;

}

void rfEnableDebugMode() {
	rfDebugMode = 1;
	rfDebugCounter = 3;
}

void rfDisableDebugMode() {
	rfDebugMode = 0;
}

void rfDebugSendData() {
	ackPayload[0] = rfDebugCounter;
	while(rfData[0] != rfDebugCounter) {
		handleRFCommands();
	}
	ackPayload[0] = 0x00;
	if(rfDebugCounter < 255) {
		rfDebugCounter++;
	} else {
		rfDebugCounter = 3;
	}
}

void rfDebugNextPacket() {
	rfDebugCounter = 3;
}






