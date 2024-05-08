/*!
	@file     main.cpp
	@author   Gavin Lyons
	@brief Example cpp file for ST7735_TFT_PICO library.
			Tests  Hello World
	@note  See USER OPTIONS 0-3 in SETUP function

	@test
		-# Test0 Print out Hello world  

*/

// Section ::  libraries 


// Section :: Defines   
//  Test timing related defines 
#define TEST_DELAY1 1000 // mS
#define TEST_DELAY2 2000 // mS
#define TEST_DELAY5 5000 // mS

// Section :: Globals 
//ST7735_TFT myTFT;

//  Section ::  Function Headers 

void tft_init(void);  // setup + user options
void Test0(char a);  
void EndTests(void);
