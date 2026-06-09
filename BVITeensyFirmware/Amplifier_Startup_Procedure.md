# Amplifier Startup Procedure


## Pin Definitions
#define AMPLIFIER_PIN_ENABLE_A 35
#define AMPLIFIER_PIN_ENABLE_B 34
#define AMPLIFIER_PIN_ENABLE_C 33
#define AMPLIFIER_PIN_PWM_A 7
#define AMPLIFIER_PIN_PWM_B 8
#define AMPLIFIER_PIN_PWM_C 25

## ASCII Definitions
	String asciiSetIdle			= "";
	String asciiGetCurrent		= "g r0x0c\r";
	String asciiGetBaud			= "g r0x90\r";
	String asciiGetEncoderCount = "g r0x17\r";
	String asciiGetName			= "g f0x92\r";
	String asciiSetBaud115237	= "s r0x90 115237\r";
	String asciiSetCurrentMode	= "s r0x24 3\r";
	String asciiSetEncoderZero	= "s r0x17 0\r";

## Startup Sequence
1. Configure Pins
   - Set pins as outputs
     - pinMode ( EN_A , OUTPUT )
     - pinMode ( EN_B , OUTPUT )
     - pinMode ( EN_C , OUTPUT )
     - pinMode ( PWM_A , OUTPUT )
     - pinMode ( PWM_B , OUTPUT )
     - pinMode ( PWM_C , OUTPUT )
   - Set PWM analog write resolution
     - analogWriteResolution ( PWM_A , 12 ) 
     - analogWriteResolution ( PWM_B , 12 ) 
     - analogWriteResolution ( PWM_C , 12 ) 
2. Set initial pin values (disabled)
   - digitalWriteFast ( EN_A , LOW ) 
   - digitalWriteFast ( EN_B , LOW ) 
   - digitalWriteFast ( EN_C , LOW ) 
3. Zero amplifier output
   - analogWrite ( PWM_A , 2047 ) 
   - analogWrite ( PWM_B , 2047 ) 
   - analogWrite ( PWM_C , 2047 ) 
4. Establish hardware serial connections at default baud
   - HWSerialA.begin ( 9600 ) 
   - HWSerialB.begin ( 9600 ) 
   - HWSerialC.begin ( 9600 ) 
5. Reset amplifiers 
   - Reset amplifier A
     - digitalWriteFast ( EN_A , HIGH ) 
     - digitalWriteFast ( EN_A , LOW ) 
     - delay ( 500 ) 
     - digitalWriteFast ( EN_A , HIGH ) 
   - Reset amplifier B
     - digitalWriteFast ( EN_B , HIGH ) 
     - digitalWriteFast ( EN_B , LOW ) 
     - delay ( 500 ) 
     - digitalWriteFast ( EN_B , HIGH ) 
   - Reset amplifier C
     - digitalWriteFast ( EN_C , HIGH ) 
     - digitalWriteFast ( EN_C , LOW ) 
     - delay ( 500 ) 
     - digitalWriteFast ( EN_C , HIGH ) 
6. Set Current Command Mode
   - sendPacket ( SERIAL_A , ascii_SetCurrentMode ) 
   - sendPacket ( SERIAL_B , ascii_SetCurrentMode ) 
   - sendPacket ( SERIAL_C , ascii_SetCurrentMode ) 
7. Enable
   - digitalWriteFast ( EN_A , HIGH )
   - digitalWriteFast ( EN_B , HIGH )
   - digitalWriteFast ( EN_C , HIGH )