#include "Bounce2.h"

//#include  "audiolib.h"
/*
  #include <Audio.h>
  #include <Wire.h>
  #include <SPI.h>
  #include <SD.h>
  #include <SerialFlash.h>
*/

//#include <ADC.h>
//#include <ADC_util.h>

#include "Adafruit_MCP23017.h"
//#include <i2c_t3.h>
//#include <MCP23017.h>

#include <FrequencyTimer2.h>



#include <EEPROM.h>
#include "EEPROMAnything.h"

#include <MIDI.h>
//instanciate objects

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1,  MIDI);
//#define MIDI_CC_TUNNING_H 100
//#define MIDI_CC_TUNNING_L 101

#include <MUX74HC4067.h>


// leds config *****************************************************************
#include <FastLED.h>

FASTLED_USING_NAMESPACE

#define NUM_LEDS              16+2 //max value (0-15)STEP leds 16=CLOCK 17=PLAY
#define CLOCK_LED_POS         17
#define PLAY_LED_POS          16

#define LED_DATA_PIN          15

//#define CLK_PIN     4
#define LED_TYPE              NEOPIXEL
#define COLOR_ORDER           RGB
#define LED_INTENSITY_by255   128 //max

CRGB leds[NUM_LEDS];

CRGB *ptr_clockled=&(leds[CLOCK_LED_POS]);
//ptr_clockled = &(leds[CLOCK_LED_POS]);

CRGB *ptr_playled=&(leds[PLAY_LED_POS]);
//ptr_playled = &(leds[PLAY_LED_POS]);

// software config ****************************************************************

#define FRAMES_PER_SECOND           75
#define LEDPIN                      LED_BUILTIN

#define DEFAULT_MAINCLOCK_SPEED_US  200000
#define CLOCKIN_TIMEOUT_uS          5000000
#define BOUNCE_MS                   10
#define STEP_BUTT_LONG_PRESS_MS     500

#define FCT_BUTT_REFRESH_MS         25
#define STEP_BUTT_REFRESH_MS        50
#define POT_REFRESH_MS              150
#define JACK_IN_REFRESH_MS          5

#define MAINCLOCK_MAX_STEP          240 //0//0 // mult of 24 for midi-clock 

#define GPIO_JACKandBUTT_ADDR       1 //or ? 0x20
#define GPIO_STEP_BUTTONS_ADDR      0

#define MAX_STEP                    17 // 16 regular step (1-16)  +  actual "temp" step (0)

#define MIDICLOCK                   0xF8

// hardware config ****************************************************************

//teensy analogInputs :
#define VOLTAGE_DIVIDER_PIN   A14
#define PORTAMENTO_PIN        A15
#define CLOCK_DIVIDER_PIN     A16
#define SPEED_PIN             A17
#define DUTY_CYCLE_PIN        A18
#define SEQUENCE_MODE_PIN     A19

//ADC *adc = new ADC(); // adc object

//teensy pwmOutputs :
#define  VOLTAGE_DIVIDER_OUTPUT_PIN 30 //14
#define  MIDI_CV_OUTPUT_PIN         29 // 2for future 

//teensy MUX controls pins
MUX74HC4067 mux_A_OUT(24, 25, 26, 27, 28);

//ext GPIO MCP23017
//remainder: SDA0: PIN18 ; SCL0: PIN19

//Adafruit_MCP23017 mcpJACKandBUTT;
Adafruit_MCP23017 mcpSTEP_BUTT;

//MCP23017 mcpJACKandBUTT = MCP23017(0x21);//0x21); ///removed

//MCP23017 mcpSTEP_BUTT = MCP23017(0x20);


//REMOVED//mcp pins JACK in-out and BUTT
//direct to teensy pin
#define CLOCK_IN_PIN          12
#define RESET_TRIG_IN_PIN     11

#define MULT_TRIG_OUT_PIN     10
#define RETRIG_OUT_PIN        9
#define DIV_CLOCK_OUT_PIN     8

#define START_RESTART_PIN     7
#define STOP_PIN              6
#define NEXT_STEP_PIN         4

//jack outs
#define NUM_JACK_OUT 3
const uint8_t JACK_OUT_PINS[NUM_JACK_OUT] = {MULT_TRIG_OUT_PIN, RETRIG_OUT_PIN, DIV_CLOCK_OUT_PIN};

//FCT_BUTT
#define NUM_FCT_BUTT 3
const uint8_t FCT_BUTT_PINS[NUM_FCT_BUTT] = {START_RESTART_PIN, STOP_PIN, NEXT_STEP_PIN};
Bounce * fct_buttons = new Bounce[NUM_FCT_BUTT];

//STEP switch
Bounce * step_buttons = new Bounce[MAX_STEP];
bool	step_buttons_latch[MAX_STEP];

//FCT_BUTT
void  sequencer_restart(); //butt
void  sequencer_stop();
void  sequencer_play_next_step();

typedef void (*fct_buttonFellActionList[])();
fct_buttonFellActionList fctButtonFellAction = { sequencer_restart, sequencer_stop, sequencer_play_next_step };

/*
//long press
void  sequencer_long_restart(); //butt
void  sequencer_long_stop();
void  sequencer_play_previous_step();

typedef void (*fct_buttonLongPressActionList[])();
fct_buttonLongPressActionList fctButtonLongPressAction = { sequencer_long_restart, sequencer_long_stop, sequencer_play_previous_step };
*/


//CLOCK_DIVIDER_PIN, SPEED_PIN, DUTY_CYCLE_PIN, SEQUENCE_MODE_PIN, VOLTAGE_DIVIDER_PIN, PORTAMENTO_PIN
void  update_CLOCK_DIVIDER();
void  update_SPEED();
void  update_DUTY();
void  update_SEQUENCE_MODE();
void  update_VOLTAGE_DIVIDER();
void  update_PORTAMENTO();

#define NUM_POTS 6
#define FUNCTION_POTS_LPF_COEF 0.7

const uint8_t FUNCTION_POTS_PINS[NUM_POTS] = { VOLTAGE_DIVIDER_PIN, PORTAMENTO_PIN, CLOCK_DIVIDER_PIN, SPEED_PIN, DUTY_CYCLE_PIN, SEQUENCE_MODE_PIN };

int      FUNCTION_POTS_values[NUM_POTS] = { 0, 0, 0, 0, 0, 0 };
float    FUNCTION_POTS_lpf[NUM_POTS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

typedef void (*functionPotsActionList[])();
functionPotsActionList functionPotsAction = { update_VOLTAGE_DIVIDER, update_PORTAMENTO, update_CLOCK_DIVIDER, update_SPEED, update_DUTY, update_SEQUENCE_MODE  };

//jack INs //jack input
#define NUM_JACK_IN 2

const uint8_t JACK_IN_PINS[NUM_JACK_IN] = {CLOCK_IN_PIN, RESET_TRIG_IN_PIN};
Bounce * jack_in = new Bounce[NUM_JACK_IN];

void  trig_CLOCK_IN();
typedef void (*jack_inputFellActionList[])();
jack_inputFellActionList jack_inputFellAction = { trig_CLOCK_IN, sequencer_restart };

//MIDI
elapsedMillis MIDICLOCK_IN_elapsed=0;

//enum
enum voltage_divider_list { div1_l, div2_l, div4_l, div8_l , div16_l };

enum sequence_mode_list { SINGLEAx16, LOOPAx16, SINGLEAx16PP, LOOPAx16PP, RND };

enum funtionsPots_l { voltage_divider_l, portamento_l, clock_divider_l, speed_l, duty_cycle_l, sequence_mode_l };

struct StepAnalog_s {
  bool      selected;
  uint16_t  value;
  bool      muted;
  bool      rec_enabled;
  uint16_t  portamento;
  bool      first;
  bool      last;
  void      resetStep()
  {
    selected = false;
    value = 0;
    muted = false;
    rec_enabled = false;
    portamento = 0;
    first = false;
    last = false;
  }
} ;

struct Pattern_s {
  bool      selected;

  float     A_clock_divider;
  float     A_fullScale; // 0.0 to 1.0
  uint8_t   A_sourceFrom;

  uint8_t   A_first_step; // 0 = default
  volatile uint8_t   A_last_step; //  0 = over 17


  StepAnalog_s StepA[MAX_STEP];

  void      resetPattern()
  {
    selected = false;

    A_clock_divider = 0;
    A_fullScale = 1.0; // 0.0 to 1.0

    A_first_step = 1; // 0 = default
    A_last_step = MAX_STEP; //  0 = over 17

    for( int i=0 ; i < MAX_STEP; i++ )
    {
      StepA[MAX_STEP].resetStep();
    }

  }

};

struct Songs_s {
  
  float        clock_mult_div;
  uint32_t     clock_speed; // bpm 20.00-300.00
  volatile  uint16_t     clock_duty; //0.0 - 1.0

  //char      song_name[9] ="";

  bool      CLOCK_IN_polarity;
  bool      RESET_TRIG_IN_polarity;
  bool      MULT_TRIG_OUT_polarity;
  bool      DIV_CLOCK_OUT_polarity;
  bool      RETRIG_OUT_polarity;

  Pattern_s  Pattern;

  void      resetSong()
  {
    clock_mult_div = 1.0;
    clock_speed = 2000; // bpm 20.00-300.00
    clock_duty = MAINCLOCK_MAX_STEP / 2; //0.0 - 1.0


    CLOCK_IN_polarity = false;
    RESET_TRIG_IN_polarity  = false;
    MULT_TRIG_OUT_polarity  = false;
    DIV_CLOCK_OUT_polarity  = false;
    RETRIG_OUT_polarity  = false;
    Pattern.resetPattern();

  }

};

Songs_s Song;

struct runningState_s {
  bool  play;
  bool  pause;
  bool  stop;
  // bool  record;
  volatile  bool    modePP_dirA;

  volatile  uint8_t sequence_mode;
  uint8_t voltage_divider;
  float   clock_divider;
  float   portamento;


  volatile  uint8_t loaded_A_step;

  elapsedMillis last_LED_refresh;
  elapsedMillis last_POT_refresh;
  elapsedMillis last_FCT_BUTT_refresh;
  elapsedMillis last_STEP_BUTT_refresh;
  elapsedMillis last_JACK_IN_refresh;

  unsigned int  mainClock_speed_us;
  volatile uint32_t      mainClock_true_speed_us;

    
  uint16_t      mainClock_step_copy;
  volatile uint16_t      mainClock_duty;

  uint8_t   MIDI2CV_oct_Max;
  uint16_t  MIDI2CV_oct_L;
  uint16_t  MIDI2CV_oct_H;
  uint16_t  MIDI2CV_oct_First;

};

runningState_s  actualRunningState;

struct configuration_s {
  bool      CLOCK_IN_polarity;
  bool      RESET_TRIG_IN_polarity;
  bool      MULT_TRIG_OUT_polarity;
  bool      DIV_CLOCK_OUT_polarity;
  bool      RETRIG_OUT_polarity;
  uint16_t  MIDI2CV_oct_L;
  uint16_t  MIDI2CV_oct_H;
  uint16_t  MIDI2CV_oct_First;
  
};

configuration_s configuration_EEPROM;

#define NUM_CONF_FLAGS  5

bool *(EEPROM_config_flags[5])={
  &configuration_EEPROM.CLOCK_IN_polarity,
  &configuration_EEPROM.RESET_TRIG_IN_polarity,
  &configuration_EEPROM.MULT_TRIG_OUT_polarity,
  &configuration_EEPROM.DIV_CLOCK_OUT_polarity,
  &configuration_EEPROM.RETRIG_OUT_polarity
};

bool *(running_state_config_flags[5])={
  &Song.CLOCK_IN_polarity,
  &Song.RESET_TRIG_IN_polarity,
  &Song.MULT_TRIG_OUT_polarity,
  &Song.DIV_CLOCK_OUT_polarity,
  &Song.RETRIG_OUT_polarity
};

// Create an IntervalTimer object
//IntervalTimer       mainClock;

volatile  bool      mainClock_tick=false;
volatile  uint16_t  mainClock_step=0;

elapsedMicros   	CLOCK_IN_elapsed;
bool            	CLOCK_IN_first;

//LPF
float portamento_LPF = 0.1;

//MIDI2CV config
float octave_base[12]={
  1.0,
  1.059463,
  1.122462,
  1.189207,
  1.259921,
  1.334840,
  1.414214,
  1.498307,
  1.587401,
  1.681793,
  1.781797,
  1.887749
};

#define CRITICAL(X) cli(); X; sei();
//rnd
//typedef struct ranctx { uint32_t a; uint32_t b; uint32_t c; uint32_t d; };
//ranctx ranctx;

//CLOCK *******************************************

void  mainClock_init()
{
  actualRunningState.mainClock_speed_us=Song.clock_speed;
  actualRunningState.mainClock_true_speed_us=Song.clock_speed;

  actualRunningState.mainClock_duty=Song.clock_duty;

  FrequencyTimer2::setOnOverflow(0);
  FrequencyTimer2::setPeriod(actualRunningState.mainClock_true_speed_us);
  FrequencyTimer2::disable();
  
}

void  mainClock_update(uint32_t duration)
{
  actualRunningState.mainClock_true_speed_us = duration;
}

void  mainClock_restart()
{
  FrequencyTimer2::setOnOverflow(0);
  FrequencyTimer2::disable();
  
  //NVIC_DISABLE_IRQ(mainClock);
  mainClock_step = 0;
  //NVIC_ENABLE_IRQ(mainClock);

  float parsedClock= (float)actualRunningState.mainClock_speed_us * actualRunningState.clock_divider ;
  FrequencyTimer2::setPeriod((unsigned int) parsedClock);
  FrequencyTimer2::setOnOverflow(mainClock_interrupt);

}

void  mainClock_interrupt()
{
  //mainClock_tick = true;
  
  if ( mainClock_step > MAINCLOCK_MAX_STEP - 1 )mainClock_step = 0;

  //test
  //Serial.print("clock step:");Serial.println(mainClock_step); 
  
  //if ( mainClock_step % 10 == 0 )midiOut_clock_tick();
  
  if(actualRunningState.play && mainClock_step >= (MAINCLOCK_MAX_STEP / 5))*ptr_playled = CHSV( 200, 255, 255 );
  
  if ( mainClock_step == 0 )
  {
      sequencer_divClockOut(true);
      FrequencyTimer2::setPeriod(actualRunningState.mainClock_true_speed_us);
    
      
      if(actualRunningState.play)
      {

        sequencer_loadNextStep();
        
        if (Song.Pattern.A_first_step == actualRunningState.loaded_A_step)
        {
          sequencer_retrigOut(true);
          *ptr_playled = CHSV( 200, 0, 255 ); //short flash
        }
        
        sequencer_play_step();
      }
    
    *ptr_clockled = CHSV( 136, 32, 255 );
    //Serial.print("seq step:");Serial.println(actualRunningState.loaded_A_step);
  

  }

  if ( mainClock_step >= actualRunningState.mainClock_duty )
  {
    if(actualRunningState.play)sequencer_multClockOut(false);
    *ptr_clockled = CHSV( 136, 255, 64 );
  }

  if ( mainClock_step >= (MAINCLOCK_MAX_STEP / 2)  && actualRunningState.play)
  {   
    sequencer_divClockOut(false);
    sequencer_retrigOut(false);
  }

  mainClock_step += 1;
  
}

/*
void  mainClock_poll()
{
  uint16_t  local_mainClock_step_copy;
  local_mainClock_step_copy = mainClock_step;
  Serial.print("clk");Serial.print(local_mainClock_step_copy); Serial.println(" - "); Serial.flush();

  if ( local_mainClock_step_copy != actualRunningState.mainClock_step_copy )
  {
    //local_mainClock_step_copy = actualRunningState.mainClock_step_copy;
	  actualRunningState.mainClock_step_copy = local_mainClock_step_copy;
	
    //uint16_t  duty_mapped;
    //duty_mapped=(uint16_t)(actualRunningState.mainClock_duty);

    // tick midi out clock on every 100 (on 2400)
    if ( actualRunningState.mainClock_step_copy % 100 == 0 )midiOut_clock_tick();

    if ( actualRunningState.mainClock_step_copy == 0 )
    {
      //update steps /
      sequencer_divClockOut(true);

      sequencer_loadNextStep();
      if (Song.Pattern.A_first_step == actualRunningState.loaded_A_step)sequencer_retrigOut(true);
      sequencer_play_step();
      *ptr_clockled = CRGB( 0, 255, 127 );

    }

    if ( actualRunningState.mainClock_step_copy >= actualRunningState.mainClock_duty )
    {
      sequencer_multClockOut(false);
      *ptr_clockled = CRGB( 128, 255, 32 );
    }

    if ( actualRunningState.mainClock_step_copy >= MAINCLOCK_MAX_STEP / 2 )
    {
      sequencer_divClockOut(false);
      sequencer_retrigOut(false);

    }


  }

}
*/


/*
  uint8_t loadNextPattern()
  {
  uint8_t tmpNextPtn=1;

  tmpNextPtn=Song[0].Pattern[0].next_nb;

  if( tmpNextPtn == 0 ) //default next pattern ?
  {
    tmpNextPtn = actualRunningState.loaded_pattern + 1;

    if( tmpNextPtn > MAX_STEP - 1 ) //over pattern restart song ? stop ?
    {
      if( Song[0].loop_nb != 0 ) //not loop song forever
      {
         //so count
        actualRunningState.song_loop_count++;
        if( actualRunningState.song_loop_count > Song[0].loop_nb )//overflow loop stop song ; go first pattern
        {

          actualRunningState.song_loop_count=0;
          sequencer_stop();

        }

      }
      tmpNextPtn=1;
    }
  }

  actualRunningState.loaded_pattern=tmpNextPtn;
  actualRunningState.pattern_loop_count=0;
  Song[0].Pattern[0] = Song[0].Pattern[tmpNextPtn];
  //reload STEPs

  return tmpNextPtn;


  }
*/

//PATTERN *******************************************

void  pattern_set_first_analogStep( uint8_t sel )
{

  Song.Pattern.A_first_step = sel;
}

void  pattern_set_last_analogStep( uint8_t sel )
{

    if( sel == Song.Pattern.A_last_step )return;
    
    //Song.Pattern.A_last_step = sel;
    CRITICAL(Song.Pattern.A_last_step = sel);
    Serial.print("Set Last");Serial.println(sel); Serial.flush();
  
  
}

// SEQUENCER *******************************************


void  sequencer_loadNextStep()
{

  uint8_t nextPatternStep = 1;
  uint8_t actualStep = 0;

  //enum  patternABC{ stepA_l, stepB_l, stepC_l }

  uint8_t* ptr_first_step = &Song.Pattern.A_first_step;

  uint8_t*  ptr_last_step = &Song.Pattern.A_last_step;

  bool*   ptr_modePP_dir = &actualRunningState.modePP_dirA;

  uint8_t*  ptr_loaded_step = &actualRunningState.loaded_A_step;

  struct StepAnalog_s *ptr_commonStep = &Song.Pattern.StepA[0];

  struct StepAnalog_s (*ptr_nextStep)[17] = &Song.Pattern.StepA;

  enum seq_case_list { single16_l , pingpong16_l , rnd };
  
  uint8_t seq_case;

  actualStep = *ptr_loaded_step;

  switch (actualRunningState.sequence_mode) //// STEPABCx16, LOOPABCx16, SINGLEABCx16, LOOPABCx16PP, SINGLEABCx16PP,  STEPABx32Cx16, LOOPABx32Cx16, SINGLEABx32Cx16, LOOPABx32Cx16PP, SINGLEABx32Cx16PP };
  {
    case SINGLEAx16:
    case LOOPAx16:
      seq_case = single16_l;
      break;

    case SINGLEAx16PP :
    case LOOPAx16PP :
      seq_case = pingpong16_l;
      break;
      
    case RND :
      seq_case = rnd;
      break;

    default:
      seq_case = single16_l;
  }

  switch (seq_case) //// STEPABCx16, LOOPABCx16, SINGLEABCx16, LOOPABCx16PP, SINGLEABCx16PP,  STEPABx32Cx16, LOOPABx32Cx16, SINGLEABx32Cx16, LOOPABx32Cx16PP, SINGLEABx32Cx16PP };
  {

    // single16 + loop16
    case single16_l:
      if ( actualStep + 1 >= *ptr_last_step )
      {
        //stop && go start
        sequencer_loop_or_stop();

        //go first

        nextPatternStep = 1; //*ptr_first_step;

      } else {
        //go next
        nextPatternStep = actualStep + 1;
      }
      break;

    // ping/pong16 + loop16
    case pingpong16_l :
      //if forward
      if (  ! *ptr_modePP_dir  ) //forward
      {
        if ( actualStep + 1 >= *ptr_last_step ) //last
        {
          //go backward
          *ptr_modePP_dir = true;
          // was : nextPatternStep = (*ptr_last_step) - 1;
          nextPatternStep = actualStep - 1;

        } else {
          //go forward
          nextPatternStep = actualStep + 1;
        }
      } else { //backward
        if ( actualStep - 1 < *ptr_first_step ) //first
        {
          //stop? && go start
          sequencer_loop_or_stop();

          //go first
          
          //was : nextPatternStep = *ptr_first_step;
          nextPatternStep = actualStep + 1;
          if(actualRunningState.sequence_mode == SINGLEAx16PP)nextPatternStep = *ptr_first_step;
          //nextPatternStep = actualStep + 1;
          *ptr_modePP_dir = false;

        } else {
          //go previous
          nextPatternStep = actualStep - 1;
        }
      }
      break;

      case rnd :
        nextPatternStep= random(*ptr_last_step);
      break;


  }

  *ptr_loaded_step = nextPatternStep;
  *ptr_commonStep = *(ptr_nextStep)[nextPatternStep];

  //return  nextPatternStep;
  // }

}

void  sequencer_loop_or_stop()
{
  //if( actualRunningState.sequence_mode ==  ) //A is master of loop count; B,C,SYNTH,AUDIO always loop forever
  switch ( actualRunningState.sequence_mode ) // STEPABCx16, LOOPABCx16, SINGLEABCx16, LOOPABCx16PP, SINGLEABCx16PP,  STEPABx32Cx16, LOOPABx32Cx16, SINGLEABx32Cx16, LOOPABx32Cx16PP, SINGLEABx32Cx16PP };
  {
    case SINGLEAx16 :
    case SINGLEAx16PP :

      sequencer_stop();

    break;
  }

}

void  sequencer_stop()
{
  sequencer_multClockOut(false);
  
  //FrequencyTimer2::setOnOverflow(0);
  FrequencyTimer2::disable();
  actualRunningState.stop = true;
  actualRunningState.play = false;
  
  usbMIDI.sendRealTime(usbMIDI.Stop);
  MIDI.sendRealTime(midi::MidiType::Stop);
  
  *ptr_playled = CHSV(96, 192, 64 );
  return;
}

void  sequencer_restart()
{
  //mainClock_step = 0;
  Song.Pattern.StepA[0] = Song.Pattern.StepA[1]; //re-init step
  //actualRunningState.mainClock_step_copy=0;
  actualRunningState.loaded_A_step = 0;
  actualRunningState.modePP_dirA = false;
  actualRunningState.stop = false;
  actualRunningState.play = true;
  //

  mainClock_restart();
  //mainClock_init();
  //mainClock.begin(mainClock_interrupt, parsedClock );

  sequencer_retrigOut(true);
  sequencer_play_step();
  //usbMIDI.sendRealTime(usbMIDI.Start);
  //MIDI.sendRealTime(midi::MidiType::Start);
  *ptr_playled = CHSV( 200, 255, 255 );
  return;
}

void  sequencer_play_next_step()
{
  sequencer_multClockOut(false);
  delay(5);
  sequencer_loadNextStep();
  //sequencer_play_step();
  mux_A_OUT.setChannel( actualRunningState.loaded_A_step - 1 , true);
  sequencer_multClockOut(true);
}

void  sequencer_play_previous_step()
{
  sequencer_multClockOut(false);
  delay(5);
  
  if ( actualRunningState.loaded_A_step - 1 < Song.Pattern.A_first_step ) //first
  {

    
    actualRunningState.loaded_A_step = Song.Pattern.A_last_step -1;
    
  } else {
    //go previous
    actualRunningState.loaded_A_step -= 1;
    
  }
  
  Song.Pattern.StepA[0]=Song.Pattern.StepA[actualRunningState.loaded_A_step];
  
  mux_A_OUT.setChannel( actualRunningState.loaded_A_step - 1 , true);
  sequencer_multClockOut(true);
}

void  sequencer_play_step()
{

  //mux_A_OUT.setChannel( actualRunningState.loaded_A_step - 1 , true);
  
  if ( ! Song.Pattern.StepA[actualRunningState.loaded_A_step].muted )
  {
    mux_A_OUT.setChannel( actualRunningState.loaded_A_step - 1 , true);
  
    sequencer_multClockOut(true);
  }
  
}

void  sequencer_retrigOut(bool state)
{
  digitalWriteFast(RETRIG_OUT_PIN, Song.RETRIG_OUT_polarity ^ state);
  return;

}

void  sequencer_multClockOut(bool state)
{
  if(Song.MULT_TRIG_OUT_polarity)
  {
    if(state)
    {
      pinMode(MULT_TRIG_OUT_PIN, OUTPUT);
      digitalWriteFast(MULT_TRIG_OUT_PIN, false);
    }else{
      pinMode(MULT_TRIG_OUT_PIN, INPUT);
    }
  }else{
    digitalWriteFast(MULT_TRIG_OUT_PIN, state);
  }
  //was ::: digitalWriteFast(MULT_TRIG_OUT_PIN, Song.MULT_TRIG_OUT_polarity ^ state);
  return;

}

void  sequencer_divClockOut(bool state)
{
  digitalWriteFast(DIV_CLOCK_OUT_PIN, Song.DIV_CLOCK_OUT_polarity ^ state);
  return;

}

void  midiOut_clock_tick()
{
  //Serial1.write((char)MIDICLOCK);
  usbMIDI.sendRealTime(usbMIDI.Clock);
  MIDI.sendRealTime(midi::MidiType::Clock);
  return;
}


//POTs *******************************************

void  update_CLOCK_DIVIDER()
{
  //CLK DIVIDER
  float clock_divider_list[8] = { 0.25, 0.3333 , 0.5, 1.0, 1.5, 2.0, 3.0, 4.0 };
  int rawData= constrain(1023 - FUNCTION_POTS_values[clock_divider_l],1 , 1023);
  int mappedVal=map( rawData , 0, 1023, 0, 7);
  actualRunningState.clock_divider = clock_divider_list[ mappedVal ];

  //Serial.print("clockDiv:");Serial.print(actualRunningState.clock_divider); Serial.println(" - ");
  //Serial.print("pot:");Serial.print(FUNCTION_POTS_values[clock_divider_l]); Serial.println(" - ");
    
  //debug
 // actualRunningState.clock_divider = 1.0;
  
  mainClock_update( (float)actualRunningState.mainClock_speed_us * actualRunningState.clock_divider  );
 // Serial.print("clockDiv:");Serial.print(actualRunningState.clock_divider); Serial.println(" - ");
    
}

void  update_VOLTAGE_DIVIDER()
{
  //VOLTAGE DIVIDER
  //return; //debug
  actualRunningState.voltage_divider = 3 - map(FUNCTION_POTS_values[voltage_divider_l], 0, 1020, 0, 3);
  analogWrite( VOLTAGE_DIVIDER_OUTPUT_PIN, (65535 >> actualRunningState.voltage_divider) - 1 );
  //Serial.print("VoltPOT:");Serial.print(FUNCTION_POTS_values[voltage_divider_l]); Serial.println(" - ");
 
  //Serial.print("VoltDiv:");Serial.print((65535 >> actualRunningState.voltage_divider) - 1); Serial.println(" - ");
 
}

void  update_SPEED()
{
  if(CLOCK_IN_elapsed < CLOCKIN_TIMEOUT_uS )return;
  
  //SPEED
  //actualRunningState.mainClock_speed_us = map( 1023 - FUNCTION_POTS_values[speed_l], 0, 1023, 83 * 4, 1250 * 4  );
  actualRunningState.mainClock_speed_us = map( 1023 - FUNCTION_POTS_values[speed_l], 0, 1023, 200000 / MAINCLOCK_MAX_STEP, 3000000 / MAINCLOCK_MAX_STEP  );
  
  float parsedSpeed=(float)actualRunningState.mainClock_speed_us * actualRunningState.clock_divider;
  mainClock_update( parsedSpeed );
  
  //Serial.print("nativeSpeed:"); Serial.print(actualRunningState.mainClock_speed_us); Serial.print(" us - ") ; Serial.println(" - "); 
  //Serial.print("trueSpeed:"); Serial.print((uint32_t)parsedSpeed); Serial.print(" us - ") ; Serial.println(" - "); Serial.flush();

 // Serial.print(FUNCTION_POTS_values[speed_l]); Serial.println(" - ");
  
 
}

void  update_DUTY()
{
  //DUTY
  //return; //debug
  uint16_t duty = (uint16_t)mapf(FUNCTION_POTS_values[duty_cycle_l], 0, 1023, 1, ( MAINCLOCK_MAX_STEP  ) );
  CRITICAL(actualRunningState.mainClock_duty = duty);
  //actualRunningState.mainClock_duty = duty;
  //Serial.print("Duty:");Serial.print(actualRunningState.mainClock_duty); Serial.println(" - ");


}

void  update_SEQUENCE_MODE()
{
  //SEQUENCE_MODE
  actualRunningState.sequence_mode = constrain(mapf(FUNCTION_POTS_values[sequence_mode_l], 0, 1023, 0, 5),0 , 4);
  
  if ( actualRunningState.sequence_mode < SINGLEAx16PP)actualRunningState.modePP_dirA = false;

  //Serial.print("Seq:");Serial.print(actualRunningState.sequence_mode); Serial.println(" - ");
  //Serial.print("Pot_Seq:");Serial.print(FUNCTION_POTS_values[sequence_mode_l]); Serial.println(" - "); Serial.flush();

}

void  update_PORTAMENTO()
{
  return; //debug
  actualRunningState.portamento = mapf(FUNCTION_POTS_values[portamento_l], 0.0, 1023.0, 0.7, 1.0);
}

void  update_functionsPots()
{
  update_functionsPots(false);
}

void  update_functionsPots(bool force)
{
  int16_t rawAdc = 0;
  int16_t analogVal = 0;
  
  if ( ! force && actualRunningState.last_POT_refresh < POT_REFRESH_MS )return;
  
  actualRunningState.last_POT_refresh = 0;
  
  for (uint8_t potnb = 0; potnb < NUM_POTS; potnb++)
  {
    //rawAdc= (uint16_t)adc->analogRead(FUNCTION_POTS_PINS[potnb]);
    //Serial.print( "Analog read: "); Serial.println( potnb ); Serial.flush();
    rawAdc=1023 - analogRead(FUNCTION_POTS_PINS[potnb]);
    
    analogVal = getLPF_16( &(FUNCTION_POTS_lpf[potnb]) , rawAdc, FUNCTION_POTS_LPF_COEF);

    //Serial.print("potnb:");Serial.print(potnb);Serial.print(" val:");Serial.print( analogVal ); Serial.print(" - ");
    
    if ( ( analogVal >> 2 ) != ( FUNCTION_POTS_values[potnb] >> 2 ) || force)
    {     
      FUNCTION_POTS_values[potnb] = analogVal;
      //Serial.print( "POT: "); Serial.print( potnb ); Serial.print( "  *** new analog Val: "); Serial.println( analogVal ); Serial.flush();
  
      functionPotsAction[potnb]();
      //return;
    }
  }
  //Serial.println( "." ); Serial.flush();

}

void  init_ADC_functionsPots()
{
  analogReadAveraging(96);
 
  for (uint8_t potnb = 0; potnb < NUM_POTS; potnb++)
  {
     pinMode(FUNCTION_POTS_PINS[potnb],INPUT);
  }

  elapsedMillis updatePotTime;

  while( updatePotTime < 500 )
  {
    update_functionsPots(true);
  }
}
  

//HW INs OUTs *******************************************
void init_JACKandBUTT()
{
  
  // init pin and buttons
  for (int i = 0; i < NUM_FCT_BUTT; i++)
  {
    // Setup buttons with an internal pull-up :
    pinMode(FCT_BUTT_PINS[i],INPUT_PULLUP);
    
    
    // After setting up the button, setup the Bounce instance :
    fct_buttons[i].attach(FCT_BUTT_PINS[i],INPUT_PULLUP);
    fct_buttons[i].interval(BOUNCE_MS); // bounce in ms
  }

  for (int i = 0; i < NUM_JACK_IN; i++)
  {
    // Setup buttons with an internal pull-up :
    pinMode(JACK_IN_PINS[i], INPUT_PULLUP);
    
    // After setting up the button, setup the Bounce instance :
    jack_in[i].attach(JACK_IN_PINS[i], INPUT_PULLUP);
    jack_in[i].interval(1); // really short bounce in ms
  }

  for (int i = 0; i < NUM_JACK_OUT; i++)
  {
    // Setup buttons with an internal pull-up :
    pinMode(JACK_OUT_PINS[i], OUTPUT);
  }


}

// + FCT BUTTs *******************************************
void update_fct_buttons()
{
  if ( actualRunningState.last_FCT_BUTT_refresh < FCT_BUTT_REFRESH_MS  )return;
  
  actualRunningState.last_FCT_BUTT_refresh = 0 ;
  //Serial.println( "try check fct butts: "); Serial.flush();
  
  for (int i = 0; i < NUM_FCT_BUTT; i++)
  {
    //Serial.print( " check butt: ");Serial.println(i); Serial.flush();

    fct_buttons[i].update();
    
    if ( fct_buttons[i].fell() )
    {
      if( i == 2 && ! fct_buttons[1].read() ) //next step + stop
      {
        sequencer_play_previous_step();
      }else{
        fctButtonFellAction[i]();
      }
      //Serial.print(i);Serial.println( " fct activ ******************"); Serial.flush();
    }

    
  }
}

void update_jacks_in()
{
    //return; //debug
    if ( actualRunningState.last_JACK_IN_refresh < JACK_IN_REFRESH_MS )return;
    
    actualRunningState.last_JACK_IN_refresh = 0 ;
    
    for (int i = 0; i < NUM_JACK_IN; i++)
    {
      //jack_in[i].update(mcpJACKandBUTT.digitalRead(JACK_IN_PINS[i]));
      jack_in[i].update();
      
      if ( jack_in[i].fell() )
      {
        jack_inputFellAction[i]();
        Serial.print( " jack trigg : "); Serial.println(i); Serial.flush();
      }
      
    }
}

void  trig_CLOCK_IN()
{
  //return; //debug
  if ( CLOCK_IN_elapsed < CLOCKIN_TIMEOUT_uS )
  {
     //actualRunningState.disable_pot_speed=true;
     actualRunningState.mainClock_speed_us=(uint32_t)( (float)CLOCK_IN_elapsed  / (float)MAINCLOCK_MAX_STEP );

     //mainClock.begin( mainClock_interrupt, actualRunningState.mainClock_speed_us );
     mainClock_update(actualRunningState.mainClock_speed_us * actualRunningState.clock_divider );
    //mainClock.begin( mainClock_interrupt, (uint32_t)(( (float)CLOCK_IN_elapsed * actualRunningState.clock_divider ) / (float)MAINCLOCK_MAX_STEP ) );
  }
  CLOCK_IN_elapsed = 0;
}


// STEP BUTTs *******************************************
void initI2C_GPIO_STEP_BUTTONS()
{
  
  mcpSTEP_BUTT.begin(GPIO_STEP_BUTTONS_ADDR);
  //mcpSTEP_BUTT.init();


  // init pin and buttons
  for (int i = 0; i < (MAX_STEP - 1); i++)
  {
    // Setup buttons with an internal pull-up :
    mcpSTEP_BUTT.pinMode(i, INPUT);
    //mcpSTEP_BUTT.pullUp(i, HIGH);
    // After setting up the button, setup the Bounce instance :
    delay(50); //setup pullup
    step_buttons[i].attach(true);
    step_buttons[i].interval(BOUNCE_MS); // bounce in ms
    step_buttons_latch[i]=false;
    step_buttons[i].update(true); 
  }

  elapsedMillis initStepButtons;

  while( initStepButtons < 250 )
  {
    for (int i = 0; i < (MAX_STEP - 1); i++)
    {
        step_buttons[i].update(true); 
    }
    
  }

  for (int i = 0; i < (MAX_STEP - 1); i++)
  {
       Song.Pattern.StepA[i + 1].muted = false;
  }
    
  
}

void update_step_buttons()
{
  if ( actualRunningState.last_STEP_BUTT_refresh < STEP_BUTT_REFRESH_MS )return;
  actualRunningState.last_STEP_BUTT_refresh = 0 ;

  uint16_t allBUTTsToSwap = mcpSTEP_BUTT.readGPIOAB(); //
  //uint16_t allBUTTsToSwap = mcpSTEP_BUTT.read(); 
  uint16_t allBUTTs = (allBUTTsToSwap << 8 ) + (allBUTTsToSwap >> 8); //

  
  for (int i = 0; i < (MAX_STEP - 1) ; i++)
  {
    step_buttons[i].update(bitRead(allBUTTs, i)); 
    
    if ( ! step_buttons[i].read() && ! step_buttons_latch[i] ) //set last
    {
      if( step_buttons[i].duration() >= STEP_BUTT_LONG_PRESS_MS && ! step_buttons_latch[i])
      {
        if (Song.Pattern.A_last_step == (i + 1))
        {
          pattern_set_last_analogStep( MAX_STEP );
        }else{
          pattern_set_last_analogStep(i + 1);
        }
        step_buttons_latch[i]=true;
      }
          
       //Serial.print(i);Serial.println( " butt activ"); Serial.flush();
    }
    else if ( step_buttons[i].rose() ) //set mute
    {
      if ( step_buttons[i].held() < STEP_BUTT_LONG_PRESS_MS )
      {
        Song.Pattern.StepA[i + 1].muted = ! Song.Pattern.StepA[i + 1].muted;
        if (actualRunningState.loaded_A_step == i + 1 && Song.Pattern.StepA[i + 1].muted )sequencer_multClockOut(false);
        //Serial.print(i);Serial.println( " butt short"); Serial.flush();
      }
	  step_buttons_latch[i]=false;
    }
  }


}

//LEDs *******************************************
void  initLEDs()
{
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  
  // set master brightness control
  FastLED.setBrightness(dim8_raw(LED_INTENSITY_by255));

  for( int i = 0; i < NUM_LEDS; i++ ) 
  { //9948
    fadeToBlackBy( leds, NUM_LEDS, 10);
    leds[i] = CHSV( (i * 16), 255, 127 );
    FastLED.show();
    delay(100); //
  }
  delay(500); //
  clear_LEDs();

  // set master brightness control
  //FastLED.setBrightness(127);
  
}

void  update_LEDs()
{
  if ( actualRunningState.last_LED_refresh >= ( 1000 / FRAMES_PER_SECOND ) )
  {
    actualRunningState.last_LED_refresh = 0;
    //Serial.print( "  ***  LEDs ****: "); Serial.flush();
    
    for ( int stepa = 0; stepa < (MAX_STEP - 1); stepa++)
    {
      if ( Song.Pattern.A_last_step == stepa + 1 )
      {
        leds[stepa] = CHSV( 64, 255, 127 ); //last_step
      }
      else if ( Song.Pattern.StepA[stepa + 1].muted )
      {
        leds[stepa] = CHSV( 255, 255, 56 ); //muted
      } else {
        leds[stepa] = CHSV( 96, 255, 64 ); //off
      }
    }

    leds[ actualRunningState.loaded_A_step - 1 ] += CHSV( 136, 255, 255 ); //step on
    if( ! actualRunningState.play)*ptr_playled = CHSV(96, 192, 64 );

    FastLED.setBrightness( dim8_raw(LED_INTENSITY_by255) );
    //FastLED.setBrightness( 255 );

    // send the 'leds' array out to the actual LED strip
    FastLED.show();

    // insert a delay to keep the framerate modest
    // FastLED.delay(1000/FRAMES_PER_SECOND);

    // do some periodic updates
    //EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
    //EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
  }

}

void clear_LEDs()
{
  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = 0;
  }
  FastLED.show();
}

//useful *******************************************
float getLPF( float *avgLPF , float rawData, float LPF_Coef)
{
  *avgLPF = *avgLPF - (LPF_Coef * ( *avgLPF - rawData));
  return float(*avgLPF);
}

uint16_t getLPF_16( float *avgLPF , uint16_t rawData16, float LPF_Coef)
{
  return (uint16_t)getLPF( avgLPF , (float) rawData16, LPF_Coef);
}

uint16_t with_portamento_effect()
{
  return getLPF( &portamento_LPF , (float)Song.Pattern.StepA[0].value, actualRunningState.portamento );
}

float mapf(float value, float istart, float istop, float ostart, float ostop)
{
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

int32_t map32(int32_t value, int32_t istart, int32_t istop, int32_t ostart, int32_t ostop)
{
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

//EEPROM *******************************************
void init_EEPROM()
{
    
  EEPROM_readAnything(0, configuration_EEPROM);
  for(uint8_t it=0; it < NUM_CONF_FLAGS; it++)
  {
    *(running_state_config_flags[it])=*(EEPROM_config_flags[it]);
  }
  actualRunningState.MIDI2CV_oct_H=configuration_EEPROM.MIDI2CV_oct_H;
  actualRunningState.MIDI2CV_oct_L=configuration_EEPROM.MIDI2CV_oct_L;
  actualRunningState.MIDI2CV_oct_First=configuration_EEPROM.MIDI2CV_oct_First;
  Serial.println("upload config from EEPROM done"); Serial.flush();
}

void save_config_EEPROM()
{ 
  //return;
  digitalWrite(LEDPIN, true);
  for(uint8_t it=0; it < NUM_CONF_FLAGS; it++)
  {
    *(EEPROM_config_flags[it])=*(running_state_config_flags[it]);
 
  }
  
  configuration_EEPROM.MIDI2CV_oct_H=actualRunningState.MIDI2CV_oct_H;
  configuration_EEPROM.MIDI2CV_oct_L=actualRunningState.MIDI2CV_oct_L;
  configuration_EEPROM.MIDI2CV_oct_First=actualRunningState.MIDI2CV_oct_First;
  
  EEPROM_writeAnything(0, configuration_EEPROM);
  Serial.println("saved in EEPROM done"); Serial.flush();
  digitalWrite(LEDPIN, false);
}

void set_config_at_boot()
{
  //
  uint16_t allBUTTs;
  uint16_t allBUTTsToSwap;
  uint8_t flag;
  bool   flagSave=false;
  
  elapsedMillis bounce_init;

  clear_LEDs();
  
  while( bounce_init < 500 ) // in ms
  {
    fct_buttons[0].update();
  }
  
  while( ! fct_buttons[0].read() )
  {
    fct_buttons[0].update();
    
    allBUTTsToSwap = mcpSTEP_BUTT.readGPIOAB(); //
    allBUTTs = (allBUTTsToSwap << 8 ) + (allBUTTsToSwap >> 8); //
  
    for(uint8_t it=0 ; it < NUM_CONF_FLAGS; it++)
    {
      step_buttons[it].update( bitRead(allBUTTs, it) );
           
      if ( step_buttons[it].fell() )
      {
        flagSave=true;
        *(running_state_config_flags[it])=  ! *(running_state_config_flags[it]);
      }
      
        
      if( *(running_state_config_flags[it]) )
      {
        leds[it]= CRGB( 255, 0, 0 );
      }else{
        leds[it]= CRGB( 0, 255, 0 );
      }
 
    }

    
    //tuning midi to cv
    leds[8] = CHSV( 24, 255, 255 );
    leds[9] = CHSV( 30, 255, 255 );
    leds[10] = CHSV( 36, 255, 255 );
    leds[11] = CHSV( 42, 255, 255 );

    step_buttons[8].update(bitRead(allBUTTs, 8));
    step_buttons[9].update(bitRead(allBUTTs, 9));
    step_buttons[10].update(bitRead(allBUTTs, 10));
    step_buttons[11].update(bitRead(allBUTTs, 11));

     if( ! step_buttons[8].read() )
    {
       uint8_t  oct_max;
       oct_max=map(analogRead( SPEED_PIN ),0,1023,1,10);
       actualRunningState.MIDI2CV_oct_Max=oct_max;
       leds[oct_max] += CHSV( 192, 255, 255 );
       flagSave=true;
    }

    
    
    if( ! step_buttons[9].read() )
    {
      sequencer_multClockOut(false);
      actualRunningState.MIDI2CV_oct_L = ((( analogRead( SPEED_PIN ) >> 2 ) << 8 ) + ( analogRead( DUTY_CYCLE_PIN ) >> 2 ));
      analogWrite(VOLTAGE_DIVIDER_OUTPUT_PIN, actualRunningState.MIDI2CV_oct_L );
      mux_A_OUT.setChannel( 0 , true);
  
      Serial.println(actualRunningState.MIDI2CV_oct_L); Serial.flush();
      leds[9] = CHSV( 192, 255, 255 );
      flagSave=true;
      sequencer_multClockOut(true);
    }
    else if(! step_buttons[10].read())
    {
      sequencer_multClockOut(false);
      actualRunningState.MIDI2CV_oct_H = ((( analogRead( SPEED_PIN ) >> 2 ) << 8 ) + ( analogRead( DUTY_CYCLE_PIN ) >> 2 ));
      analogWrite(VOLTAGE_DIVIDER_OUTPUT_PIN, actualRunningState.MIDI2CV_oct_H );
      mux_A_OUT.setChannel( 0 , true);
      Serial.println(actualRunningState.MIDI2CV_oct_H); Serial.flush();
      leds[10] = CHSV( 192, 255, 255 );
      flagSave=true;
      sequencer_multClockOut(true);
    }
    else if( ! step_buttons[11].read() )
    {
      sequencer_multClockOut(false);
      actualRunningState.MIDI2CV_oct_First = ((( analogRead( SPEED_PIN ) >> 2 ) << 8 ) + ( analogRead( DUTY_CYCLE_PIN ) >> 2 ));
      analogWrite(VOLTAGE_DIVIDER_OUTPUT_PIN, actualRunningState.MIDI2CV_oct_First );
      mux_A_OUT.setChannel( 0 , true);
      Serial.println(actualRunningState.MIDI2CV_oct_First); Serial.flush();
      leds[11] = CHSV( 192, 255, 255 );
      flagSave=true;
      sequencer_multClockOut(true);
    }

    
    FastLED.show(); 
    //delay(50); //
  }
  

  if(flagSave)
  {
    save_config_EEPROM();
    
    for ( int i = NUM_LEDS - 1; i >= 0; i--)
    {
      leds[i] = CHSV( 255 - (16 * i) , 255, 127 );
      FastLED.show(); 
      delay(50); //
    }
  }

  clear_LEDs();
    
}

//MIDI to CV *******************************************
void initMIDISerial()
{
  // // initialize MIDI ***********************************************************
  // midi input is setup in MIDI.begin (pin 2 serialRX)
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  //MIDI.setHandlePitchBend(handlePitchBend);
//  MIDI.setHandleControlChange(handleControlChange);
  MIDI.setHandleClock(handleClock);
  MIDI.setHandleStart(handleStart);
  MIDI.setHandleStop(handleStop);//ahaha
  //MIDI.setHandleSystemReset(handleSystemReset);
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

//USB MIDI IN *******************************************
void initMIDIUsb()
{
  // initialize MIDI usb ***********************************************************
  //
  usbMIDI.setHandleNoteOn(handleNoteOn);
  usbMIDI.setHandleNoteOff(handleNoteOff);
  // MIDI.setHandleNoteOff(handleNoteOff);
  //MIDI.setHandlePitchBend(handlePitchBend);
 // usbMIDI.setHandleControlChange(handleControlChange);
  //MIDI.setHandleSystemReset(handleSystemReset);
  //MIDI.begin(MIDI_CHANNEL_OMNI);
 // usbMIDI.setHandleClock(handleClock);
  usbMIDI.setHandleStart(handleStart);
  //usbMIDI.setHandleContinue(handleContinue);
  usbMIDI.setHandleStop(handleStop);
}

void handleClock()
{
    if ( MIDICLOCK_IN_elapsed < CLOCKIN_TIMEOUT_uS )
    {
      actualRunningState.mainClock_speed_us=(uint32_t)(( (float)MIDICLOCK_IN_elapsed / (float)MAINCLOCK_MAX_STEP ));//* actualRunningState.clock_divider * 24.0 ) / (float)MAINCLOCK_MAX_STEP );
      
      //mainClock.begin( mainClock_interrupt, actualRunningState.mainClock_speed_us );
      float parsedSpeed=(float)actualRunningState.mainClock_speed_us * (float)actualRunningState.clock_divider;
      mainClock_update((uint16_t)parsedSpeed);
    }
    
    MIDICLOCK_IN_elapsed = 0;
}

void handleStart()
{
  sequencer_restart();
}

void handleStop()
{
  sequencer_stop();
}

void handleControlChange(byte channel, byte number, byte value)
{
  return;
}

void handleNoteOn(byte channel, byte number, byte value)
{
  if (value == 0)
  {
    sequencer_multClockOut(false);
  } else {
    //10v ; 10 octaves ; 1v by oct ; 65535 / 10 = 6553.5 ; 6553.5 steps by oct ( and by volts)
    uint16_t tunning_oct=actualRunningState.MIDI2CV_oct_H  - actualRunningState.MIDI2CV_oct_L ; //smthg near 6553.5
    uint16_t tunning_shift=actualRunningState.MIDI2CV_oct_First;
    float pwm_to_cv= ((float)( octave_base[number%12] + (trunc( number / 12 ) + 1) ) * (float)tunning_oct ) ;
    //analogWrite(MIDI_CV_OUTPUT_PIN, (uint16_t) pwm_to_cv + tunning_shift ); VOLTAGE_DIVIDER_OUTPUT_PIN
    analogWrite(VOLTAGE_DIVIDER_OUTPUT_PIN, (uint16_t) pwm_to_cv + tunning_shift );
    sequencer_multClockOut(true);
  }

}

void handleNoteOff(byte channel, byte number, byte value)
{
  analogWrite(MIDI_CV_OUTPUT_PIN, 0 );
  sequencer_multClockOut(false);
}


// setup ************************************************************************
void setup()
{
  pinMode(LEDPIN,OUTPUT);
  digitalWriteFast(LEDPIN, true);

  pinMode(VOLTAGE_DIVIDER_OUTPUT_PIN,OUTPUT);
  pinMode(MIDI_CV_OUTPUT_PIN,OUTPUT);
  
  //init DEBUG serial(usb)
  Serial.begin(115200); //DEBUG
  Serial.println("FT3000 SQ-16_lite"); Serial.flush();

  //  Set MIDI baud rate: *************************************
  Serial1.begin(31250);  // set midi baudrate
  Serial1.write(0xFF); // "is alive" midi msg

  analogWriteResolution(16);
  analogWriteFrequency(VOLTAGE_DIVIDER_OUTPUT_PIN, 915.527);
  analogWriteFrequency(MIDI_CV_OUTPUT_PIN, 915.527);

  //initMIDISerial();
  initMIDIUsb();

	//init HW
	init_JACKandBUTT();
	initI2C_GPIO_STEP_BUTTONS();
  init_ADC_functionsPots();
  
	initLEDs();

  //init RND
  randomSeed(analogRead(A20));
	
	//init SONG
	Song.resetSong();
	Song.Pattern.StepA[0]=Song.Pattern.StepA[1];

  init_EEPROM();

  set_config_at_boot(); //set config; keep play pressed while powering ON
  
  //init CLOCK
  mainClock_init();
  
  //update_functionsPots();
  clear_LEDs();

  Serial.println("Init. done"); Serial.flush();
  digitalWriteFast(LEDPIN, false);
 
  //mainClock_restart();
  sequencer_restart();
  sequencer_stop();
  
}

//  MAIN ************************************************************************
void loop()
{
  //mainClock_poll();
  
  update_jacks_in();

  update_fct_buttons();

  update_functionsPots();

  update_step_buttons();

  update_LEDs();

 // MIDI.read();

 usbMIDI.read();

}


//EOF
