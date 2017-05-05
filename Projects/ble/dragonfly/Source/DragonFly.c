/*******************************************************************************
  Filename:       DragonFly.c
  Revised:        $Date: 2017-04-1 11:43:49 -0700 (Tue, 19 May 2015) $
  Revision:       $Revision: 000001 $

  Description:    This file contains the Dragonfly sample application
                  for use with the TI Bluetooth Low Energy Protocol Stack.
*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_board.h"
#include "hal_drivers.h"
#include "hal_buzzer.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_keys.h"
#include "hal_i2c.h"

#include "gatt.h"
#include "hci.h"
#include "hci_tl.h" //oJo, añadido para hci over UART (añadida tambien libreria)
#include "hci_ext.h" //oJo, añadido para hci over UART (añadida tambien libreria)
   
// Applications
#include "DragonFly.h"
#include "DragonFly_Gate.h"
#include "DragonFly_Net_Core.h"
     
#if defined FEATURE_GATTINFO
  #include "gapgattserver.h"
#endif 
#include "gattservapp.h"

#include "peripheral.h"

#include "gapbondmgr.h"


// Hardware Services
#if defined FEATURE_BATTERY
  #include "battservice.h"       //SIG
#endif

#if defined HAL_KEY
//  #include "simplekeys.h"        //UUID:0xAA00
  #include "eventservice.h"        //UUID:0xAA00
#endif

//#include "ledservice.h"      //UUID:0xAA10 -> to testservice
//#include "buzzservice.h"     //UUID:0xAA20 -> to testservice

#if defined HAL_ACC
  #include "accservice.h"        //UUID:0xAA30
#endif

#if defined FEATURE_ITEMP
  #include "intempservice.h"     //UUID:0xAA40
#endif

//App Services
#include "ByteBuffer.h"   

#if defined FEATURE_DATALOGGER
  #include "dataloggerservice.h" //UUID:0xCC00
#endif

#if defined FEATURE_ALARM
  #include "alarmservice.h"      //UUID:0xCC20
#endif

#if defined FEATURE_TEST
#include "testservice.h"       //UUID:0xCC30
#endif

#ifdef FEATURE_REGISTER
  #include "registerservice.h" //UUID:0xCC40
#endif

// Sensor Services
#if defined FEATURE_DIGITAL 
  #include "digitalservice.h"     //UUID:0xBB00
#endif

#if defined FEATURE_ANALOG
  #include "analogservice.h"      //UUID:0xBB10
#endif

#if defined FEATURE_WEATHER 
  #include "weatherservice.h"  //UUID:0xBB20
#endif

#if defined FEATURE_LUXOMETER 
  #include "luxometerservice.h"   //UUID:0xBB30
#endif

#if defined FEATURE_AIR 
  #include "airservice.h"   //UUID:0xBB40
#endif

#if defined FEATURE_UVI 
  #include "uviservice.h"   //UUID:0xBB50
#endif

#if defined FEATURE_MAGNETOMETER
#include "magnetometerservice.h"       //UUID:0xBB60
#endif

//Originales, no tocar.
//#include "irtempservice.h"
//#include "accservice.h"
//#include "intempservice.h"
//#include "humidityservice.h"
//#include "magnetometerservice.h"
//#include "barometerservice.h"
//#include "gyroservice.h"

// Sensor drivers
#include "hal_sensor.h"

#include "hal_acc.h"
#include "hal_air.h"
#include "hal_pir.h"
#include "hal_weather.h"
#include "hal_lux.h"
#include "hal_uvi.h"
#include "hal_mag.h"

// Application
#if defined (SERIAL_INTERFACE)
#include "serialInterface.h"
#endif

#include "df_util.h" //Utils for DragonFly  
#include "DragonFly.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//oJo puestas para ejemplo HCI
#define RSP_PAYLOAD_IDX                  6
#define MAX_RSP_DATA_LEN                 50
#define MAX_RSP_BUF                      ( RSP_PAYLOAD_IDX + MAX_RSP_DATA_LEN )


#define BUFFER_SIZE                 1024
// How often to perform sensor reads (milliseconds)
//#define TEMP_DEFAULT_PERIOD                   1000
//#define HUM_DEFAULT_PERIOD                    1000
//#define BAR_DEFAULT_PERIOD                    1000
//#define MAG_DEFAULT_PERIOD                    2000
#define ACC_DEFAULT_PERIOD                       1000
#define INTEMP_DEFAULT_PERIOD                    1000

#if defined FEATURE_DATALOGGER
  #define DATALOGGER_DEFAULT_PERIOD              2000
  #define ALARM_DEFAULT_PERIOD                   2000
#endif

#if defined FEATURE_DIGITAL 
  #define DIGITAL_DEFAULT_PERIOD                 1000
#endif

#if defined FEATURE_ANALOG 
  #define ANALOG_DEFAULT_PERIOD                  2000
#endif

#if defined FEATURE_WEATHER 
  #define WEATHER_DEFAULT_PERIOD                 1000
#endif

#if defined FEATURE_LUXOMETER 
  #define LUXOMETER_DEFAULT_PERIOD               1000
#endif

#if defined FEATURE_UVI 
  #define UVI_DEFAULT_PERIOD                     1000
#endif

#if defined FEATURE_AIR 
  #define AIR_DEFAULT_PERIOD                     1000  
#endif

#if defined FEATURE_MAGNETOMETER
  #define MAGNETOMETER_DEFAULT_PERIOD            1000  
#endif

//#define GYRO_DEFAULT_PERIOD                    1000
#define PERIODIC_EVT_PERIOD                      250

// Constants for two-stage reading
#define TEMP_MEAS_DELAY                          275   // Conversion time 250 ms
//#define BAR_FSM_PERIOD                         80
#define ACC_FSM_PERIOD                           20

#if defined FEATURE_DIGITAL 
  #define DIGITAL_FSM_PERIOD                     20
#endif

#if defined FEATURE_ANALOG 
  #define ANALOG_FSM_PERIOD                      20
#endif

#if defined FEATURE_WEATHER 
  #define WEATHER_FSM_PERIOD                     20
#endif

#if defined FEATURE_LUXOMETER 
  #define LUXOMETER_FSM_PERIOD                   50
#endif

#if defined FEATURE_UVI 
  #define UVI_FSM_PERIOD                         50
#endif

#if defined FEATURE_AIR
  #define AIR_FSM_PERIOD                         20
#endif

//#define HUM_FSM_PERIOD                         20
//#define GYRO_STARTUP_TIME                      60    // Start-up time max. 50 ms


// Company Identifier: Libelium. (41) 0b00101001
#define LIBELIUM_COMPANY_ID                      0x0029

#define INVALID_CONNHANDLE                       0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                           15

//NOTA IMPORTANTE, por alguna razon cuando activamos BROADCASTER no funciona bien 
//el servicio Conection Control Service necesario para hacer OAD desde móviles

// How often to check battery voltage (in ms)
#define BATTERY_CHECK_PERIOD                     40000

// Test mode bit
#define TEST_MODE_ENABLE                         0x80

// System reset
#define ST_SYS_RESET_DELAY                       10000

/*********************************************************************
 * TYPEDEFS
 */

   
/*********************************************************************
 * GLOBAL VARIABLES
 */
#if defined HAL_KEY
  uint8 Actual_Keys = 0;
#endif

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern gaprole_States_t gapProfileState;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 Dragonfly_TaskID;   // Task ID for internal task/event processing

// Sensor State Variables
#if defined FEATURE_ACC
  static uint8  accConfig = DF_CFG_SENSOR_DISABLE;
#endif

#if defined FEATURE_ITEMP
  static uint8  intempConfig = DF_CFG_SENSOR_DISABLE;
#endif

#if defined FEATURE_DATALOGGER
  static uint8  dataloggerConfig = DF_CFG_SENSOR_DISABLE;
#endif
  
#if defined FEATURE_ALARM
  static uint8  alarmConfig = DF_CFG_SENSOR_DISABLE;
#endif
  
uint16 lenght_dummy=0;

#if defined FEATURE_DIGITAL 
  static uint8  digitalConfig = DF_CFG_SENSOR_DISABLE;
#endif
  
#if defined FEATURE_ANALOG 
  static uint8  analogConfig = DF_CFG_SENSOR_DISABLE;
#endif
  
#if defined FEATURE_WEATHER 
  static uint8  weatherConfig = DF_CFG_SENSOR_DISABLE;
#endif
  
#if defined FEATURE_LUXOMETER 
  static bool luxometerEnabled = FALSE;
  static uint8  luxometerState = HAL_LUX_MEAS_STATE_0;
#endif

#if defined FEATURE_UVI 
  static bool uviEnabled = FALSE;
  static uint8  uviState = HAL_UVI_MEAS_STATE_0;
#endif

#if defined FEATURE_AIR 
  static bool airEnabled = FALSE;
//  static uint8  airState = HAL_AIR_MEAS_STATE_0;
#endif
  
#if defined FEATURE_MAGNETOMETER
  static uint8  magnetometerConfig = DF_CFG_SENSOR_DISABLE;
#endif
  
//static bool   barEnabled = FALSE;
//static bool   humiEnabled = FALSE;
//static bool   gyroEnabled = FALSE;

//static bool   barBusy = FALSE;
//static uint8  humiState = 0;

static bool   sysResetRequest = FALSE;
//static bool   BuzzerComplete = FALSE;

//static uint16 sensorMagPeriod = MAG_DEFAULT_PERIOD;

#if defined FEATURE_ACC 
static uint16 sensorAccPeriod = ACC_DEFAULT_PERIOD;
#endif

#if defined FEATURE_ITEMP 
static uint16 sensorIntempPeriod = INTEMP_DEFAULT_PERIOD;
#endif

#if defined FEATURE_DATALOGGER
  static uint32 sensorDataloggerPeriod = DATALOGGER_DEFAULT_PERIOD;
#endif

#if defined FEATURE_ALARM
static uint16 sensorAlarmPeriod = ALARM_DEFAULT_PERIOD;
#endif 

#if defined FEATURE_DIGITAL 
  static uint16 sensorDigitalPeriod = DIGITAL_DEFAULT_PERIOD;
#endif
  
#if defined FEATURE_ANALOG 
  static uint32 sensorAnalogPeriod = ANALOG_DEFAULT_PERIOD;
#endif
  
#if defined FEATURE_WEATHER 
  static uint16 sensorWeatherPeriod = WEATHER_DEFAULT_PERIOD;
#endif
  
#if defined FEATURE_LUXOMETER 
  static uint16 sensorLuxometerPeriod = LUXOMETER_DEFAULT_PERIOD;
#endif

#if defined FEATURE_UVI 
  static uint16 sensorUViPeriod = UVI_DEFAULT_PERIOD;
#endif

#if defined FEATURE_AIR 
  static uint32 sensorAirPeriod = AIR_DEFAULT_PERIOD;
#endif

#if defined FEATURE_MAGNETOMETER
  static uint16 sensorMagnetometerPeriod = MAGNETOMETER_DEFAULT_PERIOD;
#endif
  
//static uint16 sensorTmpPeriod = TEMP_DEFAULT_PERIOD;
//static uint16 sensorHumPeriod = HUM_DEFAULT_PERIOD;
//static uint16 sensorBarPeriod = BAR_DEFAULT_PERIOD;
//static uint16 sensorGyrPeriod = GYRO_DEFAULT_PERIOD;

//static uint8  sensorGyroAxes = 0;
//static bool   sensorGyroUpdateAxes = FALSE;
static uint16 selfTestResult = 0;
static bool   testMode = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void dragonFly_ProcessOSALMsg( osal_event_hdr_t *pMsg );

//static void readIrTempData( void );
//static void readHumData( void );

#if defined FEATURE_ACC 
static void readAccData( void );
#endif

#if defined FEATURE_ITEMP 
static void readIntempData( void );
#endif

#if defined FEATURE_ALARM
static void readAlarmData( void );
#endif

#if defined FEATURE_DATALOGGER
static void readDataloggerData( void );
static void dumpDataloggerData( void );
static void putDataloggerData( void );
#endif

#if defined FEATURE_DIGITAL 
  static void readDigitalData( void );
#endif
  
#if defined FEATURE_ANALOG 
  static void readAnalogData( void );
#endif
  
#if defined FEATURE_WEATHER 
  static void readWeatherData( void );
#endif
  
#if defined FEATURE_LUXOMETER 
  static void readLuxometerData( void );
#endif

#if defined FEATURE_UVI 
  static void readUViData( void );
#endif
  
#if defined FEATURE_AIR 
  static void readAirData( void );
#endif

#if defined FEATURE_MAGNETOMETER
  static void readMagnetometerData( void );
#endif
  
//static void readMagData( void );
//static void readBarData( void );
//static void readBarCalibration( void );
//static void readGyroData( void );

//static void barometerChangeCB( uint8 paramID );
//static void irTempChangeCB( uint8 paramID );

#if defined FEATURE_ACC
  static void accelChangeCB( uint8 paramID );
#endif

#if defined FEATURE_ITEMP 
  static void intempChangeCB( uint8 paramID );
#endif
  
#if defined FEATURE_DATALOGGER
  static void dataloggerChangeCB( uint8 paramID );
#endif

#if defined FEATURE_ALARM  
static void alarmChangeCB( uint8 paramID );
#endif

#if defined FEATURE_DIGITAL 
  static void digitalChangeCB( uint8 paramID );
#endif
  
#if defined FEATURE_ANALOG 
  static void analogChangeCB( uint8 paramID );
#endif
  
#if defined FEATURE_WEATHER 
  static void weatherChangeCB( uint8 paramID );
#endif
  
#if defined FEATURE_LUXOMETER 
  static void luxometerChangeCB( uint8 paramID );
#endif

#if defined FEATURE_UVI 
  static void uviChangeCB( uint8 paramID );
#endif

#if defined FEATURE_AIR 
  static void airChangeCB( uint8 paramID );
#endif
  
//static void humidityChangeCB( uint8 paramID);
//static void magnetometerChangeCB( uint8 paramID );
//static void gyroChangeCB( uint8 paramID );

#if defined FEATURE_TEST
static void testChangeCB( uint8 paramID );
#endif

#if defined FEATURE_MAGNETOMETER
static void magnetometerChangeCB( uint8 paramID );
#endif

#if defined HAL_KEY
static void dragonFly_HandleKeys( uint8 shift, uint8 keys );
#endif

static void resetCharacteristicValue( uint16 servID, uint8 paramID, uint32 value, 
                                     uint8 paramLen );
static void resetCharacteristicValues( void );

static void performPeriodicTask( void );

// HAL Related Callback functions
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
  extern void BuzzerRing( uint16 timeout, uint8 tone );
  extern void BuzzerCompleteCB( void );
#endif


/*********************************************************************
 * PROFILE CALLBACKS
 */
#if defined FEATURE_ACC
static sensorCBs_t dragonFly_AccelCBs =
{
  accelChangeCB,            // Characteristic value change callback
};
#endif

#if defined FEATURE_ITEMP
static sensorCBs_t dragonFly_IntempCBs =
{
  intempChangeCB,           // Characteristic value change callback
};
#endif

#if defined FEATURE_DATALOGGER
static sensorCBs_t dragonFly_DataloggerCBs =
{
  dataloggerChangeCB,           // Characteristic value change callback
};
#endif

#if defined FEATURE_ALARM
static sensorCBs_t dragonFly_AlarmCBs =
{
  alarmChangeCB,           // Characteristic value change callback
};
#endif

#if defined FEATURE_DIGITAL 
static sensorCBs_t dragonFly_DigitalCBs =
{
  digitalChangeCB,          // Characteristic value change callback
};
#endif

#if defined FEATURE_ANALOG 
static sensorCBs_t dragonFly_AnalogCBs =
{
  analogChangeCB,           // Characteristic value change callback
};
#endif

#if defined FEATURE_WEATHER 
static sensorCBs_t dragonFly_WeatherCBs =
{
  weatherChangeCB,       // Characteristic value change callback
};
#endif

#if defined FEATURE_LUXOMETER 
static sensorCBs_t dragonFly_LuxometerCBs =
{
  luxometerChangeCB,        // Characteristic value change callback
};
#endif

#if defined FEATURE_UVI 
static sensorCBs_t dragonFly_UViCBs =
{
  uviChangeCB,        // Characteristic value change callback
};
#endif

#if defined FEATURE_AIR 
static sensorCBs_t dragonFly_AirCBs =
{
  airChangeCB,        // Characteristic value change callback
};
#endif

#if defined FEATURE_MAGNETOMETER
static sensorCBs_t dragonFly_MagnetometerCBs =
{
  magnetometerChangeCB,             // Charactersitic value change callback
};
#endif

/*
static sensorCBs_t dragonFly_HumidCBs =
{
  humidityChangeCB,         // Characteristic value change callback
};
*/
/*
static sensorCBs_t dragonFly_MagnetometerCBs =
{
  magnetometerChangeCB,     // Characteristic value change callback
};
*/
/*
static sensorCBs_t dragonFly_GyroCBs =
{
  gyroChangeCB,             // Characteristic value change callback
};
*/

#if defined FEATURE_TEST
static testCBs_t dragonFly_TestCBs =
{
  testChangeCB,             // Charactersitic value change callback
};
#endif


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      DragonFly_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void DragonFly_Init( uint8 task_id )
{
  buffer_init(BUFFER_SIZE);
  buffer_fill();
  Dragonfly_TaskID = task_id;
    
  // Add services
#if defined FEATURE_GATTINFO
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
#endif
  
#if defined FEATURE_ATTRIBUTE 
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
#endif
  
#if defined FEATURE_DEVINFO
  DevInfo_AddService();                           // Device Information Service
#endif
  
#if defined FEATURE_BATTERY
  // Battery Service
  Batt_AddService();
#endif
  
#if defined HAL_ADC
  // Initialize the ADC for battery reads
  HalAdcInit();
#endif
  
#if defined FEATURE_ACC
  Accel_AddService();                               // Accelerometer Service
#endif 

#if defined FEATURE_ITEMP
  Intemp_AddService();                              // Internal Temperature Service
#endif
  
#if defined FEATURE_DATALOGGER
  Datalogger_AddService();                          // Datalogger Service
#endif

#if defined FEATURE_ALARM  
  Alarm_AddService();                               // Alarm Service
#endif  

#if defined FEATURE_DIGITAL 
  Digital_AddService();                             // Digital Service
#endif
  
#if defined FEATURE_ANALOG 
  Analog_AddService();                              // Analog Service
#endif
  
#if defined FEATURE_WEATHER 
  Weather_AddService();                             // Weather Service
#endif
  
#if defined FEATURE_LUXOMETER 
  Luxometer_AddService();                           // Luxometer Service
#endif

#if defined FEATURE_UVI 
  UVi_AddService();                                 // Sun UV index Service
#endif
  
#if defined FEATURE_AIR 
  Air_AddService();                                 // Air Service
#endif

//  IRTemp_AddService();                            // IR Temperature Service
//  Humidity_AddService();                          // Humidity Service
//  Magnetometer_AddService();                      // Magnetometer Service
//  Barometer_AddService();                         // Barometer Service
//  Gyro_AddService();                              // Gyro Service
#if defined FEATURE_KEY
//  SK_AddService( GATT_ALL_SERVICES );             // Simple Keys Profile
  Event_AddService();                               // Events service
#endif
  
#ifdef FEATURE_REGISTER
  Register_addService();                            // Generic register access
#endif

#if defined FEATURE_MAGNETOMETER
  Magnetometer_AddService();                         // Magnetometer Profile
#endif

#if defined FEATURE_TEST
  Test_AddService();                              // Test Profile
#endif
  
  // Setup the Seensor Profile Characteristic Values
  resetCharacteristicValues();

  // Register for all key events - This app will handle all key events
#if defined HAL_KEY
  RegisterForKeys( Dragonfly_TaskID );
#endif

  // Initialise sensor drivers in HalDriverInit() in hal_drivers.c
  // makes sure LEDs are off
//  HalLedSet( (HAL_LED_1 | HAL_LED_2 |  HAL_LED_3), HAL_LED_MODE_OFF );
//  HALIRTempInit();
//  HalHumiInit();
//  HalMagInit();
//  HalAccInit(); //Same for Accelerometer and Internal Temperature
//  HalBarInit();
//  HalGyroInit();

  // Register callbacks with profile
//  VOID IRTemp_RegisterAppCBs( &dragonFly_IrTempCBs );
//  VOID Magnetometer_RegisterAppCBs( &dragonFly_MagnetometerCBs );
#if defined FEATURE_ACC
  VOID Accel_RegisterAppCBs( &dragonFly_AccelCBs );
#endif
  
#if defined FEATURE_ITEMP
  VOID Intemp_RegisterAppCBs( &dragonFly_IntempCBs );
#endif
  
#if defined FEATURE_DATALOGGER
  VOID Datalogger_RegisterAppCBs( &dragonFly_DataloggerCBs );
#endif
  
#if defined FEATURE_ALARM
  VOID Alarm_RegisterAppCBs( &dragonFly_AlarmCBs );
#endif
  
#if defined FEATURE_DIGITAL 
  VOID Digital_RegisterAppCBs( &dragonFly_DigitalCBs );
#endif
  
#if defined FEATURE_ANALOG 
  VOID Analog_RegisterAppCBs( &dragonFly_AnalogCBs );
#endif
  
#if defined FEATURE_WEATHER 
  VOID Weather_RegisterAppCBs( &dragonFly_WeatherCBs );
#endif
  
#if defined FEATURE_LUXOMETER 
  VOID Luxometer_RegisterAppCBs( &dragonFly_LuxometerCBs );
#endif

#if defined FEATURE_UVI 
  VOID UVi_RegisterAppCBs( &dragonFly_UViCBs );
#endif
  
#if defined FEATURE_AIR 
  VOID Air_RegisterAppCBs( &dragonFly_AirCBs );
#endif
  
//  VOID Humidity_RegisterAppCBs( &dragonFly_HumidCBs );
//  VOID Barometer_RegisterAppCBs( &dragonFly_BarometerCBs );
//  VOID Gyro_RegisterAppCBs( &dragonFly_GyroCBs );

#if defined FEATURE_MAGNETOMETER
  VOID Magnetometer_RegisterAppCBs( &dragonFly_MagnetometerCBs );
#endif

#if defined FEATURE_TEST  
  VOID Test_RegisterAppCBs( &dragonFly_TestCBs );
#endif

}


/*********************************************************************
 * @fn      DragonFly_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 DragonFly_ProcessEvent (uint8 task_id, uint16 events)
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if (events & SYS_EVENT_MSG)
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive (Dragonfly_TaskID)) != NULL)
    {
      dragonFly_ProcessOSALMsg ((osal_event_hdr_t *)pMsg);

      // Release the OSAL message
      VOID osal_msg_deallocate (pMsg);
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Handle system reset (long press on side key)
  if (events & DF_SYS_RESET_EVT)
  {
    if (sysResetRequest)
    {
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
      BuzzerRing (500, HAL_BUZZER_LOW_TONE); // short low 
      ST_HAL_DELAY(62500); //500ms
      
#endif
      HAL_SYSTEM_RESET();
    }
    return (events ^ DF_SYS_RESET_EVT);
  }

  if ( events & DF_SYS_STAR_DEVICE_EVT )
  {
    // First periodic Start Event
    osal_start_timerEx( Dragonfly_TaskID, DF_SYS_PERIODIC_EVT, PERIODIC_EVT_PERIOD );

    // Set timer for first battery read event
    osal_start_timerEx( Dragonfly_TaskID, DF_HAL_BATTERY_EVT, BATTERY_CHECK_PERIOD );
    
    return ( events ^ DF_SYS_STAR_DEVICE_EVT );
  }


#if defined FEATURE_BATTERY 
  ///////////////////////////
  //    BATTERY SERVICE    //
  ///////////////////////////
  if ( events & DF_HAL_BATTERY_EVT )
  {
    // Restart timer
    if ( BATTERY_CHECK_PERIOD )
    {
      osal_start_timerEx( Dragonfly_TaskID, DF_HAL_BATTERY_EVT, BATTERY_CHECK_PERIOD );
    }

    // perform battery level check
    Batt_MeasLevel( );

    return (events ^ DF_HAL_BATTERY_EVT);
  }
#endif


#if defined FEATURE_ACC 
  //////////////////////////
  //    Accelerometer     //
  //////////////////////////
  if ( events & DF_HAL_ACCELEROMETER_EVT )
  {
    if(accConfig != DF_CFG_SENSOR_DISABLE)
    {
      readAccData();
      osal_start_timerEx( Dragonfly_TaskID, DF_HAL_ACCELEROMETER_EVT, sensorAccPeriod );
    }
    else
    {
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_DATA, 0, ACCELEROMETER_DATA_LEN );
      VOID resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_HAL_ACCELEROMETER_EVT);
  }
#endif
  

#if defined FEATURE_ITEMP 
  /////////////////////////////////
  //    Internal Temperature     //
  /////////////////////////////////
  if ( events & DF_HAL_INTEMP_EVT )
  {
    if(intempConfig != DF_CFG_SENSOR_DISABLE)
    {
      readIntempData();
      osal_start_timerEx( Dragonfly_TaskID, DF_HAL_INTEMP_EVT, sensorIntempPeriod );
    }
    else
    {
      VOID resetCharacteristicValue( INTEMP_SERV_UUID, SENSOR_DATA, 0, INTEMP_DATA_LEN );
      VOID resetCharacteristicValue( INTEMP_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_HAL_INTEMP_EVT);
  }
#endif
  

#if defined FEATURE_DATALOGGER
  /////////////////////////////////
  //          Datalogger         //
  /////////////////////////////////
  if ( events & DF_APP_DATALOGGER_EVT )
  {
    if(dataloggerConfig == DF_CFG_DUMP) 
    {
      dumpDataloggerData();
    }
    else if(dataloggerConfig == DF_CFG_TEST)
    {
      readDataloggerData();
      osal_start_timerEx( Dragonfly_TaskID, DF_APP_DATALOGGER_EVT, sensorDataloggerPeriod );
    }
    else if(dataloggerConfig != DF_CFG_SENSOR_DISABLE)
    {
    }
    else
    {
      VOID resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_DATA, 0, DATALOGGER_DATA_LEN );
//      VOID resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
      VOID resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, DATALOGGER_CONF_LEN );
    }

    return (events ^ DF_APP_DATALOGGER_EVT);
  }
#endif
  

#if defined FEATURE_ALARM  
  /////////////////////////////////
  //            Alarm            //
  /////////////////////////////////
  if ( events & DF_APP_ALARM_EVT )
  {
    if(alarmConfig != DF_CFG_SENSOR_DISABLE)
    {
      readAlarmData();
      osal_start_timerEx( Dragonfly_TaskID, DF_APP_ALARM_EVT, sensorAlarmPeriod );
    }
    else
    {
      VOID resetCharacteristicValue( ALARM_SERV_UUID, SENSOR_DATA, 0, ALARM_DATA_LEN );
      VOID resetCharacteristicValue( ALARM_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_APP_ALARM_EVT);
  }
#endif
  

#if defined FEATURE_DIGITAL 
  //////////////////////////
  //        Digital       //
  //////////////////////////
  if ( events & DF_SEN_DIGITAL_EVT )
  {
    if (digitalConfig != DF_CFG_SENSOR_DISABLE)
    {
      //Mirar esto, primera lectura cada DF_SEN_DIGITAL_EVT y luego toma varias muestras con DIGITAL_FSM_PERIOD
//      HalHumiExecMeasurementStep(humiState);
//      if (digitalState == 2)
//      {
        readDigitalData();
//        digitalState = 0;
        osal_start_timerEx( Dragonfly_TaskID, DF_SEN_DIGITAL_EVT, sensorDigitalPeriod );
//      }
//      else
//      {
//        humiState++;
//        osal_start_timerEx( Dragonfly_TaskID, DF_SEN_DIGITAL_EVT, DIGITAL_FSM_PERIOD );
//      }
    }
    else
    {
      resetCharacteristicValue( DIGITAL_SERV_UUID, SENSOR_DATA, 0, DIGITAL_DATA_LEN);
      resetCharacteristicValue( DIGITAL_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_SEN_DIGITAL_EVT);
  }
#endif
  

#if defined FEATURE_ANALOG 
  //////////////////////////
  //        Analog        //
  //////////////////////////
  if ( events & DF_SEN_ANALOG_EVT )
  {
    if (analogConfig != DF_CFG_SENSOR_DISABLE)
    {
      //Mirar esto, primera lectura cada DF_SEN_ANALOG_EVT y luego toma varias muestras con ANALOG_FSM_PERIOD
//      HalHumiExecMeasurementStep(humiState);
//      if (digitalState == 2)
//      {
        readAnalogData();
//        putDataloggerData();
//        digitalState = 0;
        osal_start_timerEx( Dragonfly_TaskID, DF_SEN_ANALOG_EVT, sensorAnalogPeriod );
//      }
//      else
//      {
//        humiState++;
//        osal_start_timerEx( Dragonfly_TaskID, DF_SEN_ANALOG_EVT, ANALOG_FSM_PERIOD );
//      }
    }
    else
    {
      resetCharacteristicValue( ANALOG_SERV_UUID, SENSOR_DATA, 0, ANALOG_DATA_LEN);
      resetCharacteristicValue( ANALOG_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_SEN_ANALOG_EVT);
  }
#endif
  

#if defined FEATURE_WEATHER 
  //////////////////////////////
  //        Weather        //
  //////////////////////////////
  if ( events & DF_SEN_WEATHER_EVT )
  {
    if (weatherConfig != DF_CFG_SENSOR_DISABLE)
    {
      //Mirar esto, primera lectura cada DF_SEN_ANALOG_EVT y luego toma varias muestras con ANALOG_FSM_PERIOD
//      HalHumiExecMeasurementStep(humiState);
//      if (digitalState == 2)
//      {
        readWeatherData();
//        digitalState = 0;
        osal_start_timerEx( Dragonfly_TaskID, DF_SEN_WEATHER_EVT, sensorWeatherPeriod );
//      }
//      else
//      {
//        humiState++;
//        osal_start_timerEx( Dragonfly_TaskID, DF_SEN_ANALOG_EVT, ANALOG_FSM_PERIOD );
//      }
    }
    else
    {
      resetCharacteristicValue( WEATHER_SERV_UUID, SENSOR_DATA, 0, WEATHER_DATA_LEN);
      resetCharacteristicValue( WEATHER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_SEN_WEATHER_EVT);
  }
#endif
  

#if defined FEATURE_LUXOMETER 
  //////////////////////////////
  //        Luxometer         //
  //////////////////////////////
  if (events & DF_SEN_LUXOMETER_EVT)
  {
    if (luxometerEnabled != DF_CFG_SENSOR_DISABLE)
    {
      //Mirar esto, primera lectura cada DF_SEN_ANALOG_EVT y luego toma varias muestras con ANALOG_FSM_PERIOD
      HalLuxExecMeasurementStep(luxometerState);
      if (luxometerState == HAL_LUX_MEAS_STATE_1)
      {
        readLuxometerData();
        luxometerState = HAL_LUX_MEAS_STATE_0;
        osal_start_timerEx(Dragonfly_TaskID, DF_SEN_LUXOMETER_EVT, sensorLuxometerPeriod);
      }
      else
      {
        luxometerState++;
        //Poner LUXOMETER_FSM_PERIOD en funcion de integration time
        osal_start_timerEx(Dragonfly_TaskID, DF_SEN_LUXOMETER_EVT, LUXOMETER_FSM_PERIOD);
      }
    }
    else
    {
      resetCharacteristicValue(LUXOMETER_SERV_UUID, SENSOR_DATA, 0, LUXOMETER_DATA_LEN);
      resetCharacteristicValue(LUXOMETER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof(uint8));
    }

    return (events ^ DF_SEN_LUXOMETER_EVT);
  }
#endif

  
#if defined FEATURE_UVI 
  //////////////////////////////
  //        UV Index          //
  //////////////////////////////
  if (events & DF_SEN_UVI_EVT)
  {
    if (uviEnabled != DF_CFG_SENSOR_DISABLE)
    {
      //oJo Mirar esto, primera lectura cada DF_SEN_ANALOG_EVT y luego toma varias muestras con ANALOG_FSM_PERIOD
      HalUViExecMeasurementStep(uviState);
      if (uviState == HAL_UVI_MEAS_STATE_1)
      {
        readUViData();
        uviState = HAL_UVI_MEAS_STATE_0;
        osal_start_timerEx(Dragonfly_TaskID, DF_SEN_UVI_EVT, sensorUViPeriod);
      }
      else
      {
        uviState++;
        //Poner UVI_FSM_PERIOD en funcion de integration time
        osal_start_timerEx(Dragonfly_TaskID, DF_SEN_UVI_EVT, UVI_FSM_PERIOD);
      }
    }
    else
    {
      resetCharacteristicValue(UVI_SERV_UUID, SENSOR_DATA, 0, UVI_DATA_LEN);
      resetCharacteristicValue(UVI_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof(uint8));
    }

    return (events ^ DF_SEN_UVI_EVT);
  }
#endif
  

#if defined FEATURE_AIR 
  //////////////////////////////
  //        Air Sensor        //
  //////////////////////////////
  if (events & DF_SEN_AIR_EVT)
  {
    if (airEnabled != DF_CFG_SENSOR_DISABLE)
    {
      //Mirar esto, primera lectura cada DF_SEN_ANALOG_EVT y luego toma varias muestras con ANALOG_FSM_PERIOD
        readAirData();
        osal_start_timerEx( Dragonfly_TaskID, DF_SEN_AIR_EVT, sensorAirPeriod );
    }
    else
    {
      resetCharacteristicValue( AIR_SERV_UUID, SENSOR_DATA, 0, AIR_DATA_LEN);
      resetCharacteristicValue( AIR_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_SEN_AIR_EVT);
  }
#endif
  

#if defined FEATURE_MAGNETOMETER
  //////////////////////////
  //     Magnetometer     //
  //////////////////////////
  if ( events & DF_SEN_MAGNETOMETER_EVT )
  {
    if (magnetometerConfig != DF_CFG_SENSOR_DISABLE)
    {
      readMagnetometerData();
      osal_start_timerEx( Dragonfly_TaskID, DF_SEN_MAGNETOMETER_EVT, sensorMagnetometerPeriod );
    }
    else
    {
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_DATA, 0, MAGNETOMETER_DATA_LEN);
      resetCharacteristicValue( MAGNETOMETER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ DF_SEN_MAGNETOMETER_EVT);
  }
#endif  

  //////////////////////////
  //    Periodic Task     //
  //////////////////////////
  //Periodic Task
  if ( events & DF_SYS_PERIODIC_EVT )
  {
    // Restart timer
    if ( PERIODIC_EVT_PERIOD )
    {
 //     osal_start_timerEx( Dragonfly_TaskID, ST_PERIODIC_EVT, PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
 //   performPeriodicTask();
    
      // Perform periodic application task
      if (gapProfileState == GAPROLE_CONNECTED)
      {
        performPeriodicTask();
      }

    return (events ^ DF_SYS_PERIODIC_EVT);
  }

  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      DragonFly_Test
 *
 * @brief   Run a self-test of the DragonFly
 *
 * @param   none
 *
 * @return  bitmask of error flags
 */
uint16 DragonFly_Test(void)
{
  selfTestResult = HalSensorTest();
  HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
#if defined FEATURE_TEST
  // Write the self-test result to the test service
  Test_SetParameter( TEST_DATA_ATTR, TEST_DATA_LEN, &selfTestResult);
#endif
  return selfTestResult;
}


/*********************************************************************
* Private functions
*/


/*********************************************************************
 * @fn      dragonFly_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void dragonFly_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
#if defined HAL_KEY
      dragonFly_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
#endif
      break;

    default:
      // do nothing
      break;
  }
}

#if defined HAL_KEY
/*********************************************************************
 * @fn      dragonFly_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void dragonFly_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;
  VOID shift;  // Intentionally unreferenced parameter

  // HAL_KEY_SW_1 Handled
  if (keys & HAL_KEY_SW_1)
  {
    // oJo, eliminar este tipo de reset
    // Reset the system if multiple accelerometer double taps pressed for more than 10 seconds
    sysResetRequest = TRUE;
    osal_start_timerEx( Dragonfly_TaskID, DF_SYS_RESET_EVT, ST_SYS_RESET_DELAY );

    if (!testMode ) 
    {      
#if defined(GAP_ROLE_CENTRAL) && defined(GAP_ROLE_PERIPHERAL)
      DragonFly_Net_Core_RoleMode(GAPROLE_SWITCH); //oJo, para pasar la activacion a NetCore
#endif
      // Normal mode for HAL_KEY_SW_1 -> INT1 Accelerometer 
      // If device is not in a connection, an accelerometer double tap interruption 
      // puts in advertesing stutus
      if ( gapProfileState != GAPROLE_CONNECTED )
      {
//->NetCore oJo, enviar a netcore
#if defined (GAP_ROLE_PERIPHERAL)
//        uint8 current_adv_enabled_status;
//        uint8 new_adv_enabled_status;
//
//        // Find the current GAP advertising status
//        GAPPeripheralRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
//
//        if( current_adv_enabled_status == FALSE )
//          new_adv_enabled_status = TRUE;
////        else
////          new_adv_enabled_status = FALSE;
//        
//        // Change the GAP advertisement status to opposite of current status
//        GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
#endif
      }
    }
    else
    {
      // Test mode & HAL_KEY_SW_1
      if ( keys & HAL_KEY_SW_1 )
      {
        SK_Keys |= SK_KEY_INT1;
        HalLedSet(HAL_LED_3,HAL_LED_MODE_ON);
      }
    }
  }

  // In All States & HAL_KEY_SW_1 drops
  if (!(keys & HAL_KEY_SW_1))
  {
    // Cancel system reset request
    sysResetRequest = FALSE;
  }
  
  // In All States & HAL_KEY_SW_3 PIR interruption    
  if ( keys & HAL_KEY_SW_3 )   
  {
    //oJo procesar aqui lo que hay que hacer cuando el PIR detecta movimiento
    //mirar en OnBorad las acciones que se realizan para este evento
    SK_Keys |= SK_KEY_AUX;
    HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
      
    //oJo, en realidad no es necesario inicializar el PIR, solo hay que r
    //econocer la interrupcion escribiendo un 0 en DIRECTLINK
#if (defined HAL_PIR) && (HAL_PIR == TRUE)
    HalPirInit();
#endif
    
  }
  else
    HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
    
  
  // In All States & HAL_KEY_SW_4 QI_charger connected    
  if ( keys & HAL_KEY_SW_4 )   
  {
    SK_Keys |= SK_KEY_QI;
    HalLedSet(HAL_LED_ALL, HAL_LED_MODE_ON);
  }  
  
  // In connected mode
  if ( gapProfileState == GAPROLE_CONNECTED )
  {
    // Connected mode & HAL_KEY_SW_1
    if ( keys & HAL_KEY_SW_1 )
    {
      SK_Keys |= SK_KEY_INT1;
//      HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
    }

    // Connected mode & HAL_KEY_SW_2
    if ( keys & HAL_KEY_SW_2 )   
    {
      SK_Keys |= SK_KEY_INT2;
      HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
    }

    // Connected & no keys activated
    if ( keys == 0 )   
    {
      HalLedSet(HAL_LED_ALL,HAL_LED_MODE_OFF);
    }
  }

    Actual_Keys = keys;
  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
//  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
  
    uint8 aData[EVENT_DATA_LEN];
    aData[0] = SK_Keys;
    Event_SetParameter( SENSOR_DATA, EVENT_DATA_LEN, aData);  
}
#endif

/*********************************************************************
 * @fn      resetSensorSetup
 *
 * @brief   Turn off all sensors that are on
 *
 * @param   none
 *
 * @return  none
 */
extern void resetSensorSetup (void)
{
/*
  if (HalIRTempStatus()!=TMP006_OFF || irTempEnabled)
  {
    HalIRTempTurnOff();
    irTempEnabled = FALSE;
  }
*/
#if defined FEATURE_ACC  
  if (accConfig != DF_CFG_SENSOR_DISABLE)
  {
    accConfig = DF_CFG_SENSOR_DISABLE;
  }
#endif
  
#if defined FEATURE_ITEMP
  if (intempConfig != DF_CFG_SENSOR_DISABLE)
  {
    intempConfig = DF_CFG_SENSOR_DISABLE;
  }
#endif
  
#if defined FEATURE_DATALOGGER 
  if (dataloggerConfig != DF_CFG_SENSOR_DISABLE)
  {
    //Keep datalogger alive
    //dataloggerConfig = DF_CFG_SENSOR_DISABLE;
  }
#endif
  
#if defined FEATURE_ALARM  
  if (alarmConfig != DF_CFG_SENSOR_DISABLE)
  {
    alarmConfig = DF_CFG_SENSOR_DISABLE;
  }
#endif

#if defined FEATURE_DIGITAL 
  if (digitalConfig != DF_CFG_SENSOR_DISABLE)
  {
    digitalConfig = DF_CFG_SENSOR_DISABLE;
  }
#endif
  
#if defined FEATURE_ANALOG 
  if (analogConfig != DF_CFG_SENSOR_DISABLE)
  {
    //Keep analog reads alive in wait state
//PONER AQUI CONDICION SI ESTA HABILITADO EN CONFIG
//    analogConfig = DF_CFG_SENSOR_DISABLE;
  }
#endif
  
#if defined FEATURE_WEATHER 
  if (weatherConfig != DF_CFG_SENSOR_DISABLE)
  {
    weatherConfig = DF_CFG_SENSOR_DISABLE;
  }
#endif
  
#if defined FEATURE_LUXOMETER 
  if (luxometerEnabled != DF_CFG_SENSOR_DISABLE)
  {
    luxometerEnabled = DF_CFG_SENSOR_DISABLE;
  }
#endif  

#if defined FEATURE_UVI 
  if (uviEnabled != DF_CFG_SENSOR_DISABLE)
  {
    uviEnabled = DF_CFG_SENSOR_DISABLE;
  }
#endif  
  
#if defined FEATURE_AIR 
  if (airEnabled != DF_CFG_SENSOR_DISABLE)
  {
    airEnabled = DF_CFG_SENSOR_DISABLE;
  }
#endif  

/*
  if (HalMagStatus()!=MAG3110_OFF || magEnabled)
  {
    HalMagTurnOff();
    magEnabled = FALSE;
  }
*/
/*
  if (gyroEnabled)
  {
    HalGyroTurnOff();
    gyroEnabled = FALSE;
  }
*/
/*
  if (barEnabled)
  {
    HalBarInit();
    barEnabled = FALSE;
  }
*/
/*
  if (humiEnabled)
  {
    HalHumiInit();
    humiEnabled = FALSE;
  }
*/
  // Reset internal states
//  sensorGyroAxes = 0;
//  sensorGyroUpdateAxes = FALSE;
  testMode = FALSE;

  // Reset all characteristics values
//  resetCharacteristicValues();
}



#if defined FEATURE_ACC 
/*********************************************************************
 * @fn      readAccData
 *
 * @brief   Read accelerometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readAccData(void)
{
  uint8 aData[ACCELEROMETER_DATA_LEN];
//  uint8 x;
//  aData[0]=0x41;
//  aData[1]=0x42;
//  aData[2]=0x43;

//  accReadAcc(&aData[0], &aData[1], &aData[2]);

//HalSensorReadReg( 0x03, &x, sizeof(x));  

//   aData[0] = x;
//  if (HalAccRead(aData))
//  if (HalAccID(aData))
//  if (HalAccTemp(aData))
  //if (HalAccReadX(aData))
  if (HalAccGetAll(aData))
//  if (HalAccOrientation(aData))
//aData[0]=P1;     //Puerto
//aData[1]=PICTL;  //Control falling rising
//aData[2]=P1IFG;  //Interruption Flag Generator
//aData[3]=P1IEN;  //Activacion interrupcion
//aData[4]=P1IF;   //Interrupcion en el puerto 1
////aData[3]=PICTL;
////aData[4]=P1INP;
//aData[5]=IEN2;
//aData[6]=P1SEL;

{
    Accel_SetParameter( SENSOR_DATA, ACCELEROMETER_DATA_LEN, aData);
  }
}
#endif


#if defined FEATURE_ITEMP
/*********************************************************************
 * @fn      readIntempData
 *
 * @brief   Read internal temperature data
 *
 * @param   none
 *
 * @return  none
 */
static void readIntempData(void)
{
  uint8 aData[INTEMP_DATA_LEN];

//  if (HalAccRead(aData))
  {
    Intemp_SetParameter( SENSOR_DATA, INTEMP_DATA_LEN, aData);
  }
}
#endif


#if defined FEATURE_DATALOGGER
/*********************************************************************
 * @fn      readDataloggerData
 *
 * @brief   Read datalogger data
 *
 * @param   none
 *
 * @return  none
 */
static void readDataloggerData(void)
{
  uint8 aData[DATALOGGER_DATA_LEN];
  uint16 size;//, capacity;


        HalLedSet(HAL_LED_3, HAL_LED_MODE_BLINK );

  
  uint32 systemClock = osal_GetSystemClock();
  
  size = buffer_getSize();
//  capacity = buffer_getCapacity();
  buffer_putInt(size);
  buffer_putLong(systemClock);
 
  aData[0] = HI_UINT16(size);//(size >> 8) & 0x00FF;
  aData[1] = LO_UINT16(size);//size & 0x00FF;
  
  aData[2] = BREAK_UINT32(systemClock, 3);
  aData[3] = BREAK_UINT32(systemClock, 2);
  aData[4] = BREAK_UINT32(systemClock, 1);
  aData[5] = BREAK_UINT32(systemClock, 0);

  for (int i=6; i < DATALOGGER_DATA_LEN; i++)
  {
    aData[i]= buffer_peek(size-i);
  }
  
//  if (HalAccRead(aData))
  {
    Datalogger_SetParameter(SENSOR_DATA, DATALOGGER_DATA_LEN, aData);
  }
}
#endif


#if defined FEATURE_DATALOGGER
/*********************************************************************
 * @fn      dumpDataloggerData
 *
 * @brief   Dump datalogger data
 *
 * @param   none
 *
 * @return  none
 */
static void dumpDataloggerData(void)
{
  uint8 aData[DATALOGGER_DATA_LEN];
  uint16 position;

  Datalogger_GetParameter( SENSOR_DATA, (uint8 *) &aData);

  position = BUILD_UINT16(aData[0], aData[1]);
  
  for (int i=2; i < DATALOGGER_DATA_LEN; i++)
  {
//    aData[i]= buffer_getFromBack();
//    aData[i]= buffer_get();
    aData[i]= buffer_peek(position-2+i);
  }
    Datalogger_SetParameter( SENSOR_DATA, DATALOGGER_DATA_LEN, aData);

}
#endif


#if defined FEATURE_DATALOGGER
/*********************************************************************
 * @fn      putDataloggerData
 *
 * @brief   Put datalogger data
 *
 * @param   none
 *
 * @return  none
 */
static void putDataloggerData(void)
{
  uint8 aData[DATALOGGER_DATA_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//  uint8 aPacket[6];//[ANALOG_DATA_PACKET];
  uint16 size, capacity;


  //Get system Clock, Size and Capacity
  uint32 systemClock = osal_GetSystemClock();
  size = buffer_getSize();
  capacity = buffer_getCapacity();
  
  //Get data from Analog Service
#if defined FEATURE_ANALOG
  Analog_GetParameter(SENSOR_DATA, (uint8 *) &aData);
#endif
  
  //Check buffer space before put packet
//  if ((buffer_getSize()+6) <= buffer_getCapacity())
  if ((size+6) <= capacity)
  {
    //Put Packet
    buffer_putLong(systemClock);
    buffer_putInt(BUILD_UINT16(aData[0], aData[1]));

    //Blink Blue Led
    HalLedSet(HAL_LED_3, HAL_LED_MODE_BLINK );
  }
  else 
    //Buffer Full, Blink Red Led
    HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
 
  size = buffer_getSize();
  //Actualize datalogger data characteristic
  aData[0] = LO_UINT16(size);//size & 0x00FF;
  aData[1] = HI_UINT16(size);//(size >> 8) & 0x00FF;
  
  for (int i=2; i < DATALOGGER_DATA_LEN; i++)
  {
    aData[i]= buffer_peek(size-DATALOGGER_DATA_LEN-2+i);
  }
  
//  if (HalAccRead(aData))
  {
    Datalogger_SetParameter( SENSOR_DATA, DATALOGGER_DATA_LEN, aData);
  }
}
#endif


#if defined FEATURE_ALARM
/*********************************************************************
 * @fn      readAlarmData
 *
 * @brief   Alarm data
 *
 * @param   none
 *
 * @return  none
 */
static void readAlarmData(void)
{
  uint8 aData[ALARM_DATA_LEN];

//  if (HalAccRead(aData))
  {
    Alarm_SetParameter( SENSOR_DATA, ALARM_DATA_LEN, aData);
  }
}
#endif


#if defined FEATURE_DIGITAL 
/*********************************************************************
 * @fn      readDigitalData
 *
 * @brief   Read digital sensor data
 *
 * @param   none
 *
 * @return  none
 */
static void readDigitalData(void)
{
  uint8 dData[DIGITAL_DATA_LEN];

//Añadir
//  if (HalDigitalRead(aData))
  {
    Digital_SetParameter( SENSOR_DATA, DIGITAL_DATA_LEN, dData);
  }
}
#endif


#if defined FEATURE_ANALOG 
/*********************************************************************
 * @fn      readAnalogData
 *
 * @brief   Read analog sensor data
 *
 * @param   none
 *
 * @return  none
 */
//static void readAnalogData(void)
//{
////    uint8* k;
////    k = (uint8*)osal_mem_alloc(ANALOG_DATA_LEN);
//  uint8 dData[ANALOG_DATA_LEN] = {0x55,0x55,0x55,0x55};
//
////  uint8 pData[ANALOG_PERI_LEN];
//  uint8 reg;
////  uint16 value;
//  
////  value=HalAdcRead(HAL_ADC_CHANNEL_0,HAL_ADC_RESOLUTION_12);
////  
////  dData[0]=value & 0x00FF; //LSB
////  dData[1]=(value>>=8)& 0x00FF; //MSB
////  dData[2]=4; 
////  dData[3]=4; 
//  Analog_GetParameter( SENSOR_CONF, (uint8 *) &reg);
//  
////  Analog_GetParameter( SENSOR_PERI, (uint8 *) &pData);
//
////  if (pData[0]!=0xFF) HalAirPutReg(reg, pData[0] );
////    HalAirGetID(dData);
////  HalAirGetReg( reg,(uint8 *) &dData);
////  HalAirGetData((uint8 *) &dData);
//  HalAirGetData(dData);
//  
//    //Añadir
////  if (HalAdcRead(0,12)) //CH0, 12bit resollution  
//  {
//    Analog_SetParameter( SENSOR_DATA, ANALOG_DATA_LEN, dData);
//  }
//}

static void readAnalogData(void)
{
  uint8 dData[ANALOG_DATA_LEN] = {0x55,0x55};

  uint8 reg;
  uint16 value;
  

  HalLedSet(HAL_LED_1,HAL_LED_MODE_BLINK);
  

  //Power on Plants and wait 1ms for stability (oJo- Cambiar esto para que no pare el flow)
#if defined(HAL_BOARD_DRAGONFLY_v8B_PLANTS)
  //if dragonfly plants, read A0_0
  PLANTS_PW_SBIT=1;
  ST_HAL_DELAY(125);

  value=HalAdcRead(PLANTS_DATA_SBIT,HAL_ADC_RESOLUTION_12);

  PLANTS_PW_SBIT=0;  
#else  
  #if defined(HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT)
    HalPirGetData16((uint8 *) &value);
  #else  
    //read battery level in A0_7
    HalAdcSetReference(HAL_ADC_REF_AVDD);
    value=HalAdcRead(HAL_ADC_CHANNEL_7,HAL_ADC_RESOLUTION_12);
  #endif
#endif
  

  dData[0]=value & 0x00FF; //LSB
  dData[1]=(value>>=8)& 0x00FF; //MSB
  
  {
    Analog_SetParameter( SENSOR_DATA, ANALOG_DATA_LEN, dData);
  }

  //If configuration register=2, then put data in datalogger.
  Analog_GetParameter( SENSOR_CONF, (uint8 *) &reg);

#if defined FEATURE_DATALOGGER
  if (reg==0x02) putDataloggerData();
#endif

}
#endif //defined FEATURE_ANALOG 


#if defined FEATURE_WEATHER 
/*********************************************************************
 * @fn      readWeatherData
 *
 * @brief   Read weather sensor data
 *
 * @param   none
 *
 * @return  none
 */
static void readWeatherData(void)
{
  uint8 dData[WEATHER_DATA_LEN] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
  uint8 cfg=0;

  HalWeatherReadData(cfg,(uint8 *) &dData);
  
//Añadir
//  if (HalDigitalRead(aData))
//  if (HalPirGetData16((uint8 *) &dData)) 
  {
    Weather_SetParameter( SENSOR_DATA, WEATHER_DATA_LEN, dData);
  }
}
#endif


#if defined FEATURE_LUXOMETER 
/*********************************************************************
 * @fn      readLuxometerData
 *
 * @brief   Read luxometer sensor data
 *
 * @param   none
 *
 * @return  none
 */
static void readLuxometerData(void)
{
  uint8 lData[LUXOMETER_DATA_LEN];

  if (HalLuxReadMeasurement(lData))
  {
    Luxometer_SetParameter( SENSOR_DATA, LUXOMETER_DATA_LEN, lData);
  }
}
#endif


#if defined FEATURE_UVI 
/*********************************************************************
 * @fn      readUViData
 *
 * @brief   Read UV index sensor data
 *
 * @param   none
 *
 * @return  none
 */
static void readUViData(void)
{
  uint8 lData[UVI_DATA_LEN];

  if (!HalUViReadMeasurement(lData) && !HalUVluxReadMeasurement(lData+2))
  {
    UVi_SetParameter(SENSOR_DATA, UVI_DATA_LEN, lData);
  }
}
#endif


#if defined FEATURE_AIR 
/*********************************************************************
 * @fn      readAirData
 *
 * @brief   Read air quallity sensor data
 *
 * @param   none
 *
 * @return  none
 */
static void readAirData(void)
{
  uint8 aData[AIR_DATA_LEN]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
  uint8 reg;

  Air_GetParameter(SENSOR_CONF, (uint8 *) &reg);
  if (HalAirGetReg(reg,(uint8 *) &aData))
//  if (HalAirGetData((uint8 *) &aData))
  {
    HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);

    Air_SetParameter( SENSOR_DATA, AIR_DATA_LEN, aData);
  }
  else
    HalLedSet(HAL_LED_2,HAL_LED_MODE_BLINK);
}
#endif


#if defined FEATURE_MAGNETOMETER
/*********************************************************************
 * @fn      readMagnetometerData
 *
 * @brief   Read magnetometer data
 *
 * @param   none
 *
 * @return  none
 */
void readMagnetometerData(void)
{
  uint8 mData[MAGNETOMETER_DATA_LEN];

  if (HalMagGetXYZ(mData))
  {
    Magnetometer_SetParameter(SENSOR_DATA, MAGNETOMETER_DATA_LEN, mData);
  }
}
#endif

#if defined FEATURE_ACC 
/*********************************************************************
 * @fn      accelChangeCB
 *
 * @brief   Callback from Acceleromter Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void accelChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Accel_GetParameter( SENSOR_CONF, &newValue );
      if ( newValue == DF_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (accConfig != DF_CFG_SENSOR_DISABLE)
        {
          accConfig = DF_CFG_SENSOR_DISABLE;
          osal_set_event( Dragonfly_TaskID, DF_HAL_ACCELEROMETER_EVT);
        }
      }
      else
      {
        if (accConfig == DF_CFG_SENSOR_DISABLE)
        {
          // Start scheduling only on change disabled -> enabled
          osal_set_event( Dragonfly_TaskID, DF_HAL_ACCELEROMETER_EVT);
        }
        // Scheduled already, so just change range
        accConfig = newValue;
        HalAccSetRange(accConfig);
      }
      break;

    case SENSOR_PERI:
      Accel_GetParameter( SENSOR_PERI, &newValue );
      sensorAccPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}
#endif

#if defined FEATURE_ITEMP
/*********************************************************************
 * @fn      intempChangeCB
 *
 * @brief   Callback from Internal Temperature Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void intempChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Intemp_GetParameter( SENSOR_CONF, &newValue );
      if ( newValue == DF_CFG_SENSOR_DISABLE)
      {
        // Put sensor to sleep
        if (intempConfig != DF_CFG_SENSOR_DISABLE)
        {
          intempConfig = DF_CFG_SENSOR_DISABLE;
          osal_set_event( Dragonfly_TaskID, DF_HAL_INTEMP_EVT);
        }
      }
      else
      {
        if (intempConfig == DF_CFG_SENSOR_DISABLE)
        {
          // Start scheduling only on change disabled -> enabled
//          intempConfig = DF_CFG_SENSOR_ENABLE;
          osal_set_event( Dragonfly_TaskID, DF_HAL_INTEMP_EVT);
        }
        // Scheduled already, so just change range
        intempConfig = newValue;
//        HalAccSetRange(accConfig);
      }
      break;

    case SENSOR_PERI:
      Intemp_GetParameter( SENSOR_PERI, &newValue );
      sensorIntempPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}
#endif

/*********************************************************************
 * @fn      dataloggerChangeCB
 *
 * @brief   Callback from Datalogger Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
#if defined FEATURE_DATALOGGER
static void dataloggerChangeCB( uint8 paramID )
{
  uint32 newPeriValue;
  uint16 newConfValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Datalogger_GetParameter( SENSOR_CONF, &newConfValue );
      if ( newConfValue == DF_CFG_DUMP)
      {
//        dumpDataloggerData();
        // Put datalogger to sleep
//        if (dataloggerConfig != DF_CFG_SENSOR_DISABLE)
//        {
          dataloggerConfig = newConfValue;
          osal_set_event( Dragonfly_TaskID, DF_APP_DATALOGGER_EVT);
//        }
      }
      else if(newConfValue == DF_CFG_CLEAN)
      {
        buffer_clear();
        buffer_fill();
      }
      else if ( newConfValue == DF_CFG_SENSOR_DISABLE)
      {
        // Put datalogger to sleep
        if (dataloggerConfig != DF_CFG_SENSOR_DISABLE)
        {
          dataloggerConfig = DF_CFG_SENSOR_DISABLE;
          osal_set_event( Dragonfly_TaskID, DF_APP_DATALOGGER_EVT);
        }
      }
      else
      {
        if (dataloggerConfig == DF_CFG_SENSOR_DISABLE)
        {
          // Start scheduling only on change disabled -> enabled
//          dataloggerConfig = DF_CFG_SENSOR_ENABLE;
          osal_set_event( Dragonfly_TaskID, DF_APP_DATALOGGER_EVT);
        }
        // Scheduled already, so just change range
        dataloggerConfig = newConfValue;
//        HalAccSetRange(accConfig);
      }
      break;
      
    case SENSOR_PERI:
      Datalogger_GetParameter( SENSOR_PERI, &newPeriValue );
      sensorDataloggerPeriod = newPeriValue*SENSOR_PERIOD_RESOLUTION;
      break;

    case SENSOR_DATA:
      //Generate event if in DUMP mode
        if (dataloggerConfig == DF_CFG_DUMP)
        {
          osal_set_event( Dragonfly_TaskID, DF_APP_DATALOGGER_EVT);
        }
   
      break;
      
    default:
      // Should not get here
      break;
  }
}
#endif

/*********************************************************************
 * @fn      alarmChangeCB
 *
 * @brief   Callback from Alarm Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
#if defined FEATURE_ALARM
static void alarmChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Alarm_GetParameter( SENSOR_CONF, &newValue );
      if ( newValue == DF_CFG_SENSOR_DISABLE)
      {
        // Put alarm to sleep
        if (alarmConfig != DF_CFG_SENSOR_DISABLE)
        {
          alarmConfig = DF_CFG_SENSOR_DISABLE;
          osal_set_event( Dragonfly_TaskID, DF_APP_ALARM_EVT);
        }
      }
      else
      {
        if (alarmConfig == DF_CFG_SENSOR_DISABLE)
        {
          // Start scheduling only on change disabled -> enabled
          osal_set_event( Dragonfly_TaskID, DF_APP_ALARM_EVT);
        }
        // Scheduled already, so just change range
        alarmConfig = newValue;
//        HalAccSetRange(accConfig);
      }
      break;

    case SENSOR_PERI:
      Alarm_GetParameter( SENSOR_PERI, &newValue );
      sensorAlarmPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}
#endif

/*********************************************************************
 * @fn      digitalChangeCB
 *
 * @brief   Callback from Digital Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
#if defined FEATURE_DIGITAL 
static void digitalChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch ( paramID)
  {
  case  SENSOR_CONF:
    Digital_GetParameter( SENSOR_CONF, &newValue );
    
    if ( newValue == DF_CFG_SENSOR_DISABLE)
    {
      if (digitalConfig != DF_CFG_SENSOR_DISABLE)
      {
        digitalConfig = DF_CFG_SENSOR_DISABLE;
        osal_set_event( Dragonfly_TaskID, DF_SEN_DIGITAL_EVT);
      }
    }
    else
    {
      if (digitalConfig == DF_CFG_SENSOR_DISABLE)
      {
        osal_set_event( Dragonfly_TaskID, DF_SEN_DIGITAL_EVT);
      }
      digitalConfig = newValue;
    }
    break;
    
  case SENSOR_PERI:
    Digital_GetParameter( SENSOR_PERI, &newValue );
    sensorDigitalPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}
#endif

/*********************************************************************
 * @fn      analogChangeCB
 *
 * @brief   Callback from Analog Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
#if defined FEATURE_ANALOG 
static void analogChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint32 newPeriValue;

  switch ( paramID)
  {
  case  SENSOR_CONF:
    Analog_GetParameter( SENSOR_CONF, &newValue );
    
    if ( newValue == DF_CFG_SENSOR_DISABLE)
    {
      if (analogConfig != DF_CFG_SENSOR_DISABLE )
      {
        analogConfig = FALSE;
        osal_set_event( Dragonfly_TaskID, DF_SEN_ANALOG_EVT);
      }
    }
    else
    {
      if (analogConfig == DF_CFG_SENSOR_DISABLE)
      {
        osal_set_event( Dragonfly_TaskID, DF_SEN_ANALOG_EVT);
      }
      analogConfig = newValue;
    }
    break;
    
  case SENSOR_PERI:
    Analog_GetParameter( SENSOR_PERI, &newPeriValue );
    sensorAnalogPeriod = newPeriValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}
#endif

/*********************************************************************
 * @fn      weatherChangeCB
 *
 * @brief   Callback from Weather Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
#if defined FEATURE_WEATHER 
static void weatherChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch ( paramID)
  {
  case  SENSOR_CONF:
    Weather_GetParameter( SENSOR_CONF, &newValue );
    
    if ( newValue == DF_CFG_SENSOR_DISABLE)
    {
      if (weatherConfig != DF_CFG_SENSOR_DISABLE)
      {
        weatherConfig = DF_CFG_SENSOR_DISABLE;
        osal_set_event( Dragonfly_TaskID, DF_SEN_WEATHER_EVT);
      }
    }
    else
    {
      if (weatherConfig == DF_CFG_SENSOR_DISABLE)
      {
        osal_set_event( Dragonfly_TaskID, DF_SEN_WEATHER_EVT);
      }
      weatherConfig = newValue;
    }
    break;
    
  case SENSOR_PERI:
    Weather_GetParameter( SENSOR_PERI, &newValue );
    sensorWeatherPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}
#endif

/*********************************************************************
 * @fn      luxometerChangeCB
 *
 * @brief   Callback from Luxometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
#if defined FEATURE_LUXOMETER 
static void luxometerChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch ( paramID)
  {
  case  SENSOR_CONF:
    Luxometer_GetParameter( SENSOR_CONF, &newValue );
    
    if ( newValue == DF_CFG_SENSOR_DISABLE)
    {
      if (luxometerEnabled)
      {
        luxometerEnabled = FALSE;
        osal_set_event( Dragonfly_TaskID, DF_SEN_LUXOMETER_EVT);
      }
    }
    else
    {
      if (!luxometerEnabled)
      {
        luxometerEnabled = TRUE;
        luxometerState = HAL_LUX_MEAS_STATE_0;
        osal_set_event( Dragonfly_TaskID, DF_SEN_LUXOMETER_EVT);
      }
    }
    break;
    
  case SENSOR_PERI:
    Luxometer_GetParameter( SENSOR_PERI, &newValue );
    sensorLuxometerPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}
#endif

/*********************************************************************
 * @fn      uviChangeCB
 *
 * @brief   Callback from UVi Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
#if defined FEATURE_UVI 
static void uviChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch ( paramID)
  {
  case  SENSOR_CONF:
    UVi_GetParameter( SENSOR_CONF, &newValue );
    
    if ( newValue == DF_CFG_SENSOR_DISABLE)
    {
      if (uviEnabled)
      {
        uviEnabled = FALSE;
        osal_set_event( Dragonfly_TaskID, DF_SEN_UVI_EVT);
      }
    }
    else
    {
      if (!uviEnabled)
      {
        uviEnabled = TRUE;
        uviState = HAL_UVI_MEAS_STATE_0;
        osal_set_event( Dragonfly_TaskID, DF_SEN_UVI_EVT);
      }
    }
    break;
    
  case SENSOR_PERI:
    UVi_GetParameter( SENSOR_PERI, &newValue );
    sensorUViPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}
#endif


#if defined FEATURE_AIR 
/*********************************************************************
 * @fn      airChangeCB
 *
 * @brief   Callback from Air Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void airChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch ( paramID)
  {
  case  SENSOR_CONF:
    Air_GetParameter(SENSOR_CONF, &newValue);
    
    if (newValue == DF_CFG_SENSOR_DISABLE)
    {
      if (airEnabled)
      {
        airEnabled = FALSE;
        osal_set_event(Dragonfly_TaskID, DF_SEN_AIR_EVT);
      }
    }
    else
    {
      if (!airEnabled)
      {
        airEnabled = TRUE;
//        airState = HAL_AIR_MEAS_STATE_0;
        osal_set_event(Dragonfly_TaskID, DF_SEN_AIR_EVT);
      }
    }
    break;
    
  case SENSOR_PERI:
    Air_GetParameter(SENSOR_PERI, &newValue);
    sensorAirPeriod = newValue*SENSOR_PERIOD_RESOLUTION>>1;
    osal_set_event( Dragonfly_TaskID, DF_SEN_AIR_EVT);
    break;
    
  default:
    // Should not get here
    break;
  }
}
#endif


#if defined FEATURE_MAGNETOMETER
/*********************************************************************
 * @fn      magnetometerChangeCB
 *
 * @brief   Callback from Magnetometre service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void magnetometerChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint32 newPeriValue;

  switch ( paramID)
  {
  case  SENSOR_CONF:
    Magnetometer_GetParameter( SENSOR_CONF, &newValue );
    
    if ( newValue == DF_CFG_SENSOR_DISABLE)
    {
      if (magnetometerConfig != DF_CFG_SENSOR_DISABLE )
      {
        magnetometerConfig = FALSE;
        osal_set_event( Dragonfly_TaskID, DF_SEN_MAGNETOMETER_EVT);
      }
    }
    else
    {
      if (magnetometerConfig == DF_CFG_SENSOR_DISABLE)
      {
        osal_set_event( Dragonfly_TaskID, DF_SEN_MAGNETOMETER_EVT);
      }
      magnetometerConfig = newValue;
    }
    break;
    
  case SENSOR_PERI:
    Magnetometer_GetParameter( SENSOR_PERI, &newPeriValue );
    sensorMagnetometerPeriod = newPeriValue*SENSOR_PERIOD_RESOLUTION;
    break;
    
  default:
    // Should not get here
    break;
  }
}
#endif


#if defined FEATURE_TEST
/*********************************************************************
 * @fn      testChangeCB
 *
 * @brief   Callback from Test indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void testChangeCB( uint8 paramID )
{
  if( paramID == TEST_CONF_ATTR )
  {
    uint8 newValue;

    Test_GetParameter( TEST_CONF_ATTR, &newValue );

    if (newValue & TEST_MODE_ENABLE)
    {
      testMode = TRUE;
    }
    else
    {
      testMode = FALSE;
    }

    if (testMode)
    {
      // Test mode: possible to operate LEDs. Key hits will cause notifications,
      // side key does not influence connection state
      
      if (newValue & 0x01)
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      }

      if (newValue & 0x02)
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      }

      if (newValue & 0x04)
      {
        HalLedSet(HAL_LED_3,HAL_LED_MODE_ON);
      }
      else
      {
        HalLedSet(HAL_LED_3,HAL_LED_MODE_OFF);
      }

      if (newValue & 0x08)
      {
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
        BuzzerRing (100, HAL_BUZZER_HIGH_TONE); // short high tone
#endif
      }

      if (newValue & 0x10)
      {
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
        BuzzerRing (100, HAL_BUZZER_LOW_TONE); // long low tone
#endif
      }

      if (newValue && 0x20) //Activate Charger Test
      {
//        HAL_QI_CTRL_SBIT = newValue & 0x01;
        
        
      }
    }    
    else
    {
      // Normal mode; make sure LEDs are reset and attribute cleared
      HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
      HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
      HalLedSet(HAL_LED_3,HAL_LED_MODE_OFF);
      newValue = 0x00;
      Test_SetParameter( TEST_CONF_ATTR, 1, &newValue );
    }
  }
}
#endif



#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
/*********************************************************************
 *
 * @fn      BuzzerRing
 *
 * @brief   Function to Ring Buzzer.
 *
 * @param   timeout - number of msec to ring the buzzer
 * @param   tone - type of tone (low or high)
 *
 * @return  void
 */
extern void BuzzerRing( uint16 timeout, uint8 tone )
{
  /* Provide ring buzzer */
  /* Tell OSAL to not go to sleep because buzzer uses T4 */
  osal_pwrmgr_device (PWRMGR_ALWAYS_ON);

  /* Ring buzzer */
  HalBuzzerRing (timeout, tone, BuzzerCompleteCB);
  
  /* Complete beep*/
//  BuzzerComplete = FALSE;
}
#endif

#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
/*********************************************************************
 *
 * @fn      BuzzerCompleteCB
 *
 * @brief   Callback function called when ringing of buzzer is done.
 *
 * @param   none
 *
 * @return  void
 */
extern void BuzzerCompleteCB( void )
{
  /* Tell OSAL it's OK to go to sleep */
  osal_pwrmgr_device( /*PWRMGR_ALWAYS_ON*/ PWRMGR_BATTERY );

  /* Complete beep*/
//  BuzzerComplete = TRUE;
}
#endif

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
#if defined FEATURE_KEY
  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
//  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &Actual_Keys );

  uint8 aData[EVENT_DATA_LEN];
  aData[0] = Actual_Keys;
  Event_SetParameter( SENSOR_DATA, EVENT_DATA_LEN, aData);   
#endif
  
#ifdef FEATURE_REGISTER
  // Force notification on Register Data (if enabled)
  Register_setParameter(SENSOR_DATA,0,NULL);
#endif
}


/*********************************************************************
 * @fn      resetCharacteristicValue
 *
 * @brief   Initialize a characteristic value to zero
 *
 * @param   servID - service ID (UUID)
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialise with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
//static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, 
//                                     uint8 value, uint8 paramLen)
static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, 
                                     uint32 value, uint8 paramLen)
{
  uint8* pData = osal_mem_alloc(paramLen);

  if (pData == NULL)
  {
    return;
  }

  osal_memset(pData,value,paramLen);

  switch(servUuid)
  {
/*
    case IRTEMPERATURE_SERV_UUID:
      IRTemp_SetParameter( paramID, paramLen, pData);
      break;
*/
    case ACCELEROMETER_SERV_UUID:
      Accel_SetParameter(paramID, paramLen, pData);
      break;

#if defined FEATURE_ITEMP
    case INTEMP_SERV_UUID:
      Intemp_SetParameter(paramID, paramLen, pData);
      break;
#endif

#if defined FEATURE_DATALOGGER
    case DATALOGGER_SERV_UUID:
      Datalogger_SetParameter(paramID, paramLen, pData);
      break;
#endif
      
#if defined FEATURE_ALARM 
    case ALARM_SERV_UUID:
      Alarm_SetParameter(paramID, paramLen, pData);
      break;
#endif
      
#if defined FEATURE_DIGITAL 
    case DIGITAL_SERV_UUID:
      Digital_SetParameter(paramID, paramLen, pData);
      break;
#endif
      
#if defined FEATURE_ANALOG 
    case ANALOG_SERV_UUID:
      Analog_SetParameter(paramID, paramLen, pData);
      break;
#endif
      
#if defined FEATURE_WEATHER 
    case WEATHER_SERV_UUID:
      Weather_SetParameter(paramID, paramLen, pData);
      break;
#endif
      
#if defined FEATURE_LUXOMETER 
    case LUXOMETER_SERV_UUID:
      Luxometer_SetParameter(paramID, paramLen, pData);
      break;
#endif

#if defined FEATURE_UVI 
    case UVI_SERV_UUID:
      UVi_SetParameter(paramID, paramLen, pData);
      break;
#endif

#if defined FEATURE_AIR 
    case AIR_SERV_UUID:
      Air_SetParameter(paramID, paramLen, pData);
      break;
#endif

    default:
      // Should not get here
      break;
  }

  osal_mem_free(pData);
}

/*********************************************************************
 * @fn      resetCharacteristicValues
 *
 * @brief   Initialize all the characteristic values
 *
 * @return  none
 */
static void resetCharacteristicValues( void )
{
/*
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_DATA, 0, IRTEMPERATURE_DATA_LEN);
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( IRTEMPERATURE_SERV_UUID, SENSOR_PERI, TEMP_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
*/
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_DATA, 0, ACCELEROMETER_DATA_LEN );
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( ACCELEROMETER_SERV_UUID, SENSOR_PERI, ACC_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

#if defined FEATURE_ITEMP
  resetCharacteristicValue( INTEMP_SERV_UUID, SENSOR_DATA, 0, INTEMP_DATA_LEN );
  resetCharacteristicValue( INTEMP_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( INTEMP_SERV_UUID, SENSOR_PERI, INTEMP_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
#endif

#if defined FEATURE_DATALOGGER  
  resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_DATA, 0, DATALOGGER_DATA_LEN );
//  resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, DATALOGGER_CONF_LEN);
//  resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_PERI, DATALOGGER_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
// Mirar a ver como podemos resetear esta variable bien
  resetCharacteristicValue( DATALOGGER_SERV_UUID, SENSOR_PERI, 0, DATALOGGER_PERI_LEN);
#endif
  
#if defined FEATURE_ALARM  
  resetCharacteristicValue( ALARM_SERV_UUID, SENSOR_DATA, 0, ALARM_DATA_LEN );
  resetCharacteristicValue( ALARM_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( ALARM_SERV_UUID, SENSOR_PERI, ALARM_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
#endif
  
#if defined FEATURE_DIGITAL 
  resetCharacteristicValue( DIGITAL_SERV_UUID, SENSOR_DATA, 0, DIGITAL_DATA_LEN);
  resetCharacteristicValue( DIGITAL_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( DIGITAL_SERV_UUID, SENSOR_PERI, DIGITAL_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
#endif
  
#if defined FEATURE_ANALOG 
  resetCharacteristicValue( ANALOG_SERV_UUID, SENSOR_DATA, 0, ANALOG_DATA_LEN);
  resetCharacteristicValue( ANALOG_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
//  resetCharacteristicValue( ANALOG_SERV_UUID, SENSOR_PERI, ANALOG_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
// Mirar a ver como podemos resetear esta variable bien
  resetCharacteristicValue( ANALOG_SERV_UUID, SENSOR_PERI, 0, ANALOG_PERI_LEN);
#endif
  
#if defined FEATURE_WEATHER 
  resetCharacteristicValue( WEATHER_SERV_UUID, SENSOR_DATA, 0, WEATHER_DATA_LEN);
  resetCharacteristicValue( WEATHER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( WEATHER_SERV_UUID, SENSOR_PERI, WEATHER_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
#endif
  
#if defined FEATURE_LUXOMETER 
  resetCharacteristicValue( LUXOMETER_SERV_UUID, SENSOR_DATA, 0, LUXOMETER_DATA_LEN);
  resetCharacteristicValue( LUXOMETER_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( LUXOMETER_SERV_UUID, SENSOR_PERI, LUXOMETER_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
#endif

#if defined FEATURE_UVI 
  resetCharacteristicValue( UVI_SERV_UUID, SENSOR_DATA, 0, UVI_DATA_LEN);
  resetCharacteristicValue( UVI_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( UVI_SERV_UUID, SENSOR_PERI, UVI_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
#endif

#if defined FEATURE_AIR 
  resetCharacteristicValue( AIR_SERV_UUID, SENSOR_DATA, 0, AIR_DATA_LEN);
  resetCharacteristicValue( AIR_SERV_UUID, SENSOR_CONF, DF_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( AIR_SERV_UUID, SENSOR_PERI, AIR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
#endif
}


// oJo solo para testerar SERIAL_INTERFACE, eliminar
uint8 Application_StartAdvertise(uint16 duration, uint16 interval)
{
    
  if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 status;    
         
      //TODO: Check if advertising parameters are legal
      
      //Set fast advertising interval for user-initiated connections
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, interval );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, interval );
      GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, duration );
         
      // toggle GAP advertisement status
      GAPPeripheralRole_GetParameter( GAPROLE_ADVERT_ENABLED, &status );
      if (status == FALSE)
      {
        status = !status;
        GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &status ); 
        return SUCCESS;
      }
      
    }
  return FAILURE;
}


/*********************************************************************
*********************************************************************/

