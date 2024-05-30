

#include "contiki.h"
#include "lib/random.h"
#include "clock.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "mqtt_sn.h"
#include "dev/leds.h"
#include "net/rime/rime.h"
#include "net/ip/uip.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "dev/button-sensor.h"
#include "dev/uart.h"
#include "dev/gpio.h"


#include "dev/i2c.h"
#include "dev/tsl256x.h"
#include "dev/bmpx8x.h"
#include "dev/si7021.h"
// CC2538DK has shield with sensors

static uint16_t TSL256X_light;
static uint16_t BMPx8x_pressure;
static int16_t  BMPx8x_temperature;
static uint16_t Si7021_humidity;
static uint16_t Si7021_temperature;

struct env_db {
  uint16_t  BMPx8x_temp;
  uint16_t  Si7021_temp;
  uint16_t  light;
  uint16_t  pressure;
  uint16_t  humidity;
  int32_t   Si7021_tData;
  uint32_t  Si7021_rhData;
  
};
static struct env_db env_db;


#define LED_RED        LEDS_BLUE
#define LED_GREEN      LEDS_RED
#define LED_BLUE       LEDS_GREEN



uint8_t temp_sensor_value, humid_sensor_value;

static uint16_t udp_port = 1884;
static uint16_t keep_alive = 5;
static uint16_t broker_address[] = {0xaaaa, 0, 0, 0, 0, 0, 0, 0x1};
static struct   etimer time_poll;
// static uint16_t tick_process = 0;
static char     pub_msg[30];
static char     device_id[17];
static char     topic_hw[25];
static char     *topics_mqtt[] = {"/temp",
                                  "/humid",
                                  "/press",
                                  "/light"};
// static char     *will_topic = "/6lowpan_node/offline";
// static char     *will_message = "O dispositivo esta offline";
// This topics will run so much faster than others

static  uint8_t led_on_off;                                  

mqtt_sn_con_t mqtt_sn_connection;

void mqtt_sn_callback(char *topic, char *message){
  printf("\nMessage received:");
  printf("\nTopic:%s Message:%s",topic,message);
}

void init_broker(void){
  char *all_topics[ss(topics_mqtt)+1];
  sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",
          linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
          linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
          linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
          linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
  // sprintf(topic_hw,"Hello addr:%02X%02X",linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);

  mqtt_sn_connection.client_id     = device_id;
  mqtt_sn_connection.udp_port      = udp_port;
  mqtt_sn_connection.ipv6_broker   = broker_address;
  mqtt_sn_connection.keep_alive    = keep_alive;
  //mqtt_sn_connection.will_topic    = will_topic;   // Configure as 0x00 if you don't want to use
  //mqtt_sn_connection.will_message  = will_message; // Configure as 0x00 if you don't want to use
  mqtt_sn_connection.will_topic    = 0x00;
  mqtt_sn_connection.will_message  = 0x00;

  mqtt_sn_init();   // Inicializa alocação de eventos e a principal PROCESS_THREAD do MQTT-SN

  size_t i;
  for(i=0;i<ss(topics_mqtt);i++)
    all_topics[i] = topics_mqtt[i];
  all_topics[i] = topic_hw;

  mqtt_sn_create_sck(mqtt_sn_connection,
                     all_topics,
                     ss(all_topics),
                     mqtt_sn_callback);
  // mqtt_sn_sub((char*)"#",0);
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void init_sensor() {
  GPIO_SET_OUTPUT(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5)); 
  GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5));
  SENSORS_ACTIVATE(bmpx8x);
  if(TSL256X_REF == TSL2561_SENSOR_REF) {
      PRINTF("Light sensor test --> TSL2561\n");
    } else if(TSL256X_REF == TSL2563_SENSOR_REF) {
      PRINTF("Light sensor test --> TSL2563\n");
    } else {
      PRINTF("Unknown light sensor reference, aborting\n");
    }

  SENSORS_ACTIVATE(tsl256x);
    //TSL256X_REGISTER_INT(light_interrupt_callback);
  tsl256x.configure(TSL256X_INT_OVER, 0x15B8);
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void process_sensor(uint8_t verbose) {
  int32_t   tData;
  uint32_t  rhData;
  uint8_t   H, L;

  BMPx8x_pressure = bmpx8x.value(BMPx8x_READ_PRESSURE);
  BMPx8x_temperature = bmpx8x.value(BMPx8x_READ_TEMP);
  TSL256X_light = tsl256x.value(TSL256X_VAL_READ);
      

  if(TSL256X_light != TSL256X_ERROR) {
    env_db.light = TSL256X_light;
    //context-aware control here: if light is too low, turn on LEDs
    //set_led_cc2538_shield(env_db.light < 20);   

  } else {
    PRINTF("Error, enable the DEBUG flag in the tsl256x driver for info, or check if the sensor is properly connected \n");
  } 

  if((BMPx8x_pressure != BMPx8x_ERROR) && (BMPx8x_temperature != BMPx8x_ERROR)) {
    //PRINTF("BMPx8x : Pressure = %u.%u(hPa), \n", pressure / 10, pressure % 10);
    //PRINTF("Temperature = %d.%u(ºC) \n", temperature / 10, temperature % 10);
    env_db.pressure = BMPx8x_pressure;
    env_db.BMPx8x_temp = BMPx8x_temperature;
  } else {
    PRINTF("Error, enable the DEBUG flag in the BMPx8x driver for info, or check if the sensor is properly connected \n");
  }
    
    
  //convert Si7021 temperature
  Si7021_temperature = si7021_readTemp(TEMP_NOHOLD);  
  H = (uint8_t)(Si7021_temperature >> 8);
  L = (uint8_t)(Si7021_temperature & 0xFF);
  tData = ((uint32_t)H  << 8) + (L & 0xFC);
  tData = (((tData) * 21965L) >> 13) - 46850;
  env_db.Si7021_tData =tData;
  env_db.Si7021_temp = Si7021_temperature;
  //convert Si7021 humidity 
  Si7021_humidity = si7021_readHumd(RH_NOHOLD);
  H = (uint8_t)(Si7021_humidity >> 8);
  L = (uint8_t)(Si7021_humidity & 0xFF);
  rhData = ((uint32_t)H << 8) + (L & 0xFC);
  rhData = (((rhData) * 15625L) >> 13) - 6000;
  env_db.Si7021_rhData =rhData;
  env_db.humidity = Si7021_humidity;
    
  if (verbose) {
    PRINTF("\n--------------------- READING SENSORS ----------------------\n");
      PRINTF(" - Temperature (Si7021) = %d.%d (ºC) \n", (uint16_t)(tData /1000), (uint16_t)(tData % 1000));
      PRINTF(" - Humidity (Si7021)    = %d.%d (RH) \n", (uint16_t)(rhData/1000), (uint16_t)(rhData % 1000));
      PRINTF(" - Temperature (BMPx8x) = %d.%d (ºC) \n", (uint16_t)(env_db.BMPx8x_temp/10), (uint16_t)(env_db.BMPx8x_temp % 10));
      PRINTF(" - Pressure (BMPx8x)    = %d.%d (hPa)\n", (uint16_t)(env_db.pressure / 10), (uint16_t)(env_db.pressure % 10));
      PRINTF(" - Light (TSL256X)      = %d (lux)\n", (uint16_t)(env_db.light));
    PRINTF("------------------------------------------------------------\n");
  }

}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(init_system_process, "[Contiki-OS] Initializing OS");
AUTOSTART_PROCESSES(&init_system_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(init_system_process, ev, data) {
  PROCESS_BEGIN();

  debug_os("Initializing the MQTT_SN_DEMO");

  init_broker();
  init_sensor();
  process_sensor(1);
  etimer_set(&time_poll,10*CLOCK_SECOND);

  led_on_off = 0;

  while(1) {
    PROCESS_WAIT_EVENT();
    process_sensor(1);  

    //sprintf(pub_msg, "%03d:%03d", linkaddr_node_addr.u8[7], temp_sensor_value);

    //mqtt_sn_pub("/temp", pub_msg, true, 0);
    //sprintf(pub_msg, "%03d:%03d", linkaddr_node_addr.u8[7], humid_sensor_value);
    //mqtt_sn_pub("/humid", pub_msg, true, 0);

    //temperature
// sprintf(pub_msg, "(Si7021) %d.%d (BMPx8x) %d.%d (ºC)",(uint16_t)(env_db.Si7021_tData/1000),(uint16_t)(env_db.Si7021_tData%1000),
//                                                          (uint16_t)(env_db.BMPx8x_temp/10), (uint16_t)(env_db.BMPx8x_temp % 10));
    sprintf(pub_msg, "%d.%d",(uint16_t)(env_db.Si7021_tData/1000),(uint16_t)(env_db.Si7021_tData%1000));
    mqtt_sn_pub("/temp", pub_msg, true, 0);


    // humidity
    //sprintf(pub_msg, "(Si7021) %d.%d (RH)",(uint16_t)(env_db.Si7021_rhData/1000), (uint16_t)(env_db.Si7021_rhData % 1000));
    sprintf(pub_msg, "%d.%d",(uint16_t)(env_db.Si7021_rhData/1000), (uint16_t)(env_db.Si7021_rhData % 1000));
    mqtt_sn_pub("/humid", pub_msg, true, 0);

    // Pressure
    //sprintf(pub_msg, "(BMPx8x) %d.%d (hPa)",(uint16_t)(env_db.pressure / 10), (uint16_t)(env_db.pressure % 10));
    sprintf(pub_msg, "%d.%d",(uint16_t)(env_db.pressure / 10), (uint16_t)(env_db.pressure % 10));
    mqtt_sn_pub("/press", pub_msg, true, 0);

    // Light
    //sprintf(pub_msg, "(TSL256X) %d (lux)",(uint16_t)(env_db.light));
    sprintf(pub_msg, "%d",(uint16_t)(env_db.light));
    mqtt_sn_pub("/light", pub_msg, true, 0);

    // end pub
    //debug_os("State MQTT:%s",mqtt_sn_check_status_string());
    if (etimer_expired(&time_poll)) {
      etimer_reset(&time_poll);
    }
  }

  PROCESS_END();
}
