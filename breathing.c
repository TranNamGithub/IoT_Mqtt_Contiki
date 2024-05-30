

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

#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/uart1.h"
#include "lib/ringbuf.h"

#include<stdio.h>
#include<stdint.h>

static char setting_cmd[10]="auto";
static uint8_t IndexStruct;
struct env_db {
  uint8_t   Respiratory_rate;
  char      Breath_state[20];
  uint8_t   Heart_rate;
  
};
static struct env_db env_db;

uint8_t rxStartFrame = 0;
uint8_t rxEndFrame = 1;
uint8_t rxBuffer[20];
uint8_t rxbufferIndex = 0; 
uint8_t numberBytes;

static struct   etimer time_poll;
static uint16_t udp_port = 1884;
static uint16_t keep_alive = 5;
static uint16_t broker_address[] = {0xaaaa, 0, 0, 0, 0, 0, 0, 0x1};
// static uint16_t tick_process = 0;
static char     pub_msg[20];
static char     breath_cmd[20];
static char     device_id[17];
static char     topic_hw[25];
static char     *topics_mqtt[] = {"/b_control_d",
                                  "/a_or_m",
                                  "/breath_state",
                                  "/respi_rate",
                                  "/heart_rate"};
// static char     *will_topic = "/6lowpan_node/offline";
// static char     *will_message = "O dispositivo esta offline";
// This topics will run so much faster than others
                             

mqtt_sn_con_t mqtt_sn_connection;

void mqtt_sn_callback(char *topic, char *message){

  if(strcmp(topic, "/b_control_d") == 0){
    strcpy(breath_cmd, message);
  }
  else if(strcmp(topic, "/a_or_m") == 0){
    strcpy(setting_cmd, message);
  }

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
unsigned int uart0_send_bytes(const  unsigned  char *s, unsigned int len)
{

  unsigned int i = 0;
  while(s && *s != 0){
  if(i >= len){
    break;
  }
  uart_write_byte(0, *s++);
  i++;
   }
//   printf("uart0 send \n");
   return i;

}

unsigned int uart1_send_bytes(const  unsigned  char *s, unsigned int len)
{

  unsigned int i = 0;
  while(s && *s != 0){
  if(i >= len){
    break;
  }
  uart_write_byte(1, *s++);
  i++;
   }
//   printf("uart1 send \n");
   return i;
}

/*---------------------------------------------------------------------------*/

int my_uart1_input_byte(unsigned char c)
{
  //printf("%X\n", c);
  if(c == 0x55 && rxStartFrame == 0){
    rxEndFrame = 0;
    rxStartFrame = 1;
    rxbufferIndex = 0;
    rxBuffer[rxbufferIndex++] = c;
  }
  else if (rxbufferIndex>0 )
  {
    rxBuffer[rxbufferIndex++] = c;
    if(rxbufferIndex == 2){
      numberBytes = c;
    }
    if(rxbufferIndex -1 == numberBytes){
      rxEndFrame = 1;
      rxStartFrame = 0;

    }
  }

  return 1;



}

static void inprocess_data_output(uint8_t EndFrame, uint8_t *data){
  if(EndFrame == 1){
    if(data[0] == 0x55 && data[3] == 0x05){
      switch(data[4]){
        case 0x01: //respiratory parameters
          if(data[5] == 0x01){
            env_db.Respiratory_rate = data[6];
            IndexStruct = 1;
          }
          else if(data[5] == 0x04){
            switch(data[6]){
              case 0x01:
                strcpy(env_db.Breath_state, "Abnormal");
              break;
              case 0x02:
                strcpy(env_db.Breath_state, "none");
              break;
              case 0x03:
                strcpy(env_db.Breath_state, "normal");
              break;
              case 0x04:
                strcpy(env_db.Breath_state, "move");
              break;
              case 0x05:
                strcpy(env_db.Breath_state, "Abnormal rapid");
              break;
            }
            IndexStruct = 2;
          }
        break;
        case 0x03: // Scenario assessment

        break;
        case 0x04: // Duration parameter
        break;
        case 0x05: // sleep quality parameters
        break;
        case 0x06: // Heart rate parameters
          if(data[5] == 0x01){
            env_db.Heart_rate = data[6];
          }
          IndexStruct = 3;
        break;
      }
    }
    rxEndFrame = 0;
  }
}

static void init_gpio(void){
  GPIO_SET_OUTPUT(GPIO_B_BASE, (0x01 | 0x01<<1)); 
  GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1));
}

static void v_GPIO_Handle(void) {
  if(strcmp(setting_cmd, "manual") == 0){
    if (strcmp(breath_cmd, "breath_on") == 0) {

      GPIO_SET_PIN(GPIO_B_BASE, (0x01<<1)); // turn on led co2

    }
    else if (strcmp(breath_cmd, "breath_off") == 0) {

      GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<1)); //turn on led co2

    }
  }
}
static void check_breath_state(void){
  if(strcmp(setting_cmd, "auto") == 0){
    if (strcmp(env_db.Breath_state, "Abnormal") == 0) {

      GPIO_SET_PIN(GPIO_B_BASE, (0x01<<1));

    }
    else if (strcmp(env_db.Breath_state, "Abnormal rapid") == 0) {

      GPIO_SET_PIN(GPIO_B_BASE, (0x01<<1)); 

    }
    else if (strcmp(env_db.Breath_state, "none") == 0) {

      GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<1)); 

    }
    else if (strcmp(env_db.Breath_state, "normal") == 0) {

      GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<1)); 

    }
    else if (strcmp(env_db.Breath_state, "move") == 0) {

      GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<1)); 

    }
  }
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

/* Variables: the application specific event value */

PROCESS(init_system_process, "[Contiki-OS] Initializing OS");
/* Implementation of the second process */
AUTOSTART_PROCESSES(&init_system_process);

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(init_system_process, ev, data) {
  PROCESS_BEGIN();

  debug_os("Initializing the MQTT_SN_DEMO");
  init_broker();
  init_gpio();
  uart_init(1); 
  uart_set_input(1,my_uart1_input_byte);
  etimer_set(&time_poll,0.005*CLOCK_SECOND);

  while(1) {
    inprocess_data_output(rxEndFrame,rxBuffer);
    check_breath_state();
    PROCESS_WAIT_EVENT();
    v_GPIO_Handle();
    if(IndexStruct == 1){
      sprintf(pub_msg,"%d", env_db.Respiratory_rate);
      mqtt_sn_pub("/respi_rate", pub_msg, true, 0);
      IndexStruct = 0;
    }
    else if(IndexStruct == 2){
      sprintf(pub_msg,"%s",env_db.Breath_state);
      mqtt_sn_pub("/breath_state", pub_msg, true, 0);
      IndexStruct = 0;
    }
    if(IndexStruct == 3){
      sprintf(pub_msg,"%d",env_db.Heart_rate);
      mqtt_sn_pub("/heart_rate", pub_msg, true, 0);
      IndexStruct = 0;
    }
    mqtt_sn_sub("/b_control_d",0);
    mqtt_sn_sub("/a_or_m",0);
    if (etimer_expired(&time_poll)) {
      etimer_reset(&time_poll);
    }



  }

  PROCESS_END();
}
