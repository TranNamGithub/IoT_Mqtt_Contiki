

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
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/uart1.h"
#include "lib/ringbuf.h"
#include "dev/gpio.h"

#include<stdio.h>
#include<stdint.h>

static uint8_t count = 0;
static uint16_t temp_radar = 0;
static char setting_cmd[10] ="auto";    // command for led control
/*define of SENSOR CO2 UART*/
#define MAX_RX_BUFFER_SIZE 3
uint32_t rxBufferIndex = 0;
static uint8_t rxBuffer[MAX_RX_BUFFER_SIZE];
static uint16_t Pas_co2;
uint8_t set_mes_and_limit[4];
uint8_t reading, writing;
char Value_Co2_Oled[4];
static uint16_t co2_threshold_default =500;
/*end define of SENSOR CO2 UART*/


/*define of SENSOR radar UART*/
uint8_t rxStartFrame = 0;
uint8_t rxEndFrame = 0;
uint8_t rxBufferRadarIndex = 0;
uint8_t rxBufferradar[20];
static char rxBufferradarValue[7];
static uint8_t check_off = 0;
/*end  define of SENSOR radar UART*/

struct env_db {
  uint16_t  radar;
  uint16_t  co2;
  uint16_t  co2_meas;// 0-2000
  uint16_t  co2_threshold;//5-3600
  
};

static struct env_db env_db;
/* End define*/
///
#define LED_RED        LEDS_BLUE
#define LED_GREEN      LEDS_RED
#define LED_BLUE       LEDS_GREEN

uint8_t temp_sensor_value, humid_sensor_value;

static uint16_t udp_port = 1884;
static uint16_t keep_alive = 5;
static uint16_t broker_address[] = {0xaaaa, 0, 0, 0, 0, 0, 0, 0x1};
static struct   etimer time_poll;
// static uint16_t tick_process = 0;
static char     pub_msg[20];
static char     led_cmd[20];    // command for led control
static char     device_id[17];
static char     topic_hw[25];
static char     *topics_mqtt[] = {"/led_control",
                                  "/led_d",
                                  "/co2",
                                  "/cfco2",
                                  "/c_cfco2",
                                  "/radar",
                                  "/a_or_m"};
// static char     *will_topic = "/6lowpan_node/offline";
// static char     *will_message = "O dispositivo esta offline";
// This topics will run so much faster than others

uint16_t GetMsg_Co2_Meas(char* message){
  uint16_t msgValue = 0;
  msgValue = (message[4]-0x30)*1000;
  msgValue += (message[5]-0x30)*100;
  msgValue += (message[6]-0x30)*10;
  msgValue += (message[7]-0x30);
  return msgValue;
}

uint16_t GetMsg_Co2_thres(char* message){
  uint16_t msgValue = 0;
  msgValue = (message[0]-0x30)*1000;
  msgValue += (message[1]-0x30)*100;
  msgValue += (message[2]-0x30)*10;
  msgValue += (message[3]-0x30);
  return msgValue;  
}
uint16_t GetRange_radar(char* value){
  uint16_t msgValue = 0;
  msgValue = (value[2]-0x30)*10000;
  msgValue += (value[3]-0x30)*1000;
  msgValue += (value[4]-0x30)*100;
  msgValue += (value[5]-0x30)*10;
  msgValue += (value[6]-0x30);
  return msgValue;
}                              
mqtt_sn_con_t mqtt_sn_connection;
static void Co2_threshold_set(void);
static void Configco2_measurement(void);

void mqtt_sn_callback(char *topic, char *message){
  if(strcmp(topic, "/cfco2") == 0){
    env_db.co2_meas= GetMsg_Co2_Meas(message);
    env_db.co2_threshold =GetMsg_Co2_thres(message);
    Configco2_measurement();

  }
  else if(strcmp(topic, "/led_d") == 0){
    strcpy(led_cmd, message);
  }
  else if(strcmp(topic, "/a_or_m") == 0){
    strcpy(setting_cmd, message);
  }

}

void init_broker(void){
  char *all_topics[ss(topics_mqtt)+1];
  sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X1234",
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
  //mqtt_sn_sub((char*)"#",0);
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
unsigned int
uart0_send_bytes(const  unsigned  char *s, unsigned int len)
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

unsigned int
uart1_send_bytes(const  unsigned  char *s, unsigned int len)
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


int
my_uart1_input_byte(unsigned char c)
{
  rxBuffer[rxBufferIndex++] = c;
  if(writing ==1 && rxBufferIndex==2){
    rxBufferIndex = 0;
    writing = 0;
    rxBufferIndex = 0;
  }
  if (rxBufferIndex >= MAX_RX_BUFFER_SIZE)
  {
      // Handle buffer overflow here, if needed
      // For example, reset rxBufferIndex to 0
      rxBufferIndex = 0;
  }
  return 1;



}

int
my_uart0_input_byte(unsigned char c)
{
  if(c == 0x4F){
    rxEndFrame = 0;
    rxStartFrame = 1;
    rxBufferRadarIndex = 0;
    rxBufferradar[rxBufferRadarIndex++] = c;
  }
  else if (rxBufferRadarIndex>0 )
  {
    rxBufferradar[rxBufferRadarIndex++] = c;
   if(c ==0x0D && ((rxBufferRadarIndex-1) > 2) && rxStartFrame ==1){ /// data out is on: ON\r\nRange 82\r\n :::: is off: OFF\r\n
      rxEndFrame = 1;
    }
    if(c == 0x0A && ((rxBufferRadarIndex-1) > 3) && rxEndFrame == 1){
      rxStartFrame = 0;
      rxEndFrame = 2;
      if(rxBufferradar[0] ==0x4F && rxBufferradar[1] ==0x4E){  // O N

        rxBufferradarValue[0] = 0;
        rxBufferradarValue[1] = 1;
        uint8_t i,k;
        for (i = rxBufferRadarIndex-3; i >=10 ; --i)/// ON\r\nRange 82\r\n value starts index[10]
        {

          rxBufferradarValue[i-(rxBufferRadarIndex-3)+6] = rxBufferradar[i];
        }
        for (k = 2; k <= 6-(rxBufferRadarIndex-2-10); ++k)
        {
          rxBufferradarValue[k] =0x30; // '0'
        }

      }
      else if(rxBufferradar[0] == 0x4F && rxBufferradar[1] ==0x46 && rxBufferradar[2] ==0x46){ // O F F
        rxBufferradarValue[0] = 0;
        rxBufferradarValue[1] = 0;
        int i;
        for (i = 2; i <= 6; ++i)
        {
          rxBufferradarValue[i] = 0x30;
        }

      }
    }

  }

  
  //printf("gia tri la %x\n",rxBufferradar[rxBufferIndex]);
  return 1;



}

// function for co2
static inline uint8_t xensiv_pasco2_digit_to_ascii(uint8_t digit)
{
  if (digit < 10U)
  {
    return (uint8_t)(digit + 0x30U);
  }
  else
  {
    return (uint8_t)(digit + 0x37U);
  }
}

static inline uint8_t xensiv_pasco2_ascii_to_digit(uint8_t ascii)
{
  if (ascii < (uint8_t)'A')
  {
    return (uint8_t)(ascii - (uint8_t)'0');
  }
  else
  {
    return (uint8_t)(10u + (uint8_t)(ascii - (uint8_t)'A'));
  }
}

uint8_t pasco2_uart_read(uint8_t reg_addr, uint8_t *data)
{
  reading =1;
  uint8_t uart_buf[5] =
      {
          (uint8_t)'r',
          (uint8_t)',',
          xensiv_pasco2_digit_to_ascii((reg_addr & (uint8_t)0xF0) >> 4U),
          xensiv_pasco2_digit_to_ascii(reg_addr & (uint8_t)0x0F),
          (uint8_t)'\n'};
  uint8_t i;
  for (i = 0; i < 5; i++)
  {
    uart_write_byte(1, uart_buf[i]);
  }
  clock_wait(10);
  return (uint8_t)((xensiv_pasco2_ascii_to_digit(data[0])))*(0x10U)+(uint8_t)((xensiv_pasco2_ascii_to_digit(data[1])));
}

void pasco2_uart_write(uint8_t reg_addr, uint8_t value)
{

  writing =1;
  uint8_t uart_buf[8] =
      {
          (uint8_t)'w',
          (uint8_t)',',
          xensiv_pasco2_digit_to_ascii((reg_addr & 0xF0U) >> 4U), xensiv_pasco2_digit_to_ascii(reg_addr & 0x0FU),
          (uint8_t)',',
          xensiv_pasco2_digit_to_ascii((value & 0xF0U) >> 4U), xensiv_pasco2_digit_to_ascii(value & 0x0FU),
          (uint8_t)'\n'};

  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    uart_write_byte(1, uart_buf[i]);
  }

    clock_wait(10);

}

// end function for co2

//function for radar
void send_command_preamble_();
void send_command_postamble_();
void open_command_mode();
void shutdown_command_mode();
void Request_Firmware_Version();
void Request_Restart();
void read_minimum_distance();
void set_minimum_distance(uint8_t MinValue);
void read_maximum_distance();
void set_maximum_distance(uint8_t MaxValue);
void read_delay_time();
void set_delay_time(uint8_t TimeValue);
void read_maintain_threshold();
void set_maintain_threshold(uint8_t MaintainThresholdValue);
void serial_port_status_output_mode();
void energy_value_ouput_mode();
void debug_ouput_mode();
void close_en_mode();
//end function for radar

static void init_sensor(){
  //init co2
  uart_init(0); 
  uart_set_input(1,my_uart1_input_byte);
  uart_init(1); 
  uart_set_input(0,my_uart0_input_byte);
  pasco2_uart_read(0x00, rxBuffer);
  // check status
  pasco2_uart_read(0x01, rxBuffer);
  // idle mode
  pasco2_uart_write(0x04, 0x0000);
  pasco2_uart_write(0x02, 0x00);
  pasco2_uart_write(0x03, 0x10);
  pasco2_uart_write(0x04, 0x02);
  // init rarar
  clock_wait(30);
  open_command_mode();
  clock_wait(30);
  set_minimum_distance(0);
  clock_wait(30);
  set_maximum_distance(12);
  clock_wait(30);
  serial_port_status_output_mode();
  clock_wait(30);
  shutdown_command_mode();
  //
  //init gpio
  GPIO_SET_OUTPUT(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2)); 
  GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2));
  // init threshold co2
  env_db.co2_threshold = co2_threshold_default;

}


static void Co2_threshold_set(void){
  if(strcmp(setting_cmd, "auto") == 0){
    uint16_t threshold;
    threshold = env_db.co2_threshold;
    if(threshold <= env_db.co2){

      GPIO_SET_PIN(GPIO_B_BASE, (0x01<<1)); // turn on led co2
      sprintf(pub_msg, "co2_on");
    }
    else if (threshold > env_db.co2){

      GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<1)); //turn on led co2
      sprintf(pub_msg, "co2_off");
    }
    mqtt_sn_pub("/led_control", pub_msg, true, 0);
  }

}


static void process_sensor(){
    uint8_t check;
    uint8_t valueHCO2;
    uint8_t valueLCO2;

    //check running auto or manual
    if(strcmp(setting_cmd, "auto") == 0){
      if(rxBufferradarValue[0] == 0 && rxBufferradarValue[1] ==1){
        GPIO_SET_PIN(GPIO_B_BASE, (0x01<<2)); // turn on led radar
      }
      else if(rxBufferradarValue[0] == 0 && rxBufferradarValue[1] ==0){
        GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<2)); //turn off led radar
      }
    }
    // check on radar or off radar,
    if(rxBufferradarValue[0] == 0 && rxBufferradarValue[1] == 1 && count ==10){

      env_db.radar = GetRange_radar(rxBufferradarValue);
      if(temp_radar != env_db.radar){
        sprintf(pub_msg, "%d%d:%d", rxBufferradarValue[0], rxBufferradarValue[1], env_db.radar);
        mqtt_sn_pub("/radar", pub_msg, true, 0);
      }
      count =0;
      check_off = 0;
      temp_radar = env_db.radar;

    }
    else if(rxBufferradarValue[0] == 0 && rxBufferradarValue[1] == 0 && check_off == 0 && count ==10){ /// pub only 1 times, when radar off

      sprintf(pub_msg, "%d%d:0", rxBufferradarValue[0], rxBufferradarValue[1]);
      mqtt_sn_pub("/radar", pub_msg, true, 0);
      check_off = 1;
    }
    // check co2 is ready , if it's ready read value and check threshold pub message
    if(count ==6){
      pasco2_uart_read(0x07, rxBuffer);
      check = (uint8_t)((xensiv_pasco2_ascii_to_digit(rxBuffer[0])))*(0x10U)+(uint8_t)((xensiv_pasco2_ascii_to_digit(rxBuffer[1])));
      if(check ==0x10)
      {
        valueLCO2 = pasco2_uart_read(0x05, rxBuffer);
        valueHCO2 = pasco2_uart_read(0x06, rxBuffer);
        Pas_co2 = valueLCO2 << 8 | valueHCO2;
        env_db.co2 = Pas_co2;
        Co2_threshold_set();
        sprintf(pub_msg, "%d", env_db.co2);
        mqtt_sn_pub("/co2", pub_msg, true, 0);

      }
    }




}


static void v_GPIO_Handle(void) {
  if(strcmp(setting_cmd, "manual") == 0){
    if (strcmp(led_cmd, "radar_on") == 0) {

      GPIO_SET_PIN(GPIO_B_BASE, (0x01<<2)); // turn on led radar

    }
    else if (strcmp(led_cmd, "radar_off") == 0) {

      GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<2)); // turn off led radar
    }
    if (strcmp(led_cmd, "co2_on") == 0) {

      GPIO_SET_PIN(GPIO_B_BASE, (0x01<<1)); // turn on led co2

    }
    else if (strcmp(led_cmd, "co2_off") == 0) {

      GPIO_CLR_PIN(GPIO_B_BASE, (0x01<<1)); //turn on led co2

    }
  }
}

static void Configco2_measurement(void){
  uint8_t valueH;
  uint8_t valueL;
  valueH = (env_db.co2_meas >> 8) & 0xFF;
  valueL = env_db.co2_meas & 0xFF;

  pasco2_uart_read(0x00, rxBuffer);
  pasco2_uart_read(0x01, rxBuffer);
  pasco2_uart_write(0x04, 0x0000);
  pasco2_uart_write(0x02,valueH);
  pasco2_uart_write(0x03,valueL);
  pasco2_uart_write(0x04, 0x02);

  sprintf(pub_msg, "ok");
  mqtt_sn_pub("/c_cfco2", pub_msg, true, 0);

}
/*---------------------------------------------------------------------------*/
PROCESS(init_system_process, "[Contiki-OS] Initializing OS");
AUTOSTART_PROCESSES(&init_system_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(init_system_process, ev, data) {
  PROCESS_BEGIN();

  //debug_os("Initializing the MQTT_SN_DEMO");

  init_broker();
  etimer_set(&time_poll,0.1*CLOCK_SECOND);
  init_sensor();
  while(1) {
    PROCESS_WAIT_EVENT();
    process_sensor();
    v_GPIO_Handle();
    mqtt_sn_sub("/led_d",0);
    mqtt_sn_sub("/cfco2",1);
    mqtt_sn_sub("/a_or_m",0);
    if (etimer_expired(&time_poll)) {
      count ++;
      etimer_reset(&time_poll);
    }
  }

  PROCESS_END();
}


void send_command_preamble_(){
  //Command preamble
  uart_write_byte(0,0xFD);
  uart_write_byte(0,0xFC);
  uart_write_byte(0,0xFB);
  uart_write_byte(0,0xFA);
  
}
void send_command_postamble_()
{
  //Command end
  uart_write_byte(0,0x04);
  uart_write_byte(0,0x03);
  uart_write_byte(0,0x02);
  uart_write_byte(0,0x01);
}

// the loop function runs over and over again forever
void open_command_mode(){
  send_command_preamble_();
  uart_write_byte(0,0x04);
  uart_write_byte(0,0x00);
  uart_write_byte(0,0xFF);
  uart_write_byte(0,0x00);
  uart_write_byte(0,0x01);
  uart_write_byte(0,0x00);
  send_command_postamble_();

}

void shutdown_command_mode(){
  send_command_preamble_();
  uart_write_byte(0,0x02);
  uart_write_byte(0,0x00);
  uart_write_byte(0,0xFE);
  uart_write_byte(0,0x00);
  send_command_postamble_();
}

void Request_Firmware_Version(){
    open_command_mode();
  clock_wait(200);
    send_command_preamble_();
    //Request firmware
    uart_write_byte(0,0x02); //Command is two bytes long
    uart_write_byte(0,0x00);
    uart_write_byte(0,0x00); //Request firmware version
    uart_write_byte(0,0x00);
    send_command_postamble_();
   clock_wait(200);
    shutdown_command_mode();
}

void Request_Restart(){
    open_command_mode();
  clock_wait(200);
    send_command_preamble_();
    //Request firmware
    uart_write_byte(0,0x02); //Command is two bytes long
    uart_write_byte(0,0x00);
    uart_write_byte(0,0x68); //Request restart
    uart_write_byte(0,0x00);
    send_command_postamble_();
    clock_wait(200);
    shutdown_command_mode();
}

void read_minimum_distance(){
  send_command_preamble_();
  uart_write_byte(0,0x04);
  uart_write_byte(0,0x00);
  uart_write_byte(0,0x08);
  uart_write_byte(0,0x00);
  uart_write_byte(0,0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}
void set_minimum_distance(uint8_t MinValue){
  //range(0x00-0x0f)
  send_command_preamble_();
  uart_write_byte(0,0x08);
  uart_write_byte(0,0x00);
  uart_write_byte(0,0x07);
  uart_write_byte(0,0x00);
  uart_write_byte(0,0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0,MinValue);
  uart_write_byte(0,0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}
void read_maximum_distance(){
  send_command_preamble_();
  uart_write_byte(0,0x04);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0,0x01);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}

void set_maximum_distance(uint8_t MaxValue){
  //range(0x00-0x0f)
  send_command_preamble_();
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x07);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x01);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, MaxValue);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}
void read_delay_time(){
  send_command_preamble_();
  uart_write_byte(0, 0x04);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x04);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}

void set_delay_time(uint8_t TimeValue){
  //range(0x00-0x0f)
  send_command_preamble_();
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x07);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x04);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, TimeValue);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_(); 
}
void read_maintain_threshold(){

  send_command_preamble_();
  uart_write_byte(0, 0x04);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x20);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}

void set_maintain_threshold(uint8_t MaintainThresholdValue){
  //range(0x00-0x0f)
  send_command_preamble_();
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x07);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x20);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, MaintainThresholdValue);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}


void serial_port_status_output_mode(){
  send_command_preamble_();
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x12);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x64);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}


void energy_value_ouput_mode(){
  send_command_preamble_();
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x12);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x04);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}

void debug_ouput_mode(){
  send_command_preamble_();
  uart_write_byte(0, 0x08);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x12);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x00);
  send_command_postamble_();

}
void close_en_mode(){
    send_command_preamble_();
  uart_write_byte(0, 0x02);
  uart_write_byte(0, 0x00);
  uart_write_byte(0, 0x63);
  uart_write_byte(0, 0x00);
  send_command_postamble_();
}
