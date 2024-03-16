#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriberPWM;
rcl_publisher_t publisherRawPot;
rcl_publisher_t publisherVolt;

std_msgs__msg__Int32 msgPot;
std_msgs__msg__Float32 msgVolt;
std_msgs__msg__Float32 msgPWM;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;
rcl_timer_t timer1;
rcl_timer_t timer2;

#define LED_PIN 12
#define PWM_PIN 27
#define POT_PIN 14

uint16_t potValue = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//CREATE SUBSCRIBER CALLBACK
const int PWMFreq = 5000; /* 5 KHz */
const int PWMChannel = 0;
const int PWMResolution = 8;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
uint16_t dutyCycle = 0;

  void subscription_callback(const void * msgin){  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if (msg->data >= 0 && msg->data <= 100 ){
    dutyCycle = map(msg->data, 0, 100, 0, MAX_DUTY_CYCLE);
    ledcWrite(PWMChannel, dutyCycle);
  }   
  }

//Read Potenciometer 
void timer1_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    potValue = analogRead(POT_PIN); 
  }
}

//Publish Potenciometer's raw value and voltage
void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msgPot.data = potValue;
    msgVolt.data = potValue*3.3/4095;
    RCSOFTCHECK(rcl_publish(&publisherRawPot, &msgPot, NULL));
    RCSOFTCHECK(rcl_publish(&publisherVolt, &msgVolt, NULL));
  }
}

void setup() {
  set_microros_transports();
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(PWM_PIN, PWMChannel);
  pinMode(LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &publisherRawPot,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "raw_pot"));
  
  RCCHECK(rclc_publisher_init_default(
    &publisherVolt,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "voltage"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriberPWM,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_duty_cycle"));

  // create timer 1
  const unsigned int timer1_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(timer1_timeout),
    timer1_callback));

  // create timer 2
  const unsigned int timer2_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer2,
    &support,
    RCL_MS_TO_NS(timer2_timeout),
    timer2_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriberPWM, &msgPWM, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
