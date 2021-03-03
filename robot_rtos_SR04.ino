/********************************************************************
*                 Obstacle Avoiding Robot Car
*
*            Copyright (C) 2015 by Ulrik Hørlyk Hjort 
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; If not, see <http://www.gnu.org/licenses/>.
**********************************************************************/
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <limits.h>

#define PWM_MOTOR_1 6
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 10
#define PWM_MOTOR_2 5
#define ECHO A0
#define TRIG A1
#define OBSTACLE_LIMIT 10.0

void distanceTask(void *pvParameters);
void imuTask(void *pvParameters);
void driveTask(void *pvParameters);

SemaphoreHandle_t xSerialSemaphore;
static TaskHandle_t driveTaskHandle = NULL;

void setup() {
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(ECHO,INPUT);
  pinMode(TRIG,OUTPUT);
  Serial.begin(9600);

  // Create a mutex semaphore to manage the Serial Port
  if (xSerialSemaphore == NULL) {    
    xSerialSemaphore = xSemaphoreCreateMutex();  
    if (xSerialSemaphore != NULL)
      // Make the Serial Port available for use
      xSemaphoreGive(xSerialSemaphore);  
  }

  xTaskCreate(
    distanceTask
    ,  "Dist"   
    ,  128 
    ,  NULL
    ,  3 
    ,  NULL);
  
  xTaskCreate(
    driveTask
    ,  "Drive" 
    ,  128  
    ,  NULL
    ,  2  
    ,  &driveTaskHandle);    
}

/*
 * distanceTask: Check for obstacles with the HC-SR04 Ultrasonic Sensor within OBSTACLE_LIMIT range 
 * and notify driveTask if obstacle is deteced.
 */
void distanceTask(void *pvParameters __attribute__((unused))) {
  long duration = 0;
  long measuredDistance = 0;

  for(;;) {  
    for (size_t i = 0; i < 5; i++) {  
      digitalWrite(TRIG,LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG,HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG,LOW);    
      duration += pulseIn(ECHO, HIGH);
    }
    
    duration /= 5;
    measuredDistance = duration * 0.034/2;
  
    /*
    if (xSemaphoreTake(xSerialSemaphore,(TickType_t) 5) == pdTRUE) {
        Serial.println(dist);        
        xSemaphoreGive(xSerialSemaphore);
    }
    */
    if (measuredDistance < OBSTACLE_LIMIT) {
      xTaskNotify(driveTaskHandle,0,eNoAction);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/*
 * driveTask runs the state machine:
 * Go FORWARD until an obstacle is detected (blocks in FORWARD state until notified
 * from distanceTask that obstacle is ahead) then change to REVERSE state and reverse
 * for about 1 second and enter TURN state. Do a short turn in TURN state and return to FORWARD state.
 */
void driveTask(void *pvParameters __attribute__((unused))) {
  uint32_t ulNotifiedValue;
  enum States{FORWARD,REVERSE,TURN};
  States state=FORWARD;
  
  for(;;) {  
    switch(state) {
      case FORWARD:        
        forward();
        setSpeed(110);
      
        xTaskNotifyWaitIndexed( 0,
                                ULONG_MAX, /* Clear all notification bits on entry. */
                                ULONG_MAX, /* Reset the notification value to 0 on exit. */
                                &ulNotifiedValue,
                                portMAX_DELAY);
      
        state=REVERSE;
      break;
      
      case REVERSE:
        reverse();
        setSpeed(255);
        vTaskDelay(pdMS_TO_TICKS((1000+random(-500, 50))));
        state=TURN;      
      break;

     case TURN:     
       turnRight();
       setSpeed(110);
       vTaskDelay(pdMS_TO_TICKS((1000+random(-500, 50))));
       state=FORWARD;      
     break; 
    }
  }
}

void _start() {
  digitalWrite(PWM_MOTOR_1,HIGH);
  digitalWrite(PWM_MOTOR_2,HIGH);
}

void _stop() {
  digitalWrite(PWM_MOTOR_1,LOW);
  digitalWrite(PWM_MOTOR_2,LOW);  
}

void forward() {
  _start();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);    
  digitalWrite(IN4,LOW);  
}

void reverse() {
  _start();
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);  
}

void turnRight() {
  _start();
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);    
}

void setSpeed(int s) {
  analogWrite(PWM_MOTOR_1,s);
  analogWrite(PWM_MOTOR_2,s);
}

void loop() {}
