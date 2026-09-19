#include <Wire.h>
#include <AutoCar.h>

#include <Wire.h>
#include "VL53L1X.h"

#include <Adafruit_NeoPixel.h>

#include <Wire.h>

#include "rgb_lcd.h"

AutoCar car;
int Stop_Detect_Time;
int a;
int _E8_BD_AC_E5_BC_AF_E5_9F_BA_E9_80_9F;
float error;
float kp;
float ki;
float kd;
float integral;
float pre_error;
float base_speed;
float dt;
float current_time;
float last_time;
float derivative;
float output;
int turn_left_time;
int front_detected_time;
int life_detect_time;
int Previous_Water_Time;
#define NUM_LEDS 60

#define BRIGHTNESS 100

Adafruit_NeoPixel led3 = Adafruit_NeoPixel(NUM_LEDS, 3, NEO_RGBW + NEO_KHZ800);

Adafruit_NeoPixel led11 = Adafruit_NeoPixel(NUM_LEDS, 11, NEO_RGBW + NEO_KHZ800);

rgb_lcd lcd;

VL53L1X Distance_Sensor1;
VL53L1X Distance_Sensor2;
VL53L1X Distance_Sensor3;
#define sensor1_io A2

#define sensor2_io A1

#define sensor3_io A0

float get_tof_1_value() {
  Distance_Sensor1.read();
  Serial.print("  L1: ");
  Serial.print(Distance_Sensor1.ranging_data.range_mm);
  delay(100);
  return Distance_Sensor1.ranging_data.range_mm;
}

void Watering() {
	if (get_tof_1_value() <= 250 && get_tof_1_value() > 0) {
		// 设定LED灯颜色
		{
		  uint32_t c11[] = { led11.Color(51, 102, 255) };
		  for(uint16_t i=0; i<led11.numPixels(); i++) {
		    led11.setPixelColor(i, c11[i]);
		    led11.show();
		  }
		}
	} else {
		// 设定LED灯颜色
		{
		  uint32_t c11[] = { led11.Color(0, 0, 0) };
		  for(uint16_t i=0; i<led11.numPixels(); i++) {
		    led11.setPixelColor(i, c11[i]);
		    led11.show();
		  }
		}
	}
}

void _E5_B7_A1_E7_BA_BF_PID() {
	Previous_Water_Time = millis();
	current_time = millis();
	dt = float(((current_time - last_time) / 1000));
	last_time = current_time;
	error = car.getLDistance() - (car.getRDistance() + 5);
	integral = integral + error * dt;
	derivative = (error - pre_error) / dt;
	output = (constrain(((kp * error + ki * integral) + kd * derivative),-100,100));
	pre_error = error;
	digitalWrite(7, HIGH);
	analogWrite(6, ((base_speed - output) - 0));
	digitalWrite(8, HIGH);
	analogWrite(9, ((base_speed - output) - 0));
	digitalWrite(4, LOW);
	analogWrite(5, (base_speed + output));
	digitalWrite(2, LOW);
	analogWrite(10, (base_speed + output));
}

float get_tof_2_value() {
  Distance_Sensor2.read();
  Serial.print("  L2: ");
  Serial.print(Distance_Sensor2.ranging_data.range_mm);
  delay(100);
  return Distance_Sensor2.ranging_data.range_mm;
}

void sign_crosswalk() {
	life_detect_time = millis();
	lcd.setCursor(5, 0);
	lcd.print("50km/h");
	base_speed = 60;
	while (millis() - life_detect_time < 3000) {
		car.probe();
		_E5_B7_A1_E7_BA_BF_PID();
	}
	lcd.clear();
	base_speed = 150;
}

void sign_forward() {
	front_detected_time = millis();
	lcd.setCursor(5, 0);
	lcd.print("60km/h");
	base_speed = 150;
	while (millis() - front_detected_time < 2000) {
		car.probe();
		_E5_B7_A1_E7_BA_BF_PID();
	}
	base_speed = 150;
	lcd.clear();
}

void water() {
	if (get_tof_1_value() < 400) {
		// 设定LED灯颜色
		{
		  uint32_t c3[] = { led3.Color(0, 0, 153) };
		  for(uint16_t i=0; i<led3.numPixels(); i++) {
		    led3.setPixelColor(i, c3[i]);
		    led3.show();
		  }
		}
		// 设定LED灯颜色
		{
		  uint32_t c11[] = { led11.Color(0, 0, 153) };
		  for(uint16_t i=0; i<led11.numPixels(); i++) {
		    led11.setPixelColor(i, c11[i]);
		    led11.show();
		  }
		}
	} else {
		// 设定LED灯颜色
		{
		  uint32_t c3[] = { led3.Color(0, 0, 0) };
		  for(uint16_t i=0; i<led3.numPixels(); i++) {
		    led3.setPixelColor(i, c3[i]);
		    led3.show();
		  }
		}
		// 设定LED灯颜色
		{
		  uint32_t c11[] = { led11.Color(0, 0, 0) };
		  for(uint16_t i=0; i<led11.numPixels(); i++) {
		    led11.setPixelColor(i, c11[i]);
		    led11.show();
		  }
		}
	}
	// 设定LED灯颜色
	{
	  uint32_t c3[] = { led3.Color(0, 0, 0) };
	  for(uint16_t i=0; i<led3.numPixels(); i++) {
	    led3.setPixelColor(i, c3[i]);
	    led3.show();
	  }
	}
	// 设定LED灯颜色
	{
	  uint32_t c11[] = { led11.Color(0, 0, 0) };
	  for(uint16_t i=0; i<led11.numPixels(); i++) {
	    led11.setPixelColor(i, c11[i]);
	    led11.show();
	  }
	}
}

void _E5_B7_A6_E8_BD_AC2() {
	digitalWrite(7, LOW);
	analogWrite(6, 150);
	digitalWrite(8, LOW);
	analogWrite(9, 150);
	digitalWrite(4, LOW);
	analogWrite(5, 150);
	digitalWrite(2, LOW);
	analogWrite(10, 150);
}

void Move_Forward() {
	digitalWrite(7, HIGH);
	analogWrite(6, 100);
	digitalWrite(8, HIGH);
	analogWrite(9, 100);
	digitalWrite(4, LOW);
	analogWrite(5, 100);
	digitalWrite(2, LOW);
	analogWrite(10, 100);
}

void sign__E5_8F_B3_E8_BD_AC() {
	digitalWrite(7, HIGH);
	analogWrite(6, 140);
	digitalWrite(8, HIGH);
	analogWrite(9, 140);
	digitalWrite(4, LOW);
	analogWrite(5, 30);
	digitalWrite(2, LOW);
	analogWrite(10, 30);
	while (car.getSignType() == 9) {
		car.probe();
	}
	delay(0.5 * 1000);
}

void sign__E5_B7_A6_E8_BD_AC() {
	digitalWrite(7, HIGH);
	analogWrite(6, 30);
	digitalWrite(8, HIGH);
	analogWrite(9, 30);
	digitalWrite(4, LOW);
	analogWrite(5, 140);
	digitalWrite(2, LOW);
	analogWrite(10, 140);
	while (car.getSignType() == 8) {
		car.probe();
	}
	delay(0.7 * 1000);
}

void park_right2() {
	digitalWrite(7, HIGH);
	analogWrite(6, 100);
	digitalWrite(8, LOW);
	analogWrite(9, 100);
	digitalWrite(4, HIGH);
	analogWrite(5, 100);
	digitalWrite(2, LOW);
	analogWrite(10, 100);
}

void parking() {
	while (get_tof_1_value() >= 250) {
		lcd.setCursor(0, 5);
		lcd.print(get_tof_1_value());
		park_left();
	}
	stop();
	delay(0.05 * 1000);
	while (get_tof_2_value() < 240) {
		lcd.setCursor(0, 5);
		lcd.print(get_tof_2_value());
		park_backward();
	}
	while (get_tof_2_value() > 350) {
		lcd.setCursor(0, 5);
		lcd.print(get_tof_2_value());
		park_forward();
	}
	stop();
	delay(0.1 * 1000);
	_E5_B7_A6_E8_BD_AC2();
	delay(0.09 * 1000);
	stop();
	delay(0.1 * 1000);
	while (get_tof_1_value() <= 135) {
		lcd.setCursor(0, 5);
		lcd.print(get_tof_1_value());
		park_right();
	}
	stop();
	delay(0.5 * 1000);
	a = -1;
}

void park_forward() {
	digitalWrite(7, HIGH);
	analogWrite(6, 50);
	digitalWrite(8, HIGH);
	analogWrite(9, 50);
	digitalWrite(4, LOW);
	analogWrite(5, 50);
	digitalWrite(2, LOW);
	analogWrite(10, 50);
}

void park_right() {
	digitalWrite(7, HIGH);
	analogWrite(6, 80);
	digitalWrite(8, LOW);
	analogWrite(9, 80);
	digitalWrite(4, HIGH);
	analogWrite(5, 80);
	digitalWrite(2, LOW);
	analogWrite(10, 80);
}

void stop() {
	digitalWrite(7, HIGH);
	analogWrite(6, 0);
	digitalWrite(8, HIGH);
	analogWrite(9, 0);
	digitalWrite(4, LOW);
	analogWrite(5, 0);
	digitalWrite(2, LOW);
	analogWrite(10, 0);
}

void park_backward() {
	digitalWrite(7, LOW);
	analogWrite(6, 50);
	digitalWrite(8, LOW);
	analogWrite(9, 50);
	digitalWrite(4, HIGH);
	analogWrite(5, 50);
	digitalWrite(2, HIGH);
	analogWrite(10, 50);
}

void park_left() {
	digitalWrite(7, LOW);
	analogWrite(6, 100);
	digitalWrite(8, HIGH);
	analogWrite(9, 100);
	digitalWrite(4, LOW);
	analogWrite(5, 100);
	digitalWrite(2, HIGH);
	analogWrite(10, 100);
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Stop_Detect_Time = 0;
  a = 0;
  _E8_BD_AC_E5_BC_AF_E5_9F_BA_E9_80_9F = 150;
  error = 0;
  kp = 0.79;
  ki = 0;
  kd = 0.045;
  integral = 0;
  pre_error = 0;
  base_speed = 150;
  dt = 0;
  current_time = 0;
  last_time = 0;
  derivative = 0;
  output = 0;
  turn_left_time = 0;
  front_detected_time = 0;
  life_detect_time = 0;
  Previous_Water_Time = 0;
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(sensor1_io, OUTPUT);
  pinMode(sensor2_io, OUTPUT);
  pinMode(sensor3_io, OUTPUT);

  delay(20);

  digitalWrite(sensor1_io,LOW);
  digitalWrite(sensor2_io,LOW);
  digitalWrite(sensor3_io,LOW);

  delay(20);

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("VL53L1X Distance Sensor tests in medium distance mode(up to 3m).");

  //init tof 1
  pinMode(sensor1_io, INPUT);
  Serial.println(Distance_Sensor1.getAddress());
  Distance_Sensor1.setTimeout(500);
  if (!Distance_Sensor1.init())
  {
    Serial.println("Failed to initialize VL53L1X Distance_Sensor1!");
    while (1);
  }

  Distance_Sensor1.setDistanceMode(VL53L1X::Short);
  Distance_Sensor1.setMeasurementTimingBudget(50000);
  Distance_Sensor1.startContinuous(50);
  Distance_Sensor1.setAddress(0x2A);
  delay(500);

  //init tof 2
  pinMode(sensor2_io, INPUT);
  Serial.println(Distance_Sensor2.getAddress());
  Distance_Sensor2.setTimeout(500);
  if (!Distance_Sensor2.init())
  {
    Serial.println("Failed to initialize VL53L1X Distance_Sensor2!");
    while (1);
  }

  Distance_Sensor2.setDistanceMode(VL53L1X::Short);
  Distance_Sensor2.setMeasurementTimingBudget(50000);
  Distance_Sensor2.startContinuous(50);
  Distance_Sensor2.setAddress(0x2B);
  delay(500);

  //init tof 3
  pinMode(sensor3_io, INPUT);
  Serial.println(Distance_Sensor3.getAddress());
  Distance_Sensor3.setTimeout(500);
  if (!Distance_Sensor3.init())
  {
    Serial.println("Failed to initialize VL53L1X Distance_Sensor3!");
    while (1);
  }

  Distance_Sensor3.setDistanceMode(VL53L1X::Short);
  Distance_Sensor3.setMeasurementTimingBudget(50000);
  Distance_Sensor3.startContinuous(50);
  Distance_Sensor3.setAddress(0x2C);
  delay(500);

  // 设定LED灯亮度
  led3.setBrightness(BRIGHTNESS);
  led3.begin();
  led3.show();

  // 设定LED灯亮度
  led11.setBrightness(BRIGHTNESS);
  led11.begin();
  led11.show();

  lcd.begin(16, 2);
  car.init();
	car.initMLPModule();
	car.initEdgeDetectModule();
	car.initHoughModule();
	car.configDriverless(1);
	digitalWrite(7, HIGH);
	analogWrite(6, 0);
	digitalWrite(8, HIGH);
	analogWrite(9, 0);
	digitalWrite(4, LOW);
	analogWrite(5, 0);
	digitalWrite(2, LOW);
	analogWrite(10, 0);
	// 设定LED灯颜色
	{
	  uint32_t c3[] = { led3.Color(0, 0, 0) };
	  for(uint16_t i=0; i<led3.numPixels(); i++) {
	    led3.setPixelColor(i, c3[i]);
	    led3.show();
	  }
	}
	// 设定LED灯颜色
	{
	  uint32_t c11[] = { led11.Color(0, 0, 0) };
	  for(uint16_t i=0; i<led11.numPixels(); i++) {
	    led11.setPixelColor(i, c11[i]);
	    led11.show();
	  }
	}
	lcd.home();
}


void loop()
{
  if (a == 0) {
  	car.probe();
  	if (car.getRDistance() > 0 || car.getLDistance() > 0) {
  		_E5_B7_A1_E7_BA_BF_PID();
  	} else if (false) {
  	}
  	if (car.signDetected() && car.getSignRatio() > 1) {
  		switch (car.getSignType()) {
  		 case 1:
  			if (car.getSignRatio() >= 10) {
  				Stop_Detect_Time = millis();
  				while (millis() - Stop_Detect_Time < 500) {
  					car.probe();
  					_E5_B7_A1_E7_BA_BF_PID();
  				}
  				stop();
  				delay(0.5 * 1000);
  				while (get_tof_2_value() < 330) {
  					lcd.setCursor(0, 5);
  					lcd.print(get_tof_2_value());
  					park_backward();
  				}
  				while (get_tof_2_value() >= 700) {
  					lcd.setCursor(0, 5);
  					lcd.print(get_tof_2_value());
  					park_forward();
  				}
  				a = 1;
  			}
  		  break;
  		 case 7:
  			sign_forward();
  		  break;
  		 case 15:
  			park_right();
  			delay(0.2 * 1000);
  			stop();
  			delay(0.1 * 1000);
  			sign_crosswalk();
  		  break;
  		 case 8:
  			if (turn_left_time == 0) {
  				if (car.getSignRatio() >= 12) {
  					sign__E5_B7_A6_E8_BD_AC();
  					delay(0.45 * 1000);
  				}
  			} else {
  				if (car.getSignRatio() >= 35) {
  					sign__E5_B7_A6_E8_BD_AC();
  					delay(0.5 * 1000);
  				}
  			}
  			turn_left_time = turn_left_time + 1;
  		  break;
  		 case 9:
  			if (car.getSignRatio() >= 20) {
  				park_forward();
  				delay(0.1 * 1000);
  				sign__E5_8F_B3_E8_BD_AC();
  				delay(0.65 * 1000);
  			}
  		  break;
  		}
  	}
  	if (millis() - Previous_Water_Time >= 100) {
  		Watering();
  		Previous_Water_Time = millis();
  	}
  } else if (false) {
  }
  if (a == 1) {
  	parking();
  	stop();
  	delay(5 * 1000);
  }

}