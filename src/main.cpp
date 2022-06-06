/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Sang Tan Truong.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by PIO under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <base64.hpp>
#include <WiFi.h>
#include <esp_camera.h>
#include "camera_pins.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Camera_state_t {
  WAIT = 0,
  CONNECTING,
  CONNECTED,
  STREAMING,
  WAIT_ACK,
  DISCONNECTED,
};

enum Camera_action_t {
  NONE = 0,
  CONNECT,
  CONNECT_SUCCESS,
  START,
  SEND_FRAME,
  ACK,
  STOP,
  TIMEOUT,
  RE_CONNECT,
  LOSS_CONNECT,
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const char* ssid = "Esp32-Hostpot";
const char* password = "12345678";
const uint16_t port = 8000;
const char* host = "192.168.4.1";

WiFiClient Client;

String StreamStart = "START STREAMING";
String StreamACK = "ACKNOWLEDGEMENT";
String StreamStop = "STOP STREAMING";

Camera_state_t CameraState;
int64_t AckTimeout = 10000*1000; //us
int64_t ReconnectTime = 2000*1000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
//Config for the camera
camera_fb_t *capture_handle(HardwareSerial *Serial);
void Serial_init(HardwareSerial *Serial, bool debugEnable);
void Cam_init(int ImageQuality, framesize_t FrameSize, HardwareSerial *Serial);
void Wifi_init(WiFiClient *Client, HardwareSerial *Serial);
void send_one_frame_to_server (WiFiClient *Client);
Camera_action_t Check_control(WiFiClient *Client);
Camera_state_t Update_camera_state (Camera_action_t Action);

//For debuging
void debug_state (Camera_state_t state, HardwareSerial *Serial);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval None
  */

void setup() {
  // put your setup code here, to run once:
  //Start Serial
  Serial_init(&Serial, true);

  //Camera Initialize
  Cam_init(40, FRAMESIZE_CIF, &Serial);
  // //Connect to WiFi hostpot
  Wifi_init(&Client, &Serial);
  //Update FSM to CONNECTING state
  CameraState = Update_camera_state(CONNECT);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  delay(500);
}
/**
  * @brief  The application loop.
  * @retval None
  */

void loop() {
  // put your main code here, to run repeatedly:
  Camera_action_t Control = Check_control(&Client);
  static int64_t TimerStart = esp_timer_get_time();
  static int64_t TimerCurrent = esp_timer_get_time();

  if(CameraState == CONNECTING) {
    if(WiFi.status() == WL_CONNECTED) {
      if(!Client.connect(host, port)) {
      Serial.println("Connection to host failed");
      } else {
        //Update FSM to CONNECTED State
        CameraState = Update_camera_state(CONNECT_SUCCESS);
      }
    }
  }
  if(CameraState == CONNECTED) {
    if(Control == START) {
      //Update FSM to STREAMING
      CameraState = Update_camera_state(START);
    }
    if(!Client.connected()) {
      //Update FSM to DISCONNECTED
      TimerStart = esp_timer_get_time();
      CameraState = Update_camera_state(LOSS_CONNECT);
    }
  }
  if(CameraState == STREAMING) {
    if(Control == STOP) {
      //Update FSM to CONNECTED State
      CameraState = Update_camera_state(CONNECT_SUCCESS);
    } else {
      send_one_frame_to_server(&Client);
      //Update FSM to WAIT_ACK and start timer
      CameraState = Update_camera_state(SEND_FRAME);
      TimerStart = esp_timer_get_time();
    }
  }
  if(CameraState == WAIT_ACK) {
    TimerCurrent = esp_timer_get_time();
    if(Control == ACK) {
      //Update FSM to STREAMING
      CameraState = Update_camera_state(ACK);
    } else {
      if(TimerCurrent - TimerStart > AckTimeout) {
        //Update FSM to Disconnected and Start timer
        CameraState =  Update_camera_state(TIMEOUT);
        Client.stop();
        TimerStart = esp_timer_get_time();
      }
    }
    if(Control == STOP) {
      //Update FSM to CONNECTED
      CameraState = Update_camera_state(STOP);
    }
  }
  if(CameraState == DISCONNECTED) {
    TimerCurrent = esp_timer_get_time();
    if(TimerCurrent - TimerStart > ReconnectTime) {
      //Update FSM to CONNECTING
      CameraState = Update_camera_state(RE_CONNECT);
    }
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Initialize WiFi.
  * @param Client The WiFiClient for TCP connecting with 4G Node.
  * @param Serial The Serial port for debuging.
  * @retval None
  */
void Wifi_init(WiFiClient *Client, HardwareSerial *Serial) {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);
}
/**
  * @brief  Initialize Serial.
  * @param Serial The Serial port for debuging.
  * @param debugEnable true for debug and false for no.
  * @retval None
  */
void Serial_init(HardwareSerial *Serial, bool debugEnable) {
  Serial->setDebugOutput(debugEnable);
  Serial->begin(115200);
  Serial->println("Begin the WiFi");
}
/**
  * @brief  Check control .
  * @param Client The client attribute.
  * @retval Camera_action_t The action from the 4G node.
  */
Camera_action_t Check_control(WiFiClient *Client) {
  Camera_action_t retval = NONE;
  String Control;
  if(Client->available()) {
    Control = Client->readStringUntil('\r');
    if(Control == StreamStart) {
      retval = START;
    }
    if(Control == StreamACK) {
      retval = ACK;
    }
    if(Control == StreamStop) {
      retval = STOP;
    }
  }
  return retval;
}
/**
  * @brief  Update camera state.
  * @param Action The action of the state.
  * @retval Camera_state_t The new state of the camera.
  */
Camera_state_t Update_camera_state (Camera_action_t Action) {
  static Camera_state_t State = WAIT;
  Camera_state_t retval = State;
  switch(State) {
    case WAIT:
      if(Action == CONNECT) {
        retval = CONNECTING;
      }
      break;
    case CONNECTING:
      if(Action == CONNECT_SUCCESS) {
        retval = CONNECTED;
      }
      break;
    case CONNECTED:
      if(Action == START) {
        retval = STREAMING;
      }
      if(Action == LOSS_CONNECT) {
        retval = DISCONNECTED;
      }
      break;
    case STREAMING:
      if(Action == SEND_FRAME) {
        retval = WAIT_ACK;
      }
      if(Action == STOP) {
        retval = CONNECTED;
      }
      break;
    case WAIT_ACK:
      if(Action == ACK) {
        retval = STREAMING;
      }
      if(Action == TIMEOUT) {
        retval = DISCONNECTED;
      }
      if(Action == STOP) {
        retval = CONNECTED;
      }
      break;
    case DISCONNECTED:
      if(Action == RE_CONNECT) {
        retval = CONNECTING;
      }
      break;
    default:
    break;
  }
  State = retval;
  debug_state(State, &Serial);
  return retval;
}
/* Support function ----------------------------------------------------------*/
/**
  * @brief  Send one frame to server.
  * @param Client The Client attribute.
  * @retval void
  */
void send_one_frame_to_server (WiFiClient *Client) {
  unsigned char *SendBuffer;
  SendBuffer = new unsigned char[20000];
  unsigned int LengthAfterEncode;
  camera_fb_t *fb;
  fb = new camera_fb_t;
  fb = capture_handle(&Serial);
  LengthAfterEncode = encode_base64((unsigned char *) fb->buf,
                                   fb->len,
                                   SendBuffer);
  Client->write(SendBuffer, LengthAfterEncode + 1);
  delete[] SendBuffer;
}

/**
  * @brief  The capture image from camera function.
  * @retval esp_err_t
  */
camera_fb_t *capture_handle(HardwareSerial *Serial) {
  camera_fb_t * fb = NULL;
  int64_t fr_start = esp_timer_get_time();

  //Capture the image
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial->println("Camera capture failed");
    while(1);
  }
  int64_t fr_end = esp_timer_get_time();
  Serial->printf("JPG: %uB %ums\n", (uint32_t) fb->len,
                (uint32_t) ((fr_end - fr_start)/1000));
  return fb;
}
void Cam_init(int ImageQuality, framesize_t FrameSize, HardwareSerial *Serial) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  if(psramFound()) {
    Serial->println("PSRAM is Detected");
  } else {
    Serial->println("PSRAM not detected");
  }
  config.jpeg_quality = ImageQuality;
  config.fb_count = 1;
  config.frame_size = FrameSize;

  //Camera initialize
  esp_err_t err = esp_camera_init(&config);
  if(err != ESP_OK) {
    Serial->printf("Camera init failed with error 0x%x", err);
    return;
  } else {
    Serial->println("Camera is initialize successful!");
    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 1);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  }
}

/**
  * @brief  Debug state.
  * @param state The state of FSM.
  * @param Serial The debuging's Serial port.
  * @retval None
  */
void debug_state (Camera_state_t state, HardwareSerial *Serial) {
  String StateMsg = "";
  switch (state)
  {
  case WAIT:
    StateMsg = "wait state";
    break;
  case CONNECTING:
    StateMsg = "Connecting state";
    break;
  case CONNECTED:
    StateMsg = "connected";
    break;
  case STREAMING:
    StateMsg = "Streaming";
    break;
  case WAIT_ACK:
    StateMsg = "Wait ACK State";
    break;
  case DISCONNECTED:
    StateMsg = "Disconnected";
    break;
  default:
    break;
  }
  Serial->print("New state is:");
  Serial->println(StateMsg);
}

/* USER CODE END 1 */
