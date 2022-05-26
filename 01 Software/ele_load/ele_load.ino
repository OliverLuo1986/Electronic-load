#include <lvgl.h>
#include <TFT_eSPI.h>
#include <lv_examples/lv_examples.h>
#include <Wire.h>
#include <Bounce2.h>
#include <INA.h>
#include <SPI.h>
#include <Ticker.h>
#include "B3950.h"
#include "MD_REncoder.h"


#define X_START_OFFSET 2
#define OUT_EN_PIN  25
#define FAN_CTL_PIN 14

#define LCD_BL      26

#define PinA 19  // 左键引脚
#define PinB 23  // 右键引脚
#define PinC 26  // 确定键引脚

static MD_REncoder R = MD_REncoder(PinA, PinB);   //旋转编码器

TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];



INA_Class         INA; 

#define MCP4725_ADDR 0x60 

int8_t request_index=0;

volatile uint8_t  deviceNumber    = UINT8_MAX;  ///< Device Number to use in example
volatile uint64_t sumBusMillVolts = 0;          ///< Sum of bus voltage readings
volatile int64_t  sumBusMicroAmps = 0;          ///< Sum of bus amperage readings
volatile uint8_t  readings        = 0;          ///< Number of measurements taken
volatile int64_t  mAH = 0;
volatile double  wH = 0;
uint8_t rotate = 3;
bool touch = false;
bool power_on = false;

static int h=0,m=0,s=0;

uint32_t curre=0;

lv_obj_t * vol_label;
lv_obj_t * cur_label;
lv_obj_t * pow_label;
lv_obj_t * cap_label;
lv_obj_t * temp_label;
lv_obj_t * time_label;
lv_obj_t * stopvol_label;
lv_obj_t * onoff_label;

lv_obj_t * label_v;
lv_obj_t * label_a;
lv_obj_t * label_w;
lv_obj_t * label_mah;
lv_obj_t * label_t;

Ticker ticker1;

int adc_temp = 0;
int res_temp = 0;


void ina226_read() {
  /*!
    @brief Interrupt service routine for the INA pin
    @details Routine is called whenever the INA_ALERT_PIN changes value
  */
 
  sumBusMillVolts += INA.getBusMilliVolts(deviceNumber);  // Add current value to sum
  sumBusMicroAmps += INA.getBusMicroAmps(deviceNumber);   // Add current value to sum
  readings++;

}  // of ISR for handling interrupts


void time_update_task()
{
  static long ms,lastms = millis();
  char tmp[10];
  
  if(power_on)
  {
    ms = millis();
    if((ms-lastms) >= 1000)
    {
      s++;
      if(s>=60)
      {
        m++;
        if(m>=60)
        {
          h++;
          m = 0;
        }
        s=0;
      }
      
      sprintf(tmp,"%d:%02d:%02d", h,m,s);
      lv_label_set_text(time_label,tmp);
  
      lastms = ms;
    }
  }
}

int get_cur_average(int cur)
{
#define NUM   10  
  static int buff[NUM];
  static char p = 0;
  char i;
  int sum=0;

  buff[p] = cur;
  p++;
  if(p>=NUM)
    p = 0;

  for(i = 0;i<NUM;i++)
  {
      sum += buff[i];
  }

  return sum/NUM;
}

void ina266_task()
{
  char tmp[64];
  static long lastMillis = millis();  // Store the last time we printed something
  volatile uint64_t vol,cur, wat;
  double v,a,w,wh;
  long ms;
  int i,tempture,vol_temp;


 
  {
    ms = millis();
   
    if((ms - lastMillis) >= 200 )
    {
      vol = INA.getBusMilliVolts(deviceNumber);
      cur = INA.getBusMicroAmps(deviceNumber)/1000;
      wat = INA.getBusMicroWatts(deviceNumber);
      ms = ms - lastMillis;
      lastMillis = millis();
  
      if(cur > 20*1000)
        cur = 0;
  
      v = vol / 1000.0;
      if(v>10)
        sprintf(tmp,"#00FFFF %0.2f",v);
      else
        sprintf(tmp,"#00FFFF %0.3f",v);
      lv_label_set_text(vol_label,tmp);

      //int c_a = get_cur_average(cur);
      //Serial.printf("c_a:%dmA\n", c_a);
      cur = get_cur_average(cur);
      a = cur/1000.0;
      if(a > 10)
        sprintf(tmp,"#0000FF %0.2f",a);
      else
        sprintf(tmp,"#0000FF %0.3f",a);
      lv_label_set_text(cur_label,tmp);
  
      w = v*a;
      if(w>10)
        sprintf(tmp,"#FF0000 %0.2f",w);
      else 
        sprintf(tmp,"#FF0000 %0.3f",w); 
      lv_label_set_text(pow_label,tmp);

      
      wH += w*ms;
      //if(wH >10)
        sprintf(tmp,"#00FF00 %0.2f", wH/(60*60*1000));
      //else
        //sprintf(tmp,"#00FF00 %0.3f", wH/(60*60*1000));
      lv_label_set_text(cap_label,tmp);
  
      sprintf(tmp,"vol:%0.3fV cur:%0.3fA, power:%0.3fW %0.3fwh\n", v, a, w,wH/(60*60*1000));
      //Serial.print(tmp);

      adc_temp = analogRead(38);
      vol_temp = 3419*adc_temp/4095;
      res_temp = 3419*100*10/vol_temp-100*10;
      //Serial.println(res_temp);

      sprintf(tmp,"#00FF00 %d", tempture);

      for(i=0;i<140;i++)
      {
        if(res_temp>B3950[i])
          break;
      }
      tempture = i-20-1;

      sprintf(tmp,"#00FF00 %d", tempture);

      if(tempture >=60)
      {
        sprintf(tmp,"#0000FF %d", tempture);
        lv_label_set_text(label_t,"#0000FF C");
      }
      else
      {
        sprintf(tmp,"#00FF00 %d", tempture);
        lv_label_set_text(label_t,"#00FF00 C");
      }
      
      lv_label_set_text(temp_label,tmp);
      //Serial.printf("temp adc:%d vol:%dmV  Res:%d  temp:%d\n", adc_temp, vol_temp, res_temp/10, tempture);

      if(tempture >= 90)
      {
        // disable output
        power_on = false;
        digitalWrite(OUT_EN_PIN,HIGH);
        lv_label_set_text(onoff_label,"#0000FF OFF");
      }
      else
      {
        //if(power_on)
        {
          if(tempture < 80)
          {
            // enable output
            //digitalWrite(OUT_EN_PIN,LOW); 
            //lv_label_set_text(onoff_label,"ON");           
          }
        }
      }

      if(tempture >= 50)
      {
        //ledcWrite(0,1023);
        digitalWrite(FAN_CTL_PIN, HIGH);
      }
      else if(tempture < 45)
      {
        //ledcWrite(0,0);
        digitalWrite(FAN_CTL_PIN, LOW);
      }

         adc_temp = analogRead(37);
         vol_temp = 3419*adc_temp/4095;
         int vol_bat = vol_temp*1428/1000;
         Serial.printf("adc:%d vol:%dmV bat vol:%d\n", adc_temp, vol_temp, vol_bat);
    } 
  }
}


void lvgl_task(void *)
{
  lv_task_handler();
}



#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char* file, uint32_t line, const char* fun, const char* dsc)
{
	Serial.printf("%s@%d %s->%s\r\n", file, line, fun, dsc);
	Serial.flush();
}
#endif






// 编码器扫描函数，用于判断左右及按下状态
uint8_t Encoder_Scan()
{
  if (digitalRead(PinC) == LOW)
  {
    //Serial.printf("[Navigation] click\n");
    return 1;
  }
  uint8_t x = R.read();
  if (x)
  {
    // x == DIR_CW;
    if (x == DIR_CW ) {
      Serial.printf("[Navigation] next\n"); 
      return 2;
    }
    else
    {
      Serial.printf("[Navigation] prev\n");
      return 3;
    }
    
  }
  return 0;// 没按下返回0
}

// 编码器状态更新函数
static bool my_encoder_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
  static uint32_t last_key = 0;
  uint8_t act_enc = Encoder_Scan();

  Serial.printf("my_encoder_read\n");
  
  if(act_enc != 0) {
      switch(act_enc) {
          case 1:
              act_enc = LV_KEY_ENTER;
              //data->state = LV_INDEV_STATE_PRESSED; 
              break;
          case 2:
              act_enc = LV_KEY_RIGHT;
              //data->state = LV_INDEV_STATE_RELEASED;
              data->enc_diff++;
              break;
          case 3:
              act_enc = LV_KEY_LEFT;
              //data->state = LV_INDEV_STATE_RELEASED;
              data->enc_diff--;
              break;
      }
      last_key = (uint32_t)act_enc;
  }
  data->key = last_key;

  return true;
}

// 按键初识化函数
static void my_encoder_init()
{
  R.begin();
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);
  pinMode(PinC, INPUT_PULLUP);
}

// 输入设备初识化函数
void lv_port_indev_init(void)
{
  // 初识化编码器
  my_encoder_init();
  // 注册输入设备
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_ENCODER;
  indev_drv.read_cb = my_encoder_read;
  lv_indev_drv_register( &indev_drv );
}


/* Display flushing */
void my_disp_flush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p)
{
	uint32_t w = (area->x2 - area->x1 + 1);
	uint32_t h = (area->y2 - area->y1 + 1);

	tft.startWrite();
	tft.setAddrWindow(area->x1, area->y1, w, h);
	tft.pushColors(&color_p->full, w * h, true);
	tft.endWrite();

	lv_disp_flush_ready(disp);
}

static void list_event_handler(lv_obj_t * obj, lv_event_t event)
{
  if (event == LV_EVENT_CLICKED) {
    printf("Clicked: %s\n", lv_list_get_btn_text(obj));
  }
}

void ui_init()
{
  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 160;
  disp_drv.ver_res = 80;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  lv_obj_t* bgk;
  bgk = lv_obj_create(lv_scr_act(), NULL);//创建对象
  lv_obj_clean_style_list(bgk, LV_OBJ_PART_MAIN); //清空对象风格
  lv_obj_set_style_local_bg_opa(bgk, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_100);//设置颜色覆盖度100%，数值越低，颜色越透。
  lv_obj_set_style_local_bg_color(bgk, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLACK);//设置背景颜色为绿色
  lv_obj_set_size(bgk, 160, 80);//设置覆盖大小  

  // 字体
  static lv_style_t font_style;
  lv_style_init(&font_style);
  lv_style_set_text_font(&font_style, LV_STATE_DEFAULT, &lv_font_montserrat_20);

  // 电压
  vol_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(vol_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(vol_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(vol_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label1, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(vol_label, 60);
  //lv_obj_align(label1, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 2);
  lv_label_set_text(vol_label,"0.000");
  lv_obj_set_pos(vol_label, X_START_OFFSET+0, 0);

  // 电压单位V
  label_v = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_recolor(label_v, true); 
  lv_label_set_text(label_v,"#00FFFF V");
  lv_obj_set_pos(label_v, X_START_OFFSET+62, 4);  
  
  // 电流
  cur_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(cur_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(cur_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(cur_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label2, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(cur_label, 60);
  lv_label_set_text(cur_label,"0.000");
  //lv_obj_align(label2, label1, LV_ALIGN_OUT_RIGHT_MID, 0, 2); 
  lv_obj_set_pos(cur_label, X_START_OFFSET+0, 20); 

  // 电流单位A
  label_a = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_recolor(label_a, true); 
  lv_label_set_text(label_a,"#0000FF A");
  lv_obj_set_pos(label_a, X_START_OFFSET+62, 24);

  // 功率
  pow_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(pow_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(pow_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(pow_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label3, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(pow_label, 60);
  lv_label_set_text(pow_label,"0.000");
  lv_obj_set_pos(pow_label, X_START_OFFSET+0, 40);
  //lv_obj_align(label3, label2, LV_ALIGN_OUT_BOTTOM_MID, 0, 2); 

  // 功率单位W
  label_w = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_recolor(label_w, true);
  lv_label_set_text(label_w,"#FF0000 W");
  lv_obj_set_pos(label_w, X_START_OFFSET+62, 45);

  // 容量
  cap_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(cap_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(cap_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(cap_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label4, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(cap_label, 60);
  lv_label_set_text(cap_label,"0.00");
  lv_obj_set_pos(cap_label, X_START_OFFSET+0, 60);
  //lv_obj_align(label4, label3, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);  

  // 容量mah
  label_mah = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_recolor(label_mah, true);
  lv_label_set_text(label_mah,"#00FF00 Wh");
  //lv_label_set_text(label_mah,"#00FF00 C");
  lv_obj_set_pos(label_mah, X_START_OFFSET+54, 64);    


  // 时间
  onoff_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(onoff_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(onoff_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(onoff_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label1, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(onoff_label, 80);
  //lv_obj_align(label1, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 2);
  lv_label_set_text(onoff_label,"#0000FF OFF");
  lv_obj_set_pos(onoff_label, X_START_OFFSET+100, 0);


  // 时间
  time_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(time_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(time_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(time_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label1, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(time_label, 80);
  //lv_obj_align(label1, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 2);
  lv_label_set_text(time_label,"0:00:00");
  lv_obj_set_pos(time_label, X_START_OFFSET+80, 20);


  // 截止电压
  stopvol_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(stopvol_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(stopvol_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(stopvol_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label1, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(stopvol_label, 60);
  //lv_obj_align(label1, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 2);
  lv_label_set_text(stopvol_label,"2.80");
  lv_obj_set_pos(stopvol_label, X_START_OFFSET+90, 40);

  // 电压单位V
  label_v = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_recolor(label_v, true); 
  lv_label_set_text(label_v,"V");
  lv_obj_set_pos(label_v, X_START_OFFSET+140, 44); 

  // 温度
  temp_label = lv_label_create(lv_scr_act(), NULL);
  lv_obj_add_style(temp_label,LV_LABEL_PART_MAIN, &font_style);
  lv_label_set_long_mode(temp_label, LV_LABEL_LONG_SROLL_CIRC);     /*Break the long lines*/
  lv_label_set_recolor(temp_label, true);                      /*Enable re-coloring by commands in the text*/
  //lv_label_set_align(label1, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
  lv_obj_set_width(temp_label, 60);
  //lv_obj_align(label1, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 2);
  lv_label_set_text(temp_label,"00");
  lv_obj_set_pos(temp_label, X_START_OFFSET+106, 60);

  // 
  label_t = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_recolor(label_t, true); 
  lv_label_set_text(label_t,"#00FF00 C");
  lv_obj_set_pos(label_t, X_START_OFFSET+140, 64);   

     
}

void setup()
{
	Serial.begin(115200); /* prepare for possible serial debug */ 

  pinMode(OUT_EN_PIN, OUTPUT);
  digitalWrite(OUT_EN_PIN,HIGH);

  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(4, OUTPUT);

  digitalWrite(33,LOW);
  digitalWrite(32,HIGH);
  digitalWrite(4,LOW);

  pinMode(FAN_CTL_PIN, OUTPUT);
  digitalWrite(FAN_CTL_PIN, LOW);
  //ledcSetup(0, 35000, 10); //通道0， 35KHz，10位解析度
  //ledcAttachPin(FAN_CTL_PIN, 0); //pin14定义为通道0的输出引脚  
  //ledcWrite(0,0);

  //pinMode(27, OUTPUT);
  //digitalWrite(27,LOW);    

	lv_init();

#if LV_USE_LOG != 0
	lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

	tft.begin(); /* TFT init */
	tft.setRotation(3); /* mirror */

	lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  lv_port_indev_init();

  ui_init();

#if 1  
  Wire.begin();
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                     // cmd to update the DAC
  int dac = 0;
  Wire.write(dac >> 4);        // the 8 most significant bits...
  Wire.write(dac << 4); // the 4 least significant bits...
  
  Wire.endTransmission();
#endif

  Wire.begin();
  uint8_t devicesFound = 0;
  //lv_label_set_text(label1,"INA226 Init...");
  while (deviceNumber == UINT8_MAX)  // Loop until we find the first device
  {
    devicesFound = INA.begin(10, 5000);  // +/- 1 Amps maximum for 0.01 Ohm resistor
    Serial.println(INA.getDeviceName(devicesFound - 1));
    for (uint8_t i = 0; i < devicesFound; i++) {
      /* Change the "INA226" in the following statement to whatever device you have attached and
         want to measure */
      if (strcmp(INA.getDeviceName(i), "INA226") == 0) {
        deviceNumber = i;
        INA.reset(deviceNumber);  // Reset device to default settings
        //lv_label_set_text(label1,"INA226 Init OK");
        break;
      }  // of if-then we have found an INA226
    }    // of for-next loop through all devices found
    if (deviceNumber == UINT8_MAX) {
      Serial.print(F("No INA found. Waiting 5s and retrying...\n"));
      //lv_label_set_text(label1,"No INA found. Waiting 5s and retrying...");
      delay(5000);
    }  // of if-then no INA226 found
  }    // of if-then no device found
  Serial.print(F("Found INA at device number "));
  Serial.println(deviceNumber);
  Serial.println();
  INA.setAveraging(4, deviceNumber); 
  INA.setBusConversion(8244, deviceNumber);             // Maximum conversion time 8.244ms
  INA.setShuntConversion(8244, deviceNumber);           // Maximum conversion time 8.244ms
  INA.setMode(INA_MODE_CONTINUOUS_BOTH, deviceNumber);  // Bus/shunt measured continuously

/*
  xTaskCreatePinnedToCore(
    time_update
    ,  "time_update" //任务名
    ,  1024  // 栈大小
    ,  NULL
    ,  0  // 任务优先级
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
    */
}

void button_handle_task()
{
  int act = Encoder_Scan();
  static long ms,lastms = 0;
  
  switch(act)
  {
    case 1:
    {
      ms = millis();
      if((ms - lastms) > 500)
      {
        Serial.printf("button down\n");
        lastms = ms;

        power_on = !power_on;
        if(power_on)
        {
            // enable output
            digitalWrite(OUT_EN_PIN,LOW); 
            lv_label_set_text(onoff_label,"#00FF00 ON"); 

            h = m = s = 0;
            wH = 0;
        }
        else
        {
            // disable output
            digitalWrite(OUT_EN_PIN,HIGH); 
            lv_label_set_text(onoff_label,"#0000FF OFF"); 
        }
      }
      break;
    }
    case 2:
    { 
      curre+=10; 

      Serial.printf("curre:%d\n", curre);  
      Wire.begin();
      Wire.beginTransmission(MCP4725_ADDR);
      Wire.write(64);                     // cmd to update the DAC
      int dac = curre;
      Wire.write(dac >> 4);        // the 8 most significant bits...
      Wire.write(dac << 4); // the 4 least significant bits...
      
      Wire.endTransmission();      
      break;
    }
    case 3:
    {
      if(curre>10)
        curre-=10;

      Serial.printf("curre:%d\n", curre);  
      Wire.begin();
      Wire.beginTransmission(MCP4725_ADDR);
      Wire.write(64);                     // cmd to update the DAC
      int dac = curre;
      Wire.write(dac >> 4);        // the 8 most significant bits...
      Wire.write(dac << 4); // the 4 least significant bits...
      
      Wire.endTransmission();        
      break;
    }
    default:break;
  }
}

void loop()
{
	lv_task_handler(); /* let the GUI do its work */
  
  ina266_task();

  time_update_task();

  button_handle_task();

	delay(5);
}
