//SD
#include <SD.h>
#include <SPI.h>

const int CS_SD = PB0;  //для карты памяти
const int CS_TFT = PA2; //для карты памяти

//звуковое оповещение
#define sound_pin PB12

//таймеры
HardwareTimer *MyTim1 = new HardwareTimer(TIM1); //таймер 5 - 1 минутный
HardwareTimer *MyTim2 = new HardwareTimer(TIM2); //таймер 4 - 1 секундный + PWM
//функция tone() ИСПОЛЬЗУЕТ timer3????? !!!!!!!!!!!!!!!!!!!!
//давления датчик
unsigned long int pressureSA; //сумма всех показаний давления за SA секунд
int pressure;                 //усредненное давление за SA секунд
int SA = 59;                  //не более 59 секунд!!!!!количество секунд для учреднения датчика давления и время реакции на регулирование
#define pressure_pin PB1
#define max_pressure 200

//RTC
#include <uRTCLib.h>
uRTCLib rtc(0x68);
uint32_t hh, mm, ss, dday, mmonth, yyear;
String currentday;

#include <DallasTemperature.h>
// линия данных термодатчиков
#define ONE_WIRE_BUS PA8
// настройка объекта oneWire для связи с любым устройством OneWire
OneWire ds(ONE_WIRE_BUS);

// адреса трех датчиков DS18B20
uint8_t sensor1[8] = {0x28, 0xFF, 0xA2, 0xFE, 0x61, 0x16, 0x03, 0x36};
uint8_t sensor2[8] = {0x28, 0xFF, 0xDE, 0xCB, 0x61, 0x16, 0x03, 0xB3};
byte data1[2];
byte data2[2];
float celsius1, celsius2;

// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// величины регулятора
float gatepoint;  // температура открытия клапана охлаждения
float setpoint;   // заданная величина текущего отбора
float startpoint; // начальная температура отбора
float headpoint;  //температура заершения отбора голов
float midpoint;   //температура завершения отбора тела
float endpoint;   //температура завершения отбора хвостов
float output = 0; // выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
// коэффициенты
float Kp = 0.01;
float Ki = 0.01;
float Kd = 1;
float _dt_s = 1; // время итерации в секундах
float error = 0;
float delta_input = 0;
int pidMin = 0;
int pidMax = 100;
// вспомогательные переменные
int prevInput = 0;
float integral = 0.0;

#include "Ucglib.h"                                // Include Ucglib library to drive the display
Ucglib_ST7735_18x128x160_HWSPI ucg(PA4, PA2, PA3); // (A0, CS, RESET)
int M = 2;                                         // масштаб
const int y = 128;                                 //высота
const int x = 160;                                 //ширина
int down = 40;                                     // верхня точка свободного поля
float graph[x];
int last_y, last_x;
String logo;
int color[3] = {255, 255, 255};

//FUNCTIONS------------------------------------------------------
// --------------  ПИД------------------------------------------
// функция расчёта выходного сигнала
int computePID(float input)
{
    error = setpoint - input;        // ошибка регулирования
    delta_input = prevInput - input; // изменение входного сигнала

    prevInput = input;

    output = 0;
    output += error * Kp;                          // пропорционально ошибке регулирования
    output += delta_input * Kd / _dt_s;            // дифференциальная составляющая
    integral += error * Ki * _dt_s;                // расчёт интегральной составляющей
                                                   // тут можно ограничить интегральную составляющую!
    if (error < 0 && ((error / delta_input) > -5)) //при быстром росте нагрева до значания 5 дельт убираем интегральную
    {
        integral = 0;
    }

    integral = constrain(integral, pidMin, pidMax); // ограничиваем интегральную составляющую
    output += integral;                             // прибавляем интегральную составляющую
    output = constrain(output, pidMin, pidMax);     // ограничиваем выход
    return input;
}

void Alarm()
{
    for (int i = 2000; i < 4000; i = i + 4)
    {
        tone(sound_pin, i);
        delay(10);
        tone(sound_pin, 1000);
        delay(300);
    }
    noTone(sound_pin);
}

//прерывание 60 сек
void pressure_ctrl()
{
    float press = (pressureSA / SA) * 0.0048828125;

    //блок контроля давления
    //если давление позволяет увеличиваем температуру на 1 градус через 100 секунд
    if ((celsius1 < endpoint) && (celsius1 > startpoint))
    {
        if ((pressureSA / SA) < max_pressure) //сравниваем среднее значение давления и максимально возможное
        {
            if (setpoint < endpoint)
            {
                setpoint++;
            }
        }
        else
        {
            if (setpoint > startpoint)
            {
                setpoint--;
            }
        }
    }
    pressureSA = 0; //обнуляем массив
    //давление
    ucg.setColor(0, 0, 0);
    ucg.drawBox(0, 0, 55, 10);
    ucg.setPrintPos(0, 10); // Set position (x,y)
    ucg.setColor(255, 255, 0);
    ucg.print(press);
    ucg.print("V");
}

//прерывание 1 раз в секунду
void rtc_sec()
{

    pressureSA += analogRead(pressure_pin);
    //---------DS1820----
    //----1
    ds.reset();
    ds.select(sensor1);
    ds.write(0x44, 0); // start conversion
    ds.reset();
    ds.select(sensor1);
    ds.write(0xBE); // Read Scratchpad
    data1[0] = ds.read();
    data1[1] = ds.read();
    //---2
    ds.reset();
    ds.select(sensor2);
    ds.write(0x44, 0); // start conversion
    ds.reset();
    ds.select(sensor2);
    ds.write(0xBE); // Read Scratchpad
    data2[0] = ds.read();
    data2[1] = ds.read();

    celsius1 = (float)((data1[1] << 8) | data1[0]) / 16.0;

    rtc.refresh();

    hh = rtc.hour();
    mm = rtc.minute();
    ss = rtc.second();
    dday = rtc.day();
    mmonth = rtc.month();
    yyear = rtc.year();

    //---------DS1820----
    //----1
    ds.reset();
    ds.select(sensor1);
    ds.write(0x44, 0); // start conversion
    ds.reset();
    ds.select(sensor1);
    ds.write(0xBE); // Read Scratchpad
    data1[0] = ds.read();
    data1[1] = ds.read();
    //---2
    ds.reset();
    ds.select(sensor2);
    ds.write(0x44, 0); // start conversion
    ds.reset();
    ds.select(sensor2);
    ds.write(0xBE); // Read Scratchpad
    data2[0] = ds.read();
    data2[1] = ds.read();

    celsius1 = (float)((data1[1] << 8) | data1[0]) / 16.0;

    //-------КОНТРОЛЬ ТЕМПЕРАТУРЫ
    //обновляем блок
    if (celsius1 < 70)
    {
        color[0] = 0;
        color[1] = 255;
        color[2] = 0;
    }
    if (celsius1 > 70 && celsius1 < setpoint)
    {
        color[0] = 255;
        color[1] = 255;
        color[2] = 0;
    }
    if (celsius1 > setpoint)
    {
        color[0] = 255;
        color[1] = 0;
        color[2] = 0;
    }
    ucg.setColor(0, 0, 0);
    ucg.drawBox(0, 10, 160, 30);
    ucg.setColor(color[0], color[1], color[2]);

    ucg.setFont(ucg_font_inb27_tf);
    ucg.setPrintPos(10, 40); // Set position (x,y)
    ucg.print(celsius1);
    ucg.drawGlyph(120, 40, 0, 0xb0); // degree sign
    ucg.setFont(ucg_font_courB12_mf);

    celsius2 = (float)((data2[1] << 8) | data2[0]) / 16.0;

    //---РИСУЕМ ГРАФИКИ-------------
    ucg.setColor(0, 0, 0);
    ucg.drawBox(0, 40, 160, 128 - 40);
    //перезапись старых значений

    for (int xx = x - 1; xx > 0; xx--)
    {
        graph[xx] = graph[xx - 1];
    }

    graph[0] = celsius1; //обновляем новую точку в нуле х

    for (int xx = x - 1; xx > 0; xx--)
    {
        //отображение массива graph
        ucg.setColor(color[0], color[1], color[2]);
        //ucg.drawBox(xx, y - graph[xx] + 20, 1, y);
        ucg.drawPixel(xx, y + 20 - graph[xx]);
        //        ucg.drawHLine(0, y + 20 - graph[xx], 20);
    }
    //    ucg.drawBox(0, y - celsius1 + 20, 10, y);
    //отбображение линии заданной температуры
    ucg.setColor(255, 255, 255);
    ucg.drawHLine(0, y + 20 - setpoint, x);
    ucg.setPrintPos(x - 35, y + 15 - setpoint); // Set position (x,y)
    ucg.print(int(setpoint));
    ucg.drawGlyph(150, y + 15 - setpoint, 0, 0xb0); // degree sign
    //ucg.drawCircle(0, y + 20 - setpoint, 3, UCG_DRAW_ALL);

    /* //давление
    ucg.setColor(10, 10, 10);
    ucg.drawBox(0, 0, 60, 10);
    ucg.setColor(255, 0, 0);
    ucg.setPrintPos(0, 10); // Set position (x,y)
    ucg.print(analogRead(pressure_pin));
*/
    //SD ЗАПИСЬ ЛОГОВ----------------

    String dataString = "";
    dataString = String(setpoint) + "," + String(celsius1) + "," + dday + "." + mmonth + "." + yyear + " " + hh + ":" + mm + ":" + ss + "," + pressure * 5 * 1024;
    String namefile = String(dday) + String(mmonth) + String(yyear) + ".txt";
    File dataFile = SD.open(namefile, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {
        dataFile.println(dataString);
        dataFile.close();
        digitalWrite(PC14, HIGH); //blue
    }
    else
    {
        digitalWrite(PC14, LOW); //blue
    }
    SPI.begin();
    if (digitalRead(PC14) == HIGH)
    {
        ucg.setColor(255, 255, 255);
        ucg.drawBox(152, 2, 10, 7);
        ucg.drawTriangle(150, 3, 154, 0, 154, 3);
        ucg.drawBox(154, 0, 6, 3);
        ucg.setColor(255, 0, 0);
        ucg.drawDisc(155, 5, 2, UCG_DRAW_ALL);
    }

    computePID(celsius1);
    //контроль давления ежесекундный
    if (analogRead(pressure_pin) > max_pressure)
    {
        output = 0;
    }
    ucg.setColor(0, 0, 0);
    ucg.drawBox(70, 0, 80, 10);
    ucg.setColor(0, 0, 255);
    ucg.setPrintPos(70, 10); // Set position (x,y)
    ucg.print("PWM");
    ucg.print(int(output));
    ucg.print("%");
    //задаем заполнение PWM для нагрузки
    MyTim2->setCaptureCompare(1, output, PERCENT_COMPARE_FORMAT); //ВЫХОД РЕГУЛЯТОР НАГРУЗКИ
}

//---------------------------------------------------------------
//SETUP SETUP SETUP-----------------------------------------------
//-------------------------------------
void setup(void) // --------------------------Start of setup
{

    pinMode(PB8, OUTPUT);       //подсветка кнопка без фиксации
    pinMode(PB9, INPUT_PULLUP); //кнопка без фиксации
    pinMode(PC14, OUTPUT);      //blue
    pinMode(PC15, OUTPUT);      //yellow
    pinMode(PA0, OUTPUT);       //red
    //pinMode(PA1, OUTPUT); //green
    pinMode(PB15, INPUT); //секундное прерывание с часов реального времени

    ////SD--
    if (!SD.begin(CS_SD))
    {
        tone(sound_pin, 3000, 1000);
    }
    //noTone(sound_pin);

    //rtc setup
    Wire.begin();

    rtc.alarmClearFlag(URTCLIB_ALARM_1);
    rtc.alarmClearFlag(URTCLIB_ALARM_2);
    rtc.sqwgSetMode(URTCLIB_SQWG_1H);

    //rtc.set(30, 34, 21, 3, 11, 11, 20); УСТАНОВКА ЧАСОВ
    //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)

    // Display setup:

    // Select a type of text background:
    ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    ucg.clearScreen(); // Clear the screen
    ucg.setRotate90(); // Put 90, 180 or 270, or comment to leave default
    ucg.setFont(ucg_font_courB12_mf);
    ucg.setColor(255, 255, 255, 0); // Set color (0,R,G,B)
    ucg.setColor(1, 0, 0, 0);       // Set color of text background (1,R,G,B)

    ///выбор режима ПЕВАК или ВТОРЯК  -------------------------------------------

    int select = 0;
    int ycircle = 24;
    for (int i = 2; i > 1;)
    {
        tone(sound_pin, 1000, 10);
        ucg.clearScreen();       // Clear the screen
        ucg.setPrintPos(28, 60); // Set position (x,y)
        ucg.print("VODKA 3S");
        ucg.setPrintPos(28, 85); // Set position (x,y)
        ucg.print("PERVAK 1S");
        ucg.setColor(255, 255, 255);
        ucg.setPrintPos(28, 30); // Set position (x,y)
        ucg.print("PUSH BUTTON");

        if (digitalRead(PB9) == LOW)
        {
            select++;
            if (select > 0 && select < 3)
            {
                ycircle = 80;
            }
            if (select > 3)
            {
                ycircle = 55;
            }
        }
        else
        {
            if (select > 0 && select < 3)
            {
                i = 0; //режим ПЕРВИЧНАЯ ПЕРЕГОНКА PERVAK
                digitalWrite(PB8, LOW);
                ycircle = 80;
            }
            if (select > 3)
            {
                ycircle = 55;
                i = 0; //режим ВТОРИЧНАЯ ПЕРЕГОНКА VODKA
                digitalWrite(PB8, HIGH);
            }
        }
        ucg.setColor(0, 255, 0);
        ucg.drawDisc(12, ycircle, 8, UCG_DRAW_ALL);
        delay(1000);
    }

    if (select > 0 && select < 3) //PERVAK
    {
        startpoint = 86; // заданная величина начала отбора
        headpoint = 98;  //температура заершения отбора голов
        midpoint = 98;   //температура завершения отбора тела
        endpoint = 98;   //температура завершения отбора хвостов
        logo = "PERVAK";
    }
    else //VODKA
    {
        startpoint = 84; // заданная величина начала отбора
        headpoint = 88;  //температура заершения отбора голов
        midpoint = 94;   //температура завершения отбора тела
        endpoint = 98;   //температура завершения отбора хвостов
        logo = "VODKA";
    }

    setpoint = startpoint;
    //заполняем массив средних значений давления в начале

    for (int i = 0; i < SA; i++)
    {
        pressureSA += analogRead(pressure_pin);
    }

    ucg.clearScreen(); // Clear the screen
    //--------------------------------------------------

    //ШИМ

    MyTim2->pause();
    MyTim2->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA0);       //для PA0 первый канал таймера 4
    MyTim2->setOverflow(1000000, MICROSEC_FORMAT);            // 1000000 microseconds = 1s
    MyTim2->setCaptureCompare(1, 20, PERCENT_COMPARE_FORMAT); // 20%
    MyTim2->attachInterrupt(rtc_sec);

    MyTim1->pause();
    MyTim1->setOverflow(SA * 1000000, MICROSEC_FORMAT); // 60000000 microseconds = 60sДАВЛЕНИЕ СДЕЛАТЬ СРЕДНЕЕ АРИФМ ЗА ВСЕ ВРЕМЯ
    MyTim1->attachInterrupt(pressure_ctrl);

    MyTim2->refresh();
    MyTim1->refresh();
    MyTim2->resume();
    MyTim1->resume();

    //attachInterrupt(PB15, rtc_sec, RISING); //от часов реального времени
}

//------SETUP End----------------------------------------------

void loop(void) // Start of loop
{
    //    noInterrupts();
    if (celsius1 > 98)
    {
        output = 0;
        tone(sound_pin, 1000);
    }
    else
    {
        noTone(sound_pin);
    }
    //  interrupts();
}
