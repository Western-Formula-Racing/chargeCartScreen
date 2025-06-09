#include <Arduino.h>
#include <SPI.h>
#include <stdarg.h>
// Definitions for our circuit board and display.
#include "CFA10100_defines.h"
#include <ESP32-TWAI-CAN.hpp>
#include "Data.h"
#include "CAN_data.h"

#if BUILD_SD
#include <SD.h>
#endif

// The very simple EVE library files
#include "EVE_base.h"
#include "EVE_draw.h"

// Our demonstrations of various EVE functions
#include "demos.h"

#define CAN_TX 5
#define CAN_RX 4

CanFrame rxFrame;

void setup()
{
  // #if (DEBUG_LEVEL != DEBUG_NONE)
  //   // Initialize UART for debugging messages
  //   Serial.begin(115200);
  //   delay(1000);
  //   Serial.println("Serial working!");
  // #endif // (DEBUG_LEVEL != DEBUG_NONE)

  
  Serial.begin(115200);
  delay(5000);  
  //Serial.println("Serial working!");
  DBG_STAT("Begin\n");

  // Configure GPIO for EVE_CS_NOT
  gpio_pad_select_gpio(EVE_CS_NOT);
  gpio_set_direction(EVE_CS_NOT, GPIO_MODE_OUTPUT);
  gpio_set_level(EVE_CS_NOT, 1); // Default to high (not selected)
  delay(10);

  // Configure GPIO for EVE_PD_NOT
  gpio_pad_select_gpio(EVE_PD_NOT);
  gpio_set_direction(EVE_PD_NOT, GPIO_MODE_OUTPUT);
  gpio_set_level(EVE_PD_NOT, 1); // Default to high (power on)
  delay(10);

  //Initialize port directions
  // EVE interrupt output (not used in this example)
  pinMode(EVE_INT, INPUT_PULLUP);
  // EVE Power Down (reset) input
  pinMode(EVE_PD_NOT, OUTPUT);
  // EVE SPI bus CS# input
  pinMode(EVE_CS_NOT, OUTPUT);
  // USD card CS
  //pinMode(SD_CS, OUTPUT);
  // Optional pin used for LED or oscilloscope debugging.
  pinMode(DEBUG_LED, OUTPUT);

  // Initialize SPI
  SPI.begin(18,13,11,15); // SCK, MISO, MOSI, CS
  //SPI.begin(18, 13, 11, 15); // SCK, MISO, MOSI, CS

  //Bump the clock to 8MHz. Appears to be the maximum.
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  DBG_GEEK("SPI initialzed to: 8MHz\n");

  //See if we can find the FTDI/BridgeTek EVE processor
  if(0 != EVE_Initialize())
    {
    DBG_STAT("Failed to initialize %s8%02X. Stopping.\n",EVE_DEVICE<0x14?"FT":"BT",EVE_DEVICE);
    while(1);
    }
  else
    {
    DBG_STAT("%s8%02X initialized.\n",EVE_DEVICE<0x14?"FT":"BT",EVE_DEVICE);
    }
  
    
  setup_can();

  canQueue = xQueueCreate(32, sizeof(twai_message_t));
  if (canQueue == NULL) {
      Serial.println("Failed to create CAN queue!");
  }

  xTaskCreatePinnedToCore(can_receiver_task, "CAN_RX", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(can_parser_task, "CAN_Parser", 4096, NULL, 1, NULL, 1);

} 

void can_receiver_task(void* pvParameters) {
    twai_message_t message;
    while (true) {
        if (twai_receive(&message, 0) == ESP_OK) {
            xQueueSend(canQueue, &message, 0);  // drop if full
        }
        vTaskDelay(1);
    }
}

void can_parser_task(void* pvParameters) {
    twai_message_t message;
    while (true) {
        while (xQueueReceive(canQueue, &message, 0) == pdTRUE) {
            parse_can_message(&message);
        }
        vTaskDelay(1);  // Small yield between bursts
    }
}


uint8_t screen = 0;

uint16_t Button(uint16_t FWo, int16_t x0, int16_t y0, int16_t x1, int16_t y1, String text,
                int16_t x_point, int16_t y_point, uint32_t color, void (*callback)());
uint16_t gridBox(uint16_t FWo, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int8_t num, float value, uint32_t color);
uint16_t voltageGridBox(uint16_t FWo, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int8_t num, float value, uint32_t color);

void decrementScreen(int8_t page);
void incrementScreen(int8_t page);
void homeScreen();
void setScreen(int8_t num);
uint16_t header(uint16_t FWo, int16_t x_point, int16_t y_point, bool home, char *title);

// float moduleData[5][2][21] = {
//     // Module 1: temperatures, voltages
//     {
//         {42.3, 18.7, 56.2, 35.8, 47.1, 28.9, 54.6, 31.2, 49.8, 25.4, 38.9, 41.0, 52.1, 34.7, 28.5, 29.4, 39.1, 47.8, 51.0, 23.5, 50.2}, // Temperatures
//         {3.54, 1.56, 4.69, 2.98, 3.92, 2.11, 4.05, 3.33, 3.11, 3.72, 4.06, 2.99, 4.12, 3.68, 3.48, 3.74, 2.99, 4.00, 3.91, 3.02, 3.87}  // Voltages
//     },
//     // Module 2: temperatures, voltages
//     {
//         {22.9, 60.4, 28.5, 39.2, 51.7, 44.3, 32.8, 45.0, 37.1, 40.2, 55.4, 29.5, 33.1, 41.9, 49.3, 42.8, 40.0, 35.2, 37.7, 52.9, 45.3}, // Temperatures
//         {1.91, 5.00, 2.38, 3.27, 4.31, 3.76, 2.84, 3.89, 2.59, 4.13, 3.95, 2.68, 3.11, 3.90, 4.24, 3.42, 3.08, 4.05, 4.01, 2.96, 3.65}  // Voltages
//     },
//     // Module 3: temperatures, voltages
//     {
//         {19.8, 64.3, 30.1, 45.6, 33.9, 41.2, 38.5, 44.1, 39.6, 49.8, 29.7, 33.5, 43.3, 46.8, 50.0, 32.1, 40.6, 34.9, 45.5, 47.9, 28.3}, // Temperatures
//         {1.65, 4.36, 2.51, 3.80, 2.83, 4.12, 1.98, 3.23, 3.05, 2.79, 3.32, 3.20, 4.15, 4.05, 3.58, 3.39, 3.88, 3.43, 3.94, 3.68, 2.79}  // Voltages
//     },
//     // Module 4: temperatures, voltages
//     {
//         {25.4, 41.5, 59.2, 21.6, 48.3, 38.9, 45.2, 44.0, 40.3, 37.8, 49.0, 32.6, 36.5, 42.7, 30.8, 45.3, 41.4, 48.7, 32.3, 40.5, 35.2}, // Temperatures
//         {2.17, 3.75, 4.13, 3.27, 3.94, 3.11, 2.98, 4.12, 3.28, 4.01, 3.68, 3.54, 3.77, 4.10, 3.81, 4.05, 3.42, 3.69, 2.97, 3.61, 3.80}  // Voltages
//     },
//     // Module 5: temperatures, voltages
//     {
//         {30.4, 50.2, 43.5, 32.8, 36.7, 41.1, 46.9, 49.8, 51.4, 40.6, 45.5, 30.1, 39.3, 47.6, 38.3, 42.2, 40.7, 49.0, 37.4, 34.8, 51.2}, // Temperatures
//         {3.48, 3.25, 2.91, 3.67, 4.01, 3.81, 3.47, 3.62, 3.87, 3.68, 3.55, 3.98, 4.02, 3.71, 4.08, 4.10, 3.72, 3.88, 3.99, 3.67, 3.88}  // Voltages
//     }};

void loop()
{
  DBG_GEEK("Loop initialization.\n");

  // Get the current write pointer from the EVE
  uint16_t FWo;
  FWo = EVE_REG_Read_16(EVE_REG_CMD_WRITE);
  DBG_GEEK("Initial Offset Read: 0x%04X = %u\n", FWo, FWo);

  // Keep track of the RAM_G memory allocation
  uint32_t RAM_G_Unused_Start;
  RAM_G_Unused_Start = 0;
  DBG_GEEK("Initial RAM_G: 0x%08lX = %lu\n", RAM_G_Unused_Start, RAM_G_Unused_Start);

  // We need to keep track of the bitmap handles and where they are used.
  //
  // By default, bitmap handles 16 to 31 are used for built-in font and 15
  // is used as scratch bitmap handle by co-processor engine commands
  // CMD_GRADIENT, CMD_BUTTON and CMD_KEYS.
  //
  // For whatever reason, I am going to allocate handles from 14 to 0.
  uint8_t next_bitmap_handle_available;
  next_bitmap_handle_available = 14;

  DBG_GEEK("EVE_Initialize_Flash() . . . ");
  FWo = EVE_Initialize_Flash(FWo);
  DBG_GEEK("done.\n");

  uint8_t flash_status;
  flash_status = EVE_REG_Read_8(EVE_REG_FLASH_STATUS);
  DBG_GEEK_Decode_Flash_Status(flash_status);

  uint8_t points_touched_mask;
  int16_t x_points[1];
  int16_t y_points[1];

  DBG_STAT("Initialization complete, entering main loop.\n");

  while (1)
  {
    FWo = Wait_for_EVE_Execution_Complete(FWo);      // Start display list
    FWo = EVE_Cmd_Dat_0(FWo, (EVE_ENC_CMD_DLSTART)); // Clear the screen - this and the previous prevent artifacts between lists
    FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_CLEAR(1 /*CLR_COL*/, 1 /*CLR_STN*/, 1 /*CLR_TAG*/));

    // set to white
    FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255, 255, 255));
    // Rectangle
    FWo = EVE_Filled_Rectangle(FWo, 0, 0, LCD_WIDTH, LCD_HEIGHT);

    //---------------Start of touch logic-----------------------------------
    points_touched_mask = Read_Touch(x_points, y_points);
    DBG_GEEK("Points touched mask: %d", points_touched_mask);

    if (0 != points_touched_mask)
    {
      // Loop through the possible touch points
      uint8_t mask;
      mask = 0x01;

      DBG_GEEK("Looing for touch\n");
      if (0 != (points_touched_mask & mask))
      {
        DBG_GEEK("(x,y): (%d, %d)\n", x_points[0], y_points[0]);
        // This code loops through all the points touched
        FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0x00, 0x00, 0x00));
        FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(0xB1));                   // Make it solid
        FWo = EVE_Point(FWo, x_points[0] * 16, y_points[0] * 16, 30 * 16); // Draw the touch dot -- a 60px point (filled circle))
        FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(0xFF));
      }
    } // end of outside if

    switch (screen)
    {
    case 0:
      FWo = temperatureGradient(FWo, x_points[0], y_points[0], 6, 1);
      break;
    case 1:
      FWo = voltageGradient(FWo, x_points[0], y_points[0], 6, 1);
      break;
    case 2:
      FWo = temperatureGradient(FWo, x_points[0], y_points[0], 6, 2);
      break;
    case 3:
      FWo = voltageGradient(FWo, x_points[0], y_points[0], 6, 2);
      break;
    case 4:
      FWo = temperatureGradient(FWo, x_points[0], y_points[0], 6, 3);
      break;
    case 5:
      FWo = voltageGradient(FWo, x_points[0], y_points[0], 6, 3);
      break;
    case 6:
      FWo = temperatureGradient(FWo, x_points[0], y_points[0], 6, 4);
      break;
    case 7:
      FWo = voltageGradient(FWo, x_points[0], y_points[0], 6, 4);
      break;
    case 8:
      FWo = temperatureGradient(FWo, x_points[0], y_points[0], 6, 5);
      break;
    case 9:
      FWo = voltageGradient(FWo, x_points[0], y_points[0], 6, 5);
      break;
    default:
      FWo = temperatureGradient(FWo, x_points[0], y_points[0], 6, 1);
      break;
    }

    // FWo = Button(FWo, 0, 0, 100, 50, "", x_points[0], y_points[0], EVE_ENC_COLOR_RGB(0, 0, 155), decrementScreen);

    // FWo = Button(FWo, LCD_WIDTH - 100, LCD_HEIGHT - 100, LCD_WIDTH, LCD_HEIGHT, "", x_points[0], y_points[0], EVE_ENC_COLOR_RGB(0, 100, 155), incrementScreen);

    //========== FINSH AND SHOW THE DISPLAY LIST ==========
    // Instruct the graphics processor to show the list
    FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_DISPLAY());
    // Make this list active
    FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_CMD_SWAP);
    // Update the ring buffer pointer so the graphics processor starts executing
    EVE_REG_Write_16(EVE_REG_CMD_WRITE, (FWo));
  } // while(1)
} // loop()
//===========================================================================

bool isGlobalPressed = 0;
bool released = 0;
/**
 * @brief A button that responds to user feedback -- in working order
 *
 * @param FWo - Pointer for display list
 * @param x0  - initial x point
 * @param y0  - initial y point
 * @param x1  - final x point
 * @param y1  - final y point
 * @param text - Text for button
 * @param x_point - x-coordinate of touch
 * @param y_point - y-coordinate of touch
 * @param color  - color of the button in rgb
 * @return uint16_t
 */
uint16_t Button(uint16_t FWo, int16_t x0, int16_t y0, int16_t x1, int16_t y1, char *text, int16_t x_point, int16_t y_point, uint32_t color, void (*callback)(int8_t), int8_t page)
{
  bool localPress = 0;
  uint8_t r = (color >> 16) & 0xFF; // Extract Red
  uint8_t g = (color >> 8) & 0xFF;  // Extract Green
  uint8_t b = color & 0xFF;

  // Add buffer zone to add tolerance for touches
  int16_t xi = x0 - 20;
  int16_t yi = y0 - 20;
  int16_t xf = x1 + 20;
  int16_t yf = y1 + 20;

  r = (r + 20 > 255) ? 255 : r + 20;
  g = (g + 20 > 255) ? 255 : g + 20;
  b = (b + 20 > 255) ? 255 : b + 20;

  // FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(255, 0, 0));
  // FWo = EVE_Filled_Rectangle(FWo, xi, yi, xf, yf);

  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0, 0, 0));
  FWo = EVE_PrintF(FWo, xi + ((xf - xi) / 2), yf - ((yf - yi) / 2), 30, EVE_OPT_CENTER, "%s", text);

  // DBG_GEEK("x,y: %d %d\n", x_point, y_point);

  if ((x_point >= (xi) && x_point <= xf) && (y_point >= yi && y_point <= yf) && !isGlobalPressed)
  {
    localPress = 1;
    isGlobalPressed = 1;
  }
  else
  {
    isGlobalPressed = false;
  }

  if ((x_point == -32768) && (y_point == -32768))
  {
    released = 1;
  }

  if (localPress && isGlobalPressed && released)
  {
    FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0, 0, 0));

    if (callback)
    {
      callback(page);
    }

    released = 0;
    localPress = 0;
    isGlobalPressed = 0;
  }
  else
  {
    FWo = EVE_Cmd_Dat_0(FWo, color);
  }

  FWo = EVE_Open_Rectangle(FWo, x0, y0, x1, y1, 2);
  return (FWo);
}

// uint16_t temperatureGradient(uint16_t FWo, int16_t x_point, int16_t y_point, int8_t num, int8_t modNum)
// {
//   FWo = header(FWo, x_point, y_point, false, "Mod. Temp");

//   int tempCounter = 1;

//   for (int i = 0; i < 3; i++)
//   {
//     for (int j = 0; j < 7; j++)
//     {
//       FWo = tempGridBox(FWo, j * (LCD_WIDTH / 7),
//                         i * (LCD_HEIGHT / 4) + (LCD_HEIGHT / 4),
//                         j * (LCD_WIDTH / 7) + (LCD_WIDTH / 7),
//                         i * (LCD_HEIGHT / 4) + 2 * (LCD_HEIGHT / 4),
//                         tempCounter, moduleData[modNum - 1][0][tempCounter],
//                         EVE_ENC_COLOR_RGB(255, 0, 0));
//       tempCounter++;
//     }
//   }

//   return (FWo);
// }

uint16_t temperatureGradient(uint16_t FWo, int16_t x_point, int16_t y_point, int8_t num, int8_t modNum)
{
  FWo = header(FWo, x_point, y_point, false, "Mod. Temp");

  int tempCounter = 0;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 7; j++) {
      if (tempCounter >= 18) break; // only show valid thermistors

      float value = Data::getCellTemp(modNum - 1, tempCounter);
      FWo = tempGridBox(FWo, j * (LCD_WIDTH / 7),
                        i * (LCD_HEIGHT / 4) + (LCD_HEIGHT / 4),
                        j * (LCD_WIDTH / 7) + (LCD_WIDTH / 7),
                        i * (LCD_HEIGHT / 4) + 2 * (LCD_HEIGHT / 4),
                        tempCounter + 1,
                        value,
                        EVE_ENC_COLOR_RGB(255, 0, 0));
      tempCounter++;
    }
  }

  return (FWo);
}

uint16_t voltageGradient(uint16_t FWo, int16_t x_point, int16_t y_point, int8_t num, int8_t modNum)
{
  FWo = header(FWo, x_point, y_point, false, "Mod. Volt");

  int voltCounter = 0;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 7; j++) {
      if (voltCounter >= 20) break;

      float value = Data::getCellVoltage(modNum - 1, voltCounter);
      FWo = voltageGridBox(FWo,
                          j * (LCD_WIDTH / 7),
                          i * (LCD_HEIGHT / 4) + (LCD_HEIGHT / 4),
                          j * (LCD_WIDTH / 7) + (LCD_WIDTH / 7),
                          i * (LCD_HEIGHT / 4) + 2 * (LCD_HEIGHT / 4),
                          voltCounter + 1,
                          value,
                          EVE_ENC_COLOR_RGB(0, 255, 0));
      voltCounter++;
    }
  }


  return (FWo);
}

// uint16_t voltageGradient(uint16_t FWo, int16_t x_point, int16_t y_point, int8_t num, int8_t modNum)
// {
//   FWo = header(FWo, x_point, y_point, false, "Mod. Volt");

//   int voltCounter = 1;

//   float voltages[3][7] = {
//       {3.54, 1.56, 4.69, 2.98, 3.92, 2.11, 4.05},
//       {1.91, 5.00, 2.38, 3.27, 4.31, 3.76, 2.84},
//       {1.65, 4.36, 2.51, 3.80, 2.83, 4.12, 1.98}};

//   for (int i = 0; i < 3; i++)
//   {
//     for (int j = 0; j < 7; j++)
//     {
//       FWo = voltageGridBox(FWo, j * (LCD_WIDTH / 7),
//                            i * (LCD_HEIGHT / 4) + (LCD_HEIGHT / 4),
//                            j * (LCD_WIDTH / 7) + (LCD_WIDTH / 7),
//                            i * (LCD_HEIGHT / 4) + 2 * (LCD_HEIGHT / 4),
//                            voltCounter, moduleData[modNum - 1][1][voltCounter],
//                            EVE_ENC_COLOR_RGB(0, 255, 0));
//       voltCounter++;
//     }
//   }

//   return (FWo);
// }

uint16_t tempGridBox(uint16_t FWo, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int8_t num, float value, uint32_t color)
{
  constexpr float TEMP_MIN = 0.0f;
  constexpr float TEMP_MAX = 65.0f;

  // Clamp the temperature first
  value = constrain(value, TEMP_MIN, TEMP_MAX);

  // Map temperature [TEMP_MIN, TEMP_MAX] to [0, 255] opacity
  uint8_t opacity = static_cast<uint8_t>(((value - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)) * 255.0f);
  int intValue = static_cast<int>(value * 10);

  // Fill
  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(opacity)); // Set opacity

  FWo = EVE_Cmd_Dat_0(FWo, color);
  FWo = EVE_Filled_Rectangle(FWo, x0, y0, x1, y1);
  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(0xFF)); // Make it solid

  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0, 0, 0));
  FWo = EVE_PrintF(FWo, x0 + 15, y0 + 15, 18, EVE_OPT_CENTER, "%d", num);

  FWo = EVE_PrintF(FWo, x0 + ((x1 - x0) / 2), y0 + ((y1 - y0) / 2), 25, EVE_OPT_CENTER, "%d.%d", intValue / 10, intValue % 10);

  return (FWo);
}

uint16_t voltageGridBox(uint16_t FWo, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int8_t num, float value, uint32_t color)
{
  constexpr float TEMP_MIN = 0.0f;
  constexpr float TEMP_MAX = 5.0f;

  // Clamp the temperature first
  value = constrain(value, TEMP_MIN, TEMP_MAX);

  // Map temperature [TEMP_MIN, TEMP_MAX] to [0, 255] opacity
  uint8_t opacity = static_cast<uint8_t>(((value - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)) * 255.0f);
  int intValue = static_cast<int>(value * 10);

  // Fill
  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(opacity)); // Set opacity

  FWo = EVE_Cmd_Dat_0(FWo, color);
  FWo = EVE_Filled_Rectangle(FWo, x0, y0, x1, y1);
  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(0xFF)); // Make it solid

  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0, 0, 0));
  FWo = EVE_PrintF(FWo, x0 + 15, y0 + 15, 18, EVE_OPT_CENTER, "%d", num);

  FWo = EVE_PrintF(FWo, x0 + ((x1 - x0) / 2), y0 + ((y1 - y0) / 2), 25, EVE_OPT_CENTER, "%d.%d", intValue / 10, intValue % 10);

  return (FWo);
}

void decrementScreen(int8_t page)
{
  if (screen != 0)
  {
    screen--;
  }
  else
  {
    screen = 9;
  }
}

void incrementScreen(int8_t page)
{
  if (screen != 9)
  {
    screen++;
  }
  else
  {
    screen = 0;
  }
}

void setScreen(int8_t page)
{
  screen = page;
}

uint16_t header(uint16_t FWo, int16_t x_point, int16_t y_point, bool home, char *title)
{
  const char *temp = (screen % 2 == 1) ? "[V]" : "[C]";

  // WFR Logo
  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_RGB(0, 0, 0));
  FWo = Button(FWo, 15, 15, 65, 60, "<", x_point, y_point, EVE_ENC_COLOR_RGB(0, 0, 0), decrementScreen, 1);

  FWo = EVE_PrintF(FWo, 200, 40, 30, EVE_OPT_CENTER, "Module %d %s", (screen / 2) + 1, temp);

  FWo = Button(FWo, (LCD_WIDTH / 2) - 50, 15, (LCD_WIDTH / 2), 60, ">", x_point, y_point, EVE_ENC_COLOR_RGB(0, 0, 0), incrementScreen, 1);

  // Voltage Reading
  FWo = EVE_PrintF(FWo, LCD_WIDTH - 200, 40, 30, EVE_OPT_CENTER, "420V");

  // Current Reading
  float Max_Cell_Temp = Data::getMaxCellTemp();
  FWo = EVE_Text(FWo,LCD_WIDTH - 280, 40, 30,EVE_OPT_CENTER,(char*)String(Max_Cell_Temp).c_str());

  // Settings button
  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(0xD1));
  FWo = Button(FWo, LCD_WIDTH - 140, 15, LCD_WIDTH - 15, 60, "0 Errors", x_point, y_point, EVE_ENC_COLOR_RGB(0, 0, 0), incrementScreen, 10);
  FWo = EVE_Cmd_Dat_0(FWo, EVE_ENC_COLOR_A(0xFF));

  return (FWo);
}