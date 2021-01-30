/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define USART_TXBUF_LEN 1512
#define USART_RXBUF_LEN 256

uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];
__IO int USART_TX_Empty = 0; //__IO = volatile = jest to informacja dla kompilatora że nie ma prawa zaoptymalizować obsługi,
__IO int USART_TX_Busy = 0;  //nie można tej zmiennej wczytywać do rejestru,
__IO int USART_RX_Empty = 0; //należy ją za każdym razem pobierać z pamięci, jest ulotna, jej wartość może się zmienić w każdej chwili
__IO int USART_RX_Busy = 0;

int interval = 2000;
uint16_t timer = 0;
uint8_t cycle = 0;

uint8_t celsiusSet = 1;

float farenheitTemp;
float measurements[1024];
uint16_t measurementsIndex = 0;

unsigned char commandBuffer[128];
unsigned char tempCommandBuffer[128];
uint8_t commandBufferSize = 0;
uint8_t frameStartDetected = 0;
uint8_t frameEndDetected = 0;
uint8_t escape = 0;
uint8_t checksum;

struct lcd_disp disp;

uint8_t showEnd = 0;
uint8_t showData = 0;
uint16_t showDataIndex = 0;
uint8_t showDataError = 0;
uint8_t showUnit = 0;
uint8_t showDelete = 0;
uint8_t showInterval = 0;
uint8_t showIntervalError = 0;
uint16_t showIntervalValue = 0;
uint8_t showWrongCommand = 0;
uint8_t showChecksums = 0;
uint8_t showWrongFrameSize = 0;
uint8_t showingMessage = 0;

uint16_t showTimer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t USART_isNotEmpty(){ //jeśli coś jest w buforze kołowym to zwraca 1, jeśli jest pusty to zwraca 0
	if(USART_RX_Empty == USART_RX_Busy) {
		return 0;
	} else {
		return 1;
	}
} //USART_kbhit

int USART_getchar() {

	uint8_t tmp;

	if(USART_RX_Empty != USART_RX_Busy) {
		tmp = USART_RxBuf[USART_RX_Busy];
		USART_RX_Busy++;

		if(USART_RX_Busy >= USART_RXBUF_LEN) USART_RX_Busy = 0;

		return tmp;
	} else return -1;
}

uint8_t USART_getline(char *buf) {

	static uint8_t bf[128];
	static uint8_t idx = 0;
	int i;
	uint8_t ret;

	while(USART_isNotEmpty()) {
		bf[idx] = USART_getchar();

		if(bf[idx] == 10 || bf[idx] == 13) { 	//sprawdzenie czy znak jest znakiem końca linii lub powrotem karetki
			bf[idx] = 0;						//jeśli jest to podmieniany jest na 0, ponieważ w c string kończy się zerem

			for(i=0; i<=idx; i++){
				buf[i] = bf[i];
			}

			ret = idx;
			idx = 0;
			return ret; //zwracam odebraną ilość znaków
		} else {
			idx++;
			if(idx>=128) idx = 0;
		}
	}
	return 0;
} //USART_getline

void USART_fsend(char * format,...) {

	char tmp_rs[256];
	int i;
	__IO int idx;
	va_list arglist;
	va_start(arglist, format);
	vsprintf(tmp_rs, format, arglist);
	va_end(arglist);

	idx = USART_TX_Empty;

	for(i=0; i<strlen(tmp_rs); i++) {
		USART_TxBuf[idx] = tmp_rs[i];
		idx++;
		if (idx >= USART_TXBUF_LEN) idx = 0;
	}

	__disable_irq();

	if((USART_TX_Empty == USART_TX_Busy) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) { //sprawdzanie dodatowo zajętości bufora nadajnika
		USART_TX_Empty = idx;																		//TXE = SET mówi o tym że bufor nadajnika jest pusty
		uint8_t tmp = USART_TxBuf[USART_TX_Busy];
		USART_TX_Busy++;

		if(USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);

	} else {
		USART_TX_Empty = idx;
	}
	__enable_irq();
} //USART_fsend

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2) {
		if(USART_TX_Empty != USART_TX_Busy) { //sprawdzenie czy jest coś do wysłania
			uint8_t tmp = USART_TxBuf[USART_TX_Busy];
			USART_TX_Busy++;
			if(USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy = 0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart2) {
		USART_RX_Empty++;
		if(USART_RX_Empty >= USART_RXBUF_LEN) USART_RX_Empty = 0;
		HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1);
	}
}

unsigned char crc8(unsigned char poly, unsigned char* data, uint8_t size) {
	/*
	 * Jest to funkcja odpowiedzialna za obliczanie
	 * sumy kontrolnej z przesłanych danych.
	 * */
    unsigned char crc = 0x00;
    int bit;

    while(size--) {
        crc ^= *data++;
        for(bit=0; bit<8; bit++) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;

}

void sendPayload(char* format, ...) {
	/*
	 * Jest to funkcja odpowiedzialna za wysyłanie
	 * komunikatów w sposób odpowiedni dla zaprojektowanego
	 * protokołu komunikacyjnego.
	 * */
	unsigned char payload[256];
	va_list payload_list;
	va_start(payload_list, format);
	vsprintf((char *)payload, format, payload_list);
	va_end(payload_list);

	USART_fsend("!%s%c;\r\n", payload, crc8(0xA6, payload, strlen((const char *)payload)));
}

void commandCheck(unsigned char* data, uint8_t size, unsigned char checksum) {
	/*
	 * Jest to funkcja odpowiedzialna za zdekodowanie
	 * otrzymanej komendy. Jako argumenty przyjmuje
	 * ona przesłaną komendę, jej rozmiar oraz przesłaną
	 * sumę kontrolną w celach jej weryfikacji.
	 * */

	unsigned char checksumCheck = crc8(0xA6, data, size);

	/*
	 * Pierwszym krokiem jest obliczenie sumy kontrolnej
	 * z otrzymanej komendy i prównanie jej z tą przesłaną.
	 * Jeśli się zgadzają to następnie komenda jest analizowana.
	 * */
	if(checksum == checksumCheck) {
		sendPayload("Checksums are correct");

		int interval_value;
		uint16_t read_value;


		if(memcmp(data,"setunit:c",9) == 0) { //memcompare
			sendPayload("Setting unit to Celsius.");
			celsiusSet = 1;
			showUnit = 1;
		} else if(memcmp(data,"setunit:f",9) == 0) {
			sendPayload("Setting unit to Farenheit.");
			celsiusSet = 0;
			showUnit = 1;
		} else if(sscanf((const char *)data,"setint:%i",&interval_value)) { //setting interval
			if(interval_value > 0) {
				sendPayload("Interval set to %i", interval_value);
				interval = interval_value;
				showIntervalValue = interval_value;
				showInterval = 1;
			} else {
				sendPayload("Interval cannot be less than 1");
				showIntervalError = 1;
			}

		} else if(sscanf((const char *)data,"read:%" SCNu16, &read_value)) { //reading saved measurement
			if(read_value >= measurementsIndex) {
				sendPayload("Value #%" SCNu16 " wasn't written yet", read_value);
				showDataError = 1;
				showDataIndex = read_value;
			} else {
				if(celsiusSet) {
					sendPayload("Value #%" SCNu16 ": %.2f C", read_value, measurements[read_value]);
					showData = 1;
					showDataIndex = read_value;
				} else {
					farenheitTemp = measurements[read_value] * 9/5 + 32;
					sendPayload("Value #%" SCNu16 ": %.2f F", read_value, farenheitTemp);
					showData = 1;
					showDataIndex = read_value;
				}

			}
		} else if(memcmp(data,"deldata",7) == 0) { //deleting saved measurements
			sendPayload("Deleting saved measurements");
			memset(measurements, 0, sizeof measurements);
			measurementsIndex = 0;
			showDelete = 1;
		} else {
			sendPayload("Command not recognized");
			showWrongCommand = 1;
		}

	}
	/*
	 * Jeśli sumy kontrolne się nie zgadzają
	 * to przesyłany oraz wyświetlany jest
	 * odpowiedni komunikat.
	 * */
	else {
		sendPayload("Checksums don't match!");
		showChecksums = 1;
	}

}

float readTemp() {
	/*
	 * Jest to funkcja odpowiedzialna za odczytywanie
	 * termperatury z czujnika MCP9808. Zwraca ona
	 * obliczoną temperaturę przedstawioną w stopniach
	 * Celsjusza.
	 * */
	float temperature;
	uint8_t temp[2];
	float f_temp[2];

	if(HAL_I2C_Mem_Read(&hi2c2, (0x18 << 1) | 0x01, 0x05, 1, temp, 2, 10) == HAL_OK) {
		temp[0] = temp[0] & 0x1F;
		if((temp[0] & 0x10) == 0x10) {
		  temp[0] = temp[0] & 0x0F;
		  f_temp[0] = temp[0];
		  f_temp[1] = temp[1];
		  temperature = 256 - (f_temp[0] * 16 + f_temp[1] / 16);
		} else {
		  f_temp[0] = temp[0];
		  f_temp[1] = temp[1];
		  temperature = (f_temp[0] * 16 + f_temp[1] / 16);
		}
	} else return -1000;

	return temperature;
}

void showTemp(uint8_t i) {
	/*
	 * Jest to funkcja odpowiedzialna za wyświetlanie
	 * temperatury na wyświetlaczu. Przyjmuje jako argument
	 * i, które oznacza który odczyt chcemy wyświetlić,
	 * np. i=0 oznacza że chcemy wyświetlić najnowszy
	 * odczyt, a i=1 oznacza że chcemy odczytać przedostatni
	 * odczyt.
	 **/
	  if(celsiusSet) {
		  sprintf((char *)disp.f_line, "Temperature:");
		  sprintf((char *)disp.s_line, "Celsius    %.2f", measurements[measurementsIndex - i]);
		  if(lcd_display(&disp) == 0) {
			  sendPayload("Display error");
		  }
	  } else {
		  farenheitTemp = measurements[measurementsIndex - i] * 9/5 + 32;
			  sprintf((char *)disp.f_line, "Temperature:");
			  sprintf((char *)disp.s_line, "Farenheit  %.2f", farenheitTemp);
			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
	  }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[0], 1);
  uint8_t read_char;

  disp.addr = (0x27 << 1);
  disp.bl = true;
  if(lcd_init(&disp) == 0) {
	  sendPayload("Display error");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*
	   * Ta sekcja kodu odpowiedzialna jest za kolekcjonowanie poprawnej ramki
	   * */

	  if(USART_isNotEmpty()) {
		  read_char = USART_getchar();
		  /*
		   *  Jeśli przesłany znak jest znakiem początku ramki
		   *  to tablica w której zbierana jest ramka jest czyszczona
		   *  (wszystkie jej elementy są zerowane) oraz indeks
		   *  zapisywanych wartości jest również zerowany. Dodatkowo
		   *  ustawiana na 1 jest również flaga wykrycia początku ramki,
		   *  która jest wykorzystywana dalej w kodzie. Zerowany jest
		   *  również indeks rozmiaru ramki.
		   *  */
		  if(read_char == 0x21) {
			  frameStartDetected = 1;
			  memset(commandBuffer, 0, sizeof commandBuffer);
			  commandBufferSize = 0;
		  }
		  /*
		   * Jeśli przesłany znak jest znakiem końca ramki i flaga
		   * wykrycia początku ramki jest ustawiona wtedy otrzymana
		   * ramka zostaje poddana analizie
		   * */
		  else if(read_char == 0x3B && frameStartDetected) {
			  if(commandBufferSize > 2) {

				  /*
				   * Jeśli rozmiar ramki jest poprawny to wyciągana jest
				   * suma kontrolna z danych (ostatni element w tablicy)
				   * i czysta komenda np. "setint:1000" przesyłana jest
				   * do analizy do funkcji commandCheck. Po analizie
				   * flaga wykrycia początku i końca ramki jest resetowana
				   * co pozwala na odebranie kolejnego komunikatu
				   * */


				  checksum = commandBuffer[commandBufferSize - 1];
				  memset(tempCommandBuffer, 0, sizeof tempCommandBuffer);
				  memcpy(tempCommandBuffer, commandBuffer, commandBufferSize - 1);

				  commandCheck(tempCommandBuffer, commandBufferSize-1, checksum);

				  frameStartDetected = 0;
				  frameEndDetected = 0;

				  /*
				   * jeśli ramka jest zbyt krótka to flaga wykrycia
				   * początku oraz końca ramki jest resetowana
				   * oraz wysyłany jest komunikat o błędzie
				   * */

			  } else {
				  frameStartDetected = 0;
				  frameEndDetected = 0;
				  showWrongFrameSize = 1;
				  sendPayload("Frame too short");
			  }
		  }
		  /*
		   * Sprawdzenie stanu flagi startu oraz końca ramki oraz przesyłanego znaku.
		   * W momencie kiedy nie jest przesyłany znak początku ramki
		   * (nie chcemy dodawać go do ładunku) dodawana jest treść ramki do ładunku.
		   * W przypadku przesyłania znaków kodowanych:
		   * - "\%" - znak początku ramki
		   * - "\^" - znak końca ramki
		   * Są one rozkodowywane oraz do ładunku dodawany jest rozkodowany znak
		   * */
		  else if(frameStartDetected && !frameEndDetected && read_char != 0x21) {
				 if(read_char == 0x5C && escape == 0) {
					 escape = 1;
				 } else if(escape == 0) {
					commandBuffer[commandBufferSize++] = read_char;
				 } else {
					if(read_char == 0x25) {
						commandBuffer[commandBufferSize++] = 0x21;
					} else if(read_char == 0x5E) {
						commandBuffer[commandBufferSize++] = 0x3B;
					} else if(read_char == 0x5C) {
						commandBuffer[commandBufferSize++] = 0x5C;
					} else {
						frameStartDetected = 0;
					}
					escape = 0;
				 }
				/*
				 * Po każdym dodaniu znaku do ładunku zostaje
				 * sprawdzany indeks rozmiaru ramki, który
				 * nie może przekraczać 126 elementów
				 * 1-125 dane + 1 CRC
				 * */
				if(commandBufferSize > 126) {
					frameStartDetected = 0;
					showWrongFrameSize = 1;
					sendPayload("Frame too long");
				}
		  }
	  }

	  /*
	   * Ta sekcja kodu odpowiedzialna jest za wyświetlanie
	   * temperatury w odpowieniej jednostce w zadanym
	   * interwale czasowym
	   * */

	  if(cycle) {
		  measurements[measurementsIndex] = readTemp();
		  cycle = 0;

		  /*
		   * Sprawdzane jest czy udało się poprawnie odczytać wartość
		   * z termometra - wartość "-1000" oznacza błąd odczytu.
		   * Jeśli odczyt jest prawidłowy to wyświetlana jest
		   * temperatura w odpowiedniej jednostce.
		   * */
		  if(measurements[measurementsIndex] != -1000) {
			  if(showingMessage == 0) {
				  showTemp(0);
			  }
			  celsiusSet ? sendPayload("Temperature: %.2f C", measurements[measurementsIndex]) : sendPayload("Temperature: %.2f F", farenheitTemp);
			  measurementsIndex++;
			  if(measurementsIndex > 1023) {
				  measurementsIndex = 0;
			  }
		  }
		  /*
		   * W przypadku błędu odczytu, odpowiedni komunikat jest przesyłany
		   * oraz wyświetlany na wyświetlaczu
		   * */
		  else {
			  sendPayload("MCPC9808 sensor error");
			  sprintf((char *)disp.f_line, "  !!! ERROR !!!");
			  sprintf((char *)disp.s_line, "MCPC9808 NOT CON");
			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }
	  }

	  /*
	   * Ta sekcja odpowiedzialna jest za wyświetlanie odpowiednich komunikatów
	   * */

	  if(showChecksums) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  sprintf((char *)disp.f_line, "   Checksums");
			  sprintf((char *)disp.s_line, "   don't match");

			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }
		  if(showEnd) {
			  showChecksums = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  } else if(showData) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  if(celsiusSet) {
				  sprintf((char *)disp.f_line, "Measurement#%d", showDataIndex);
				  sprintf((char *)disp.s_line, "     %.2fC", measurements[showDataIndex]);
				  if(lcd_display(&disp) == 0) {
					  sendPayload("Display error");
				  }
			  } else {
				  farenheitTemp = measurements[showDataIndex] * 9/5 + 32;
				  sprintf((char *)disp.f_line, "Measurement#%d", showDataIndex);
				  sprintf((char *)disp.s_line, "     %.2fF", farenheitTemp);
				  if(lcd_display(&disp) == 0) {
					  sendPayload("Display error");
				  }
			  }
		  }
		  if(showEnd) {
			  showData = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  } else if(showDataError) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  sprintf((char *)disp.f_line, "Value #%d was", showDataIndex);
			  sprintf((char *)disp.s_line, "not yet written");

			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }
		  if(showEnd) {
			  showDataError = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  } else if(showDelete) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  sprintf((char *)disp.f_line, "All measurements");
			  sprintf((char *)disp.s_line, "has been deleted");

			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }

		  if(showEnd) {
			  showDelete = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  } else if(showWrongCommand) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  sprintf((char *)disp.f_line, "  Command not");
			  sprintf((char *)disp.s_line, "  recognized");

			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }
		  if(showEnd) {
			  showWrongCommand = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  } else if(showUnit) {
		  if(!cycle && !showingMessage) {
			  showingMessage = 1;
			  showTemp(1);
		  }

		  if(cycle) {
			  showUnit = 0;
			  showingMessage = 0;
		  }
	  } else if(showInterval) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  sprintf((char *)disp.f_line, "Interval was");
			  sprintf((char *)disp.s_line, "set to %dms", showIntervalValue);
			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }
		  if(showEnd) {
			  showInterval = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  } else if(showIntervalError) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  sprintf((char *)disp.f_line, " Interval can't");
			  sprintf((char *)disp.s_line, " be less than 1");

			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }
		  if(showEnd) {
			  showIntervalError = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  } else if(showWrongFrameSize) {
		  if(showingMessage == 0) {
			  showingMessage = 1;
			  showTimer = 0;
			  showEnd = 0;
			  sprintf((char *)disp.f_line, "   Wrong frame");
			  sprintf((char *)disp.s_line, "      size");

			  if(lcd_display(&disp) == 0) {
				  sendPayload("Display error");
			  }
		  }
		  if(showEnd) {
			  showWrongFrameSize = 0;
			  showingMessage = 0;
			  showTemp(1);
		  }
	  }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
