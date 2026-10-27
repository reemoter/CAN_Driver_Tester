#include <stdint.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define INPUT_S1_GPIO_Port 0
#define INPUT_S2_GPIO_Port 0
#define INPUT_S1_Pin 1
#define INPUT_S2_Pin 2
GPIO_PinState HAL_GPIO_ReadPin(int port, int pin);
