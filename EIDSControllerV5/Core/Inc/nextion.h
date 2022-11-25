#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"


/*
 * types of nexObjects is types of nextion objects in editor
 */
enum nexObjects{ button, crop, gauge, hotspot, page, picture, progressbar, slider, text, waveform };

/*
 * @param nObj - type of object.
 * @param pid - page id.
 * @param cid - component id.
 */
typedef struct {
	enum nexObjects nObj;
	char *name;
	uint8_t pid;
	uint8_t cid;
}NexObject;

//#define nexUsart USART1
#define nexUsart USART6
//#define nexUsart USART3
//#define nexUsart UART4
//#define nexUsart UART5
//#define nexUsart USART6
//#define nexUsart UART7
//#define nexUsart UART8

#define nexUsartPinsPack 0 // Pins Pack 1
//#define nexUsartPinsPack 1 // Pins Pack 2
//#define nexUsartPinsPack 2 // Pins Pack 3

#define nex_maxTextChar 10 // Maximum length of texts of objects, 10 is default on nextion editor.


bool recvRetNumber(uint32_t *number, uint32_t timeout);
uint16_t recvRetString(char *buffer, uint16_t len, uint32_t timeout);
void sendCommand(const char* cmd);
bool recvRetCommandFinished(void);
bool nexInit(uint32_t nexBaudrate);

void nex_addTouchListener(NexObject obj);

uint16_t nex_getText(NexObject obj, char *buffer, uint16_t len);

bool nex_setText(NexObject obj, char text[nex_maxTextChar]);

bool nex_getPic(NexObject obj, uint32_t *number);

bool nex_setPic(NexObject obj, uint32_t number);

bool nex_getValue(NexObject obj, uint32_t *number);

bool nex_setValue(NexObject obj, uint32_t number);

bool nex_goToPage(NexObject obj);

void nex_click(NexObject obj);

void nex_hide(NexObject obj);

void nex_show(NexObject obj);

void nex_reset(void);

//bool nex_addValue(NexObject obj, uint8_t ch, uint8_t number);


void Nextion_Set_Text(char *object_name, char *buffer);
void Nextion_Set_Value(char *object_name, int32_t number);
void Nextion_Send_Command(char *command);
void Nextion_Send_Parameters(void);
void Nextion_Waveform(void);
void Nextion_SetGraphic(uint8_t id, uint8_t chanel, uint8_t data );
void Nextion_Page(uint8_t page_name);
void  Nextion_Send_Parameters(void );
void Nextion_Get_Value(char *object_name);


