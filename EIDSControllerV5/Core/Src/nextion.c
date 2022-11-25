
/** Put this in the src folder **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nextion.h"


#define NEX_RET_CMD_FINISHED            (0x01)
#define NEX_RET_EVENT_LAUNCHED          (0x88)
#define NEX_RET_EVENT_UPGRADED          (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_INVALID_CMD             (0x00)
#define NEX_RET_INVALID_COMPONENT_ID    (0x02)
#define NEX_RET_INVALID_PAGE_ID         (0x03)
#define NEX_RET_INVALID_PICTURE_ID      (0x04)
#define NEX_RET_INVALID_FONT_ID         (0x05)
#define NEX_RET_INVALID_BAUD            (0x11)
#define NEX_RET_INVALID_VARIABLE        (0x1A)
#define NEX_RET_INVALID_OPERATION       (0x1B)


int NextionTimeOutMs = 100;
//int usartReceiveITEnable;
//UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/*
 *
 * TASK: KULLANIM �?EKLİNİ YORUM SATIRINA KOY
 *Nextion_Set_Text("t0", "yazi");
 */
void Nextion_Set_Text(char *object_name, char *buffer)
{
	char buf[30] = { 0 };
	sprintf(buf, "%s.txt=\"%s\"", object_name, buffer);
	Nextion_Send_Command(buf);
}
/*
 * Nextion_Set_Value("n0", 150);
 */
void Nextion_Set_Value(char *object_name, int32_t number)
{
	char buf[30] = { 0 };
	sprintf(buf, "%s.val=%ld", object_name, number);
	Nextion_Send_Command(buf);
	HAL_Delay(15);
}

/*
 * Nextion_Get_Value("n0");
 */
void Nextion_Get_Value(char *object_name)
{
	char buf[30] = { 0 };
	int countDelay = 0;
	sprintf(buf, "get %s.val", object_name);
	Nextion_Send_Command(buf);

//	UART1_Write(0x67); // g
//	UART1_Write(0x65); // e
//	UART1_Write(0x74); // t
//	UART1_Write(0x20); // boşluk
//	UART1_Write_Text(gelen_nesne);
//	UART1_Write(0x2E); //.
//	UART1_Write(0x76); // v
//	UART1_Write(0x61); // a
//	UART1_Write(0x6C); // l
//	UART1_Write(0xFF);
//	UART1_Write(0xFF);
//	UART1_Write(0xFF);
	// i=0;

	//Delay_ms(30);
	while (countDelay< 20)
	{
			HAL_Delay(10);
			countDelay++;
	}
}

/*
 *
 * Nextion_Send_Command("page 3");
 */
void Nextion_Send_Command(char *command)
{
	char buf[30] = { 0 };
	char sps = 0xFF;
	int SizeBuf = sprintf(buf, "%s%c%c%c", command, sps, sps, sps);
	//HAL_UART_Transmit(&huart3, (uint8_t*) &buf, SizeBuf, NextionTimeOutMs);
	HAL_UART_Transmit(&huart6, (uint8_t*) &buf, SizeBuf, NextionTimeOutMs);
	HAL_Delay(10);																
}

/*
 *
 * Nextion_Send_Command("page 3");
 */
void Nextion_Send_Parameters(void)
{
}

//add id,kanal,data fff
void Nextion_Waveform(void)
{
	int number;
	char buf[30] = { 0 };
	while(1){
		number++;
		sprintf(buf, "add 2,0,%ld",  number);
		Nextion_Send_Command(buf);
		HAL_Delay(10);
		if(number>115){
			number = 0;
		}
	}
}

//add id,kanal,data fff
void Nextion_SetGraphic(uint8_t id, uint8_t chanel, uint8_t data )
{
	char buf[30] = { 0 };
	sprintf(buf, "add %d,%d,%d", id, chanel, data);
	Nextion_Send_Command(buf);
}


void Nextion_Page(uint8_t page_name)
{
	char buf[30] = { 0 };
	sprintf(buf, "page %d", page_name);
	Nextion_Send_Command(buf);
	HAL_Delay(100);
}

/*
 * Receive uint32_t data.
 *
 * @param number - save uint32_t data.
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed.
 *
 */
bool recvRetNumber(uint32_t *number, uint32_t timeout)
{
	uint8_t temp[8] = {0};

	if (!number){
		return false;
	}

	while(TM_USART_BufferCount(nexUsart) < 7){

	}
	HAL_Delay(10);

	int pos = TM_USART_FindCharacter(nexUsart, NEX_RET_NUMBER_HEAD);
	if(pos >= 0){

		for(int b = 0; b<pos; b++){
			TM_USART_Getc(nexUsart);
		}

		for(int i = 0; i<8; i++){
			temp[i] = TM_USART_Getc(nexUsart);
		}

		if (temp[0] == NEX_RET_NUMBER_HEAD
				&& temp[5] == 0xFF
				&& temp[6] == 0xFF
				&& temp[7] == 0xFF
		)
		{
			*number = (temp[4] << 24) | (temp[3] << 16) | (temp[2] << 8) | (temp[1]);
			return true;
		}
	}
	return false;
}


/*
 * Receive string data.
 *
 * @param buffer - save string data.
 * @param len - string buffer length.
 * @param timeout - set timeout time.
 *
 * @return the length of string buffer.
 *
 */
uint16_t recvRetString(char *buffer, uint16_t len, uint32_t timeout)
{
	uint16_t ret = 0;

	char strflag[5] = {NEX_RET_STRING_HEAD};
	char endflag[5] = {0xFF, 0xFF, 0xFF};
	char tempBuff[100] = {0};


	if (!buffer || len == 0)
	{
		return 0;
	}


	while (TM_USART_FindString(nexUsart, endflag) <= 0) {}

	if(TM_USART_Gets_withStrandEndFlag(nexUsart, tempBuff, sizeof(tempBuff), strflag, endflag) > 0){
		ret = strlen(tempBuff);
		ret = ret > len ? len : ret;
		strncpy(buffer, tempBuff, ret);
		return ret;
	}
	return 0;
}


/*
 * Send command to Nextion.
 *
 * @param cmd - the string of command.
 */
void sendCommand(const char *cmd)
{
	while (!TM_USART_BufferEmpty(nexUsart))
	{
		TM_USART_Getc(nexUsart);
	}

	TM_USART_Puts(nexUsart, (char*)cmd);
	TM_USART_Putc(nexUsart, 0xFF);
	TM_USART_Putc(nexUsart, 0xFF);
	TM_USART_Putc(nexUsart, 0xFF);
}



/*
 * Command is executed successfully.
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed.
 *
 */
bool recvRetCommandFinished(void){
	char temp[8] = {};
	char endFlag[4] = {NEX_RET_CMD_FINISHED, 0xFF, 0xFF, 0xFF};

	//setTimeout(timeout);

	while (TM_USART_BufferCount(nexUsart) < strlen(endFlag)) {
	}

	if(TM_USART_Gets_withEndFlag(nexUsart, temp, sizeof(temp), endFlag) > 0){
		return true;
	}
	return false;
}


/*
 * Init Nextion.
 *
 * @return true if success, false for failure.
 */
bool nexInit(uint32_t nexBaudrate){

	TM_USART_Init(nexUsart, nexUsartPinsPack, nexBaudrate);

	bool ret1 = false;
	bool ret2 = false;

	sendCommand("");
	sendCommand("bkcmd=1");
	ret1 = recvRetCommandFinished();
	sendCommand("page 0");
	ret2 = recvRetCommandFinished();
	return ret1 && ret2;

}

/*
void nexLoop(NexTouch *nex_listen_list[])
{
    static uint8_t __buffer[10];

    uint16_t i;
    uint8_t c;

    while (TM_USART_BufferCount(nexUsart) > 0)
    {

        HAL_Delay(10);
        c = TM_USART_Getc(nexUsart);

        if (NEX_RET_EVENT_TOUCH_HEAD == c)
        {
            if (TM_USART_BufferCount(nexUsart) >= 6)
            {
                __buffer[0] = c;
                for (i = 1; i < 7; i++)
                {
                    __buffer[i] = TM_USART_Getc(nexUsart);
                }
                __buffer[i] = 0x00;

                if (0xFF == __buffer[4] && 0xFF == __buffer[5] && 0xFF == __buffer[6])
                {
                    NexTouch::iterate(nex_listen_list, __buffer[1], __buffer[2], (int32_t)__buffer[3]);
                }

            }
        }
    }
}
 */



uint16_t nex_getText(NexObject obj, char *buffer, uint16_t len){

	char cmd[50] = {};
	strcat(cmd, "get ");
	strcat(cmd, obj.name);
	strcat(cmd, ".txt");
	sendCommand(cmd);
	return recvRetString(buffer,len,100);
}

bool nex_setText(NexObject obj, char text[nex_maxTextChar]){

	char cmd[50] = {};
	strcat(cmd, obj.name);
	strcat(cmd, ".txt=\"");
	strcat(cmd, text);
	strcat(cmd, "\"");
	sendCommand(cmd);
	return recvRetCommandFinished();
}

bool nex_getPic(NexObject obj, uint32_t *number){

	char cmd[50] = {};
	strcat(cmd, "get ");
	strcat(cmd, obj.name);
	strcat(cmd, ".pic");
	sendCommand(cmd);
	return recvRetNumber(number,100);
}

bool nex_setPic(NexObject obj, uint32_t number){

	char buf[10] = {};
	char cmd[50] = {};

	utoa(number, buf, 10);
	strcat(cmd, obj.name);
	strcat(cmd, ".pic=");
	strcat(cmd, buf);

	sendCommand(cmd);
	return recvRetCommandFinished();
}

bool nex_getValue(NexObject obj, uint32_t *number){

	char cmd[50] = {};

	strcat(cmd, "get ");
	strcat(cmd, obj.name);
	strcat(cmd, ".val");
	sendCommand(cmd);
	return recvRetNumber(number,100);

}

bool nex_setValue(NexObject obj, uint32_t number){

	char buf[10] = {0};
	char cmd[50] = {};

	utoa(number, buf, 10);
	strcat(cmd, obj.name);
	strcat(cmd, ".val=");
	strcat(cmd, buf);

	sendCommand(cmd);
	return recvRetCommandFinished();

}

bool nex_goToPage(NexObject obj){

	const char *name = obj.name;
	if (!name)
	{
		return false;
	}

	char cmd[50] = {};
	strcat(cmd, "page ");
	strcat(cmd, name);
	sendCommand(cmd);
	return recvRetCommandFinished();

}

void nex_click(NexObject obj){
	char cmd[50] = {};
	strcat(cmd, "click ");
	strcat(cmd, obj.name);
	strcat(cmd, ",1");
	sendCommand(cmd);

	char cmd2[50] = {};
	strcat(cmd2, "click ");
	strcat(cmd2, obj.name);
	strcat(cmd2, ",0");
	sendCommand(cmd2);
}

void nex_hide(NexObject obj){
	char cmd[50] = {};
	strcat(cmd, "vis ");
	strcat(cmd, obj.name);
	strcat(cmd, ",0");
	sendCommand(cmd);
}

void nex_show(NexObject obj){
	char cmd[50] = {};
	strcat(cmd, "vis ");
	strcat(cmd, obj.name);
	strcat(cmd, ",1");
	sendCommand(cmd);
}

void nex_reset(void){
	sendCommand("rest");
}

/*bool nex_addValue(NexObject obj, uint8_t ch, uint8_t number){

	char buf[15] = {0};

	if (ch > 3)
	{
		return false;
	}

	sprintf(buf, "add %u,%u,%u", getObjCid(), ch, number);

	sendCommand(buf);
	return true;

}*/
