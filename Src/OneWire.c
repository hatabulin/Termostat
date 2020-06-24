/*
 *
 * this source code implement from network by me, and adapted to HAL stm32 driver.
 *
 */

#include "OneWire.h"
#include "stm32f1xx_hal.h"
volatile uint8_t recvFlag;
uint8_t rc_buffer[5];
uint8_t debag;
extern UART_HandleTypeDef huart2;
void _usart_enable_halfduplex() {
	OW_USART->CR2 &= ~USART_CR2_LINEN;
	OW_USART->CR2 &= ~USART_CR2_CLKEN;
	OW_USART->CR3 &= ~USART_CR3_SCEN;
	OW_USART->CR3 &= ~USART_CR3_IREN;
  OW_USART->CR3 |= USART_CR3_HDSEL;
}

uint8_t getUsartIndex(void);

void usart_setup_(uint32_t baud)
{
	MX_USART2_UART_Init(baud); //!!!!
	_usart_enable_halfduplex();
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void owInit(OneWire *ow) {
	int i=0, k = 0;
	for (; i < MAXDEVICES_ON_THE_BUS; i++) {
		uint8_t *r = (uint8_t *)&ow->ids[i];
		k=0;
		for (; k < 8; k++) r[k] = 0;
	}
	k=0;
	for (; k < 8; k++) ow->lastROM[k] = 0x00;
	ow->lastDiscrepancy = 64;
}

void owReadHandler() {
	uint8_t index = getUsartIndex();
	if (((OW_USART->CR1 & USART_CR1_RXNEIE) != 0) && ((OW_USART->SR & UART_FLAG_RXNE) != (uint16_t)RESET)) {
    while ((OW_USART->SR & UART_FLAG_RXNE) == (uint16_t)RESET) {}
    rc_buffer[index] = (uint16_t)(OW_USART->DR & (uint16_t)0x01FF);	
    recvFlag &= ~(1 << index);
	}
}

uint16_t owResetCmd() {
	uint16_t owPresence;
	
	usart_setup_(9600);

	owSend(0xF0);
	owPresence = owEchoRead();

	usart_setup_(115200);
	debag = owPresence;
	return owPresence;
}

uint8_t getUsartIndex() {
	uint8_t result;
	if(OW_USART==USART1)result = 0;
	else if (OW_USART==USART2)result = 1;
	else if (OW_USART==USART3)result = 2;
	return result;
}

void owSend(uint16_t data) {
	recvFlag |= (1 << getUsartIndex());
	OW_USART->DR = (data & (uint16_t)0x01FF);
	while ((OW_USART->SR & UART_FLAG_TC) == (uint16_t)RESET) {}
}

uint8_t owReadSlot(uint16_t data) {
	return (data == OW_READ) ? 1 : 0;
}

uint8_t owEchoRead() {
	uint8_t i = getUsartIndex();
	uint16_t pause = 1000;

	while (recvFlag & (1 << i) && pause--);
	return rc_buffer[i];
}

uint8_t *byteToBits(uint8_t ow_byte, uint8_t *bits) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*bits = WIRE_1;
		} else  *bits = WIRE_0;
    bits++;
    ow_byte = ow_byte >> 1;
  }
  return bits;
}

void owSendByte(uint8_t d) {
	uint8_t data[8];
	int i;
	byteToBits(d, data);
	for (i = 0; i < 8; ++i) owSend(data[i]);
}

uint8_t bitsToByte(uint8_t *bits) {
	uint8_t target_byte, i;
	target_byte = 0;

	for (i = 0; i < 8; i++) {
		target_byte = target_byte >> 1;
		if (*bits == WIRE_1) {
			target_byte |= 0x80;
		}
		bits++;
	}
	return target_byte;
}

uint8_t owCRC(uint8_t *mas, uint8_t Len) {
	uint8_t i, dat, crc, fb, st_byt;

	st_byt = 0;
	crc = 0;
	do {
	    dat = mas[st_byt];
	    for (i = 0; i < 8; i++) {
	    	fb = crc ^ dat;
	    	fb &= 1;
	    	crc >>= 1;
	    	dat >>= 1;
	    	if (fb == 1) crc ^= 0x8c;
	    }
	    st_byt++;
	} while (st_byt < Len);

	return crc;
}

uint8_t owCRC8(RomCode *rom) {

	return owCRC((uint8_t*)rom, 7);
}

/*
 * return 1 if has got one more address
 * return 0 if hasn't
 * return -1 if error reading happened
 *
 */
int hasNextRom(OneWire *ow, uint8_t *ROM) {
	uint8_t ui32BitNumber = 0;
	int zeroFork = -1;
	uint8_t i = 0;
	if (owResetCmd() == ONEWIRE_NOBODY) return 0;

	owSendByte(ONEWIRE_SEARCH);//
	do {
		uint8_t answerBit =0;
		int byteNum = ui32BitNumber / 8;
		uint8_t *current = (ROM) + byteNum;
		uint8_t cB, cmp_cB, searchDirection = 0;
		owSend(OW_READ);
		cB = owReadSlot(owEchoRead());
		owSend(OW_READ);
		cmp_cB = owReadSlot(owEchoRead());

		if (cB == cmp_cB && cB == 1) return -1;
		if (cB != cmp_cB) {
			searchDirection = cB;
		} else {
			if (ui32BitNumber == ow->lastDiscrepancy) searchDirection = 1;
			else {
				if (ui32BitNumber > ow->lastDiscrepancy) searchDirection = 0;
				else searchDirection = (uint8_t) ((ow->lastROM[byteNum] >> ui32BitNumber % 8) & 0x01);

			    if (searchDirection == 0) zeroFork = ui32BitNumber;
			}
		}

		if (searchDirection) *(current) |= 1 << ui32BitNumber % 8;
		answerBit = (uint8_t) ((searchDirection == 0) ? WIRE_0 : WIRE_1);
		owSend(answerBit);
		ui32BitNumber++;
	} while (ui32BitNumber < 64);

	ow->lastDiscrepancy = zeroFork;
	for (; i < 7; i++) ow->lastROM[i] = ROM[i];
	return ow->lastDiscrepancy > 0;
}

int owSearchCmd(OneWire *ow) {
	int device = 0, nextROM;
	owInit(ow);
	do {
		nextROM = hasNextRom(ow, (uint8_t*)(&ow->ids[device]));
		if (nextROM<0) return -1;
		device++;
	} while (nextROM && device < MAXDEVICES_ON_THE_BUS);
	return device;
}

void owSkipRomCmd(OneWire *ow) {
	owResetCmd();
	owSendByte(ONEWIRE_SKIP_ROM);
}

void owMatchRomCmd(RomCode *rom) {
	int i = 0;
	owResetCmd();
	owSendByte(ONEWIRE_MATCH_ROM);
	for (; i < 8; i++) owSendByte(*(((uint8_t *) rom) + i));
}

void owConvertTemperatureCmd(OneWire *ow, RomCode *rom) {
	owMatchRomCmd(rom);
	owSendByte(ONEWIRE_CONVERT_TEMPERATURE);
}

uint8_t *owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data) {
	uint16_t b = 0, p;
	switch (rom->family) {
	case DS18B20:
	case DS18S20:
		p = 72;  //9*8 =72 mhz
		break;
    default:
    	return data;
  }
  owMatchRomCmd(rom);
  owSendByte(ONEWIRE_READ_SCRATCHPAD);
  while (b < p) {
	  uint8_t pos = (uint8_t) ((p - 8) / 8 - (b / 8));
	  uint8_t bt;
	  owSend(OW_READ);
	  bt = owReadSlot(owEchoRead());
	  if (bt == 1) data[pos] |= 1 << b % 8; else data[pos] &= ~(1 << b % 8);
	  b++;//следующий бит
  }
  return data;
}

void owWriteDS18B20Scratchpad(OneWire *ow, RomCode *rom, uint8_t th, uint8_t tl, uint8_t conf) {
	if (rom->family != DS18B20) return;
	owMatchRomCmd(rom);
	owSendByte(ONEWIRE_WRITE_SCRATCHPAD);
	owSendByte(th);
	owSendByte(tl);
	owSendByte(conf);
}

/**
 * Get last mesaured temperature from DS18B20 or DS18S20. These temperature MUST be measured in previous
 * opearions. If you want to measure new value you can set reSense in true. In this case next invocation
 * that method will return value calculated in that step.
 * @param ow -- OneWire bus pointer
 * @param rom -- selected device
 * @param reSense -- do you want resense temp for next time?
 * @return struct with data
 */
Temperature readTemperature(OneWire *ow, RomCode *rom, uint8_t reSense) {
	Scratchpad_DS18B20 *sp;
	Scratchpad_DS18S20 *spP;
	Temperature t;
	uint8_t pad[9];
	t.inCelsus = 0x00;
	t.frac = 0x00;
	sp = (Scratchpad_DS18B20 *) &pad;
	spP = (Scratchpad_DS18S20 *) &pad;

	switch (rom->family) {
	case DS18B20:
		owReadScratchpadCmd(ow, rom, pad);
		t.inCelsus = (int8_t) (sp->temp_msb << 4) | (sp->temp_lsb >> 4);
		t.frac = (uint8_t) ((((sp->temp_lsb & 0x0F)) * 10) >> 4);
		break;
    case DS18S20:
        owReadScratchpadCmd(ow, rom, pad);
        t.inCelsus = spP->temp_lsb >> 1;
        t.frac = (uint8_t) 5 * (spP->temp_lsb & 0x01);
        break;
    default:
    	return t;
	}

	if (reSense) owConvertTemperatureCmd(ow, rom);
	return t;
}

void owCopyScratchpadCmd(OneWire *ow, RomCode *rom) {
	owMatchRomCmd(rom);
	owSendByte(ONEWIRE_COPY_SCRATCHPAD);
}

void owRecallE2Cmd(OneWire *ow, RomCode *rom) {
	owMatchRomCmd(rom);
	owSendByte(ONEWIRE_RECALL_E2);
}
