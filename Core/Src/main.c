#include "main.h"
#include <stdio.h>
#include <string.h>
#include <malloc.h>
char rx_buffer[8];

char tranfer_complete_flag = 0;
void UART1_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	uint32_t* MODER = (uint32_t*)(0x40020400);
	*MODER |= (0b10 << 12) | (0b10 << 14);		//set PB6 (U1Tx), PB7(U1Rx)

	uint32_t* AFRL = (uint32_t*)(0x40020420);
	*AFRL  |= (0b0111 << 24) | (0b0111 << 28);

	uint32_t* BRR = (uint32_t*)(0x40011008);
	*BRR = (8<<4) | 10;

	uint32_t* CR3 = (uint32_t*)(0x40011014);
	*CR3 |= 1 << 6;							//enable DMA for receiver

	uint32_t* CR1 = (uint32_t*)(0x4001100c);
	*CR1 |= (1<< 3)|(1<<2)|(1<<13);
}


void DMA_Init(void* rx_buff, int buff_size)
{
	tranfer_complete_flag = 0;
	__HAL_RCC_DMA2_CLK_ENABLE();
	uint32_t* S2CR = (uint32_t*)(0x40026440);
	if(*S2CR & 1)
		*S2CR &= ~(1<<0);

	uint32_t* S2PAR = (uint32_t*)(0x40026448);
	*S2PAR = 0x40011004;

	uint32_t* S2M0AR = (uint32_t*)(0x4002644c);
	*S2M0AR = (uint32_t)rx_buff;

	uint32_t* S2NDTR = (uint32_t*)(0x40026444);
	*S2NDTR = buff_size;

	*S2CR = (0b100 << 25) | (1 << 10) | (1 << 4) | 1;

	uint32_t* ISER1 = (uint32_t*)(0xe000e104);
	*ISER1 |= 1 << (58 - 32);	//enable interrupt for DMA2_Steam2
}

void DMA2_Stream2_IRQHandler()
{
	uint32_t* DMA_LIFCR = (uint32_t*)(0x40026408);
	uint32_t* S2NDTR = (uint32_t*)(0x40026444);
	if(*S2NDTR == 0)
		tranfer_complete_flag = 1;
	*DMA_LIFCR |= 1 << 21;


}
void UART1_Send(char data)
{
	uint32_t* SR = (uint32_t*)(0x40011000);
	uint32_t* DR = (uint32_t*)(0x40011004);
	while(((*SR >> 7) & 1) != 1);
	*DR	= data;
	while(((*SR >> 6) & 1) != 1);
	*SR &= ~(1 << 6);
}

int __io_putchar(int ch)
{
	UART1_Send(ch);
	return 0;
}

#define FLASH_BASE_ADDR	 0x40023C00
__attribute__((section(".RamFunc"))) void SectorErase(int sec_num)
{
    if((sec_num < 0)||(sec_num > 7))
        return;
	/*
	To erase a sector, follow the procedure below:
		1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
		FLASH_SR register
		2. Set the SER bit and select the sector out of the 8 sectors (STM32F411xC/E) in the
		main memory block you wish to erase (SNB) in the FLASH_CR register
		3. Set the STRT bit in the FLASH_CR register
		4. Wait for the BSY bit to be cleared
	*/
    // wait FLASH INTERFACE ready to use by checking BSY in SR
	uint32_t* FLASH_SR = (uint32_t*)(FLASH_BASE_ADDR + 0x0C);
    while(((*FLASH_SR >> 16)&1) == 1);
    uint32_t* FLASH_CR = (uint32_t*)(FLASH_BASE_ADDR + 0x10);
    // check CR is locked?. if locked, unlock
    if((*FLASH_CR >> 31) & 1) // is locked
    {
        // unlock by the unlock sequence
        /*
        The following sequence is used to unlock this register:
            1. Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
            2. Write KEY2 = 0xCDEF89AB in the Flash key register (FLASH_KEYR)
        */
        uint32_t* FLASH_KEYR = (uint32_t*)(FLASH_BASE_ADDR + 0x04);
        *FLASH_KEYR = 0x45670123;
        *FLASH_KEYR = 0xCDEF89AB;
    }

    // choose SECTOR ERASE operation AND choose sector number to erase
    *FLASH_CR |= (1<<1) | (sec_num << 3);
    // start erase
    *FLASH_CR |= 1 << 16;
    // wait erasing finish
    while(((*FLASH_SR >> 16)&1) == 1);
}

__attribute__((section(".RamFunc"))) void ProgramData(void* addr, void* data_ptr, int data_size)
{
    /*
    The Flash memory programming sequence is as follows:
        1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the
        FLASH_SR register.
        2. Set the PG bit in the FLASH_CR register
        3. Perform the data write operation(s) to the desired memory address (inside main
        memory block or OTP area):
            - Byte access in case of x8 parallelism
            - Half-word access in case of x16 parallelism
            - Word access in case of x32 parallelism
            - Double word access in case of x64 parallelism
        4. Wait for the BSY bit to be cleared.

    */
    // wait FLASH INTERFACE ready to use by checking BSY in SR
	uint32_t* FLASH_SR = (uint32_t*)(FLASH_BASE_ADDR + 0x0C);
    while(((*FLASH_SR >> 16)&1) == 1);
    uint32_t* FLASH_CR = (uint32_t*)(FLASH_BASE_ADDR + 0x10);
    // check CR is locked?. if locked, unlock
    if((*FLASH_CR >> 31) & 1) // is locked
    {
        // unlock by the unlock sequence
        /*
        The following sequence is used to unlock this register:
            1. Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
            2. Write KEY2 = 0xCDEF89AB in the Flash key register (FLASH_KEYR)
        */
        uint32_t* FLASH_KEYR = (uint32_t*)(FLASH_BASE_ADDR + 0x04);
        *FLASH_KEYR = 0x45670123;
        *FLASH_KEYR = 0xCDEF89AB;
    }
    // choose PROGRAM operation by set PG in CR register
    *FLASH_CR |= 1 << 0;
    // Perform the data write operation(s)
    char* dir = addr;
    char* src = data_ptr;
    for(int i = 0; i < data_size; i++)
    {
        dir[i] = src[i];
    }
     // wait erasing finish
    while(((*FLASH_SR >> 16)&1) == 1);
}

__attribute__((section(".RamFunc"))) void UpdateFirmware(void* data_ptr, int data_size)
{
	SectorErase(0);
	ProgramData((void*)0x08000000, data_ptr, data_size);
	uint32_t* AIRCR = (uint32_t*)(0xE000ED0C);
	*AIRCR |= (0x5FA << 16) | (1<<2); // SYSRESETREQ
	while(1);
}
int main()
{
//	HAL_Init();
	UART1_Init();
	DMA_Init(rx_buffer, sizeof(rx_buffer));
	printf("Vui long gui kich thuoc FW:\r\n");
	while(strstr(rx_buffer, "\r\n") == NULL);
	int fw_size = 0;
	sscanf(rx_buffer, "%d", &fw_size);
	printf("kich thuoc FW chuan nhan la: %d\r\n", fw_size);
	char* new_fw = malloc(fw_size);
	DMA_Init(new_fw, fw_size);
	while(tranfer_complete_flag == 0);
	printf("da nhan du %d bytes\r\n", fw_size);
	UpdateFirmware(new_fw, fw_size);

	while(1)
	{

	}
	return 0;
}
