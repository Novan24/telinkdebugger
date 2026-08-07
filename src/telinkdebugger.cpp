// SPDX-License-Identifier: MIT
/*

Copyright (c) 2024 David Given dg@cowlark.com
*/


#include <stdio.h>
#include <stdlib.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/divider.h"

#include "sws.pio.h"
#include "globals.h"

#define FLASH_DUMP_TOTAL 0x100000u

#define SWS_PIN 2
#define RST_PIN 3
#define DBG_PIN 4
#define LED_PIN PICO_DEFAULT_LED_PIN

#define SM_RX 0
#define SM_TX 1

#define DEBUG_SWS 0
#define BUFFER_SIZE_BITS 4096

#define REG_ADDR8(n) (n)
#define REG_ADDR16(n) (n)
#define REG_ADDR32(n) (n)

#define reg_soc_id REG_ADDR16(0x7e)

#define reg_swire_data REG_ADDR8(0xb0)
#define reg_swire_ctrl1 REG_ADDR8(0xb1)
#define reg_swire_clk_div REG_ADDR8(0xb2)
#define reg_swire_id REG_ADDR8(0xb3)

#define reg_tmr_ctl REG_ADDR32(0x620)
#define FLD_TMR_WD_EN (1 << 23)

#define reg_debug_runstate REG_ADDR8(0x602)

static uint32_t input_buffer[BUFFER_SIZE_BITS / 8];
static uint32_t output_buffer[BUFFER_SIZE_BITS / 8];

static int input_buffer_bit_ptr;
static int output_buffer_bit_ptr;

static int sws_tx_program_offset;
static int sws_rx_program_offset;

static bool is_connected;

static uint8_t page_buffer[256];
static uint32_t bytes_programmed = 0;

static void write_nine_bit_byte(uint16_t byte)
{
pio_gpio_init(pio0, SWS_PIN);
pio_interrupt_clear(pio0, 0);

pio_sm_put(pio0, SM_TX, byte);  

while (!pio_interrupt_get(pio0, 0))  
    ;  
pio_interrupt_clear(pio0, 0);

}

static void write_cmd_byte(uint8_t byte)
{
#if DEBUG_SWS
printf("[CMD] %02X\n", byte);
#endif
write_nine_bit_byte(0x100 | byte);
}

static void write_data_byte(uint8_t byte)
{
#if DEBUG_SWS
printf("[DATA] %02X\n", byte);
#endif
write_nine_bit_byte(0x000 | byte);
}

static void write_data_word(uint16_t word)
{
write_data_byte(word >> 8);
write_data_byte(word & 0xff);
}

static uint8_t read_byte()
{
pio_gpio_init(pio1, SWS_PIN);
pio_gpio_init(pio1, DBG_PIN);
pio_sm_clear_fifos(pio1, SM_RX);
pio_sm_exec_wait_blocking(pio1, SM_RX, sws_rx_program_offset); // JMP offset

uint8_t value = pio_sm_get_blocking(pio1, SM_RX);  
  
#if DEBUG_SWS  
    printf("# sws rx = 0x%02X\n", value);  
#endif  
return value;

}
static bool read_byte_timeout(uint8_t* value, uint32_t timeout_ms)
{
    pio_gpio_init(pio1, SWS_PIN);
    pio_gpio_init(pio1, DBG_PIN);

    pio_sm_clear_fifos(pio1, SM_RX);
    pio_sm_exec_wait_blocking(pio1, SM_RX, sws_rx_program_offset);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (pio_sm_is_rx_fifo_empty(pio1, SM_RX))
    {
        if (time_reached(deadline))
            return false;
    }

    *value = (uint8_t)pio_sm_get(pio1, SM_RX);

#if DEBUG_SWS
    printf("# sws rx = 0x%02X\n", *value);
#endif

    return true;
}

static bool read_first_debug_byte(
    uint16_t address,
    uint8_t* value)
{
write_cmd_byte(0x5a);
write_data_word(address);
write_data_byte(0x80);

return read_byte_timeout(value, SWS_TIMEOUT_MS);
}
static bool read_first_debug_byte_timeout(uint16_t address, uint8_t* value, uint32_t timeout_ms)
{
    write_cmd_byte(0x5a);
    write_data_word(address);
    write_data_byte(0x80);

    return read_byte_timeout(value, timeout_ms);
}

bool read_next_debug_byte(uint8_t* value)
{
    return read_byte_timeout(value, SWS_TIMEOUT_MS);
}

static bool read_next_debug_byte_timeout(uint8_t* value, uint32_t timeout_ms)
{
    return read_byte_timeout(value, timeout_ms);
}

static void finish_reading_debug_bytes()
{
write_cmd_byte(0xff);
}

static uint8_t read_single_debug_byte(uint16_t address)
{
uint8_t value = read_first_debug_byte(address);
finish_reading_debug_bytes();
return value;
}

static bool read_single_debug_byte_timeout(
    uint16_t address,
    uint8_t* value,
    uint32_t timeout_ms)
{
    if (!read_first_debug_byte_timeout(address, value, timeout_ms))
    {
        finish_reading_debug_bytes();
        return false;
    }

    finish_reading_debug_bytes();
    return true;
}

static uint16_t read_single_debug_word(uint16_t address)
{
uint8_t v1 = read_first_debug_byte(address);
uint8_t v2 = read_next_debug_byte();
finish_reading_debug_bytes();
return v1 | (v2 << 8);
}

static void write_first_debug_byte(uint16_t address, uint8_t value)
{
write_cmd_byte(0x5a);
write_data_word(address);
write_data_byte(0x00);
write_data_byte(value);
}

static void write_next_debug_byte(uint8_t value)
{
write_data_byte(value);
}

static void finish_writing_debug_bytes()
{
write_cmd_byte(0xff);
}

static void write_multiple_debug_bytes(
    uint16_t address,
    const uint8_t* data,
    uint16_t len)
{
    if (len == 0)
        return;

    write_first_debug_byte(address, data[0]);

    for (uint16_t i = 1; i < len; i++)
        write_next_debug_byte(data[i]);

    finish_writing_debug_bytes();
}

static void write_single_debug_byte(uint16_t address, uint8_t value)
{
write_first_debug_byte(address, value);
finish_writing_debug_bytes();
}

static void write_single_debug_word(uint16_t address, uint16_t value)
{
write_first_debug_byte(address, value);
write_next_debug_byte(value >> 8);
finish_writing_debug_bytes();
}

static void write_single_debug_quad(uint16_t address, uint32_t value)
{
write_first_debug_byte(address, value);
write_next_debug_byte(value >> 8);
write_next_debug_byte(value >> 16);
write_next_debug_byte(value >> 24);
finish_writing_debug_bytes();
}

static void flash_cs_low()
{
write_single_debug_byte(0x000d, 0x00);
}

static void flash_cs_high()
{
write_single_debug_byte(0x000d, 0x01);
}

static void flash_read_start(uint32_t addr)
{
flash_cs_low();

// Flash READ command  
write_single_debug_byte(0x000c, 0x03);  

// 24-bit address  
write_single_debug_byte(0x000c, (addr >> 16) & 0xff);  
write_single_debug_byte(0x000c, (addr >> 8) & 0xff);  
write_single_debug_byte(0x000c, addr & 0xff);  

// FIFO mode  
write_single_debug_byte(0x00b3, 0x80);

}

static uint8_t flash_read_next()
{
write_single_debug_byte(0x000c, 0xff);

return read_single_debug_byte(0x000c);

}

static void flash_read_end()
{
write_single_debug_byte(0x00b3, 0x00);

flash_cs_high();

}

static void flash_read_jedec_id(void)
{
    flash_cs_low();

    // JEDEC ID command
    write_single_debug_byte(0x000c, 0x9F);

    // FIFO mode
    write_single_debug_byte(0x00b3, 0x80);

uint8_t mfr, type, cap;

// Manufacturer
write_single_debug_byte(0x000c, 0xFF);
if (!read_single_debug_byte_timeout(0x000c, &mfr, 50))
{
    printf("# timeout waiting manufacturer byte\n");
    write_single_debug_byte(0x00b3, 0x00);
    flash_cs_high();
    printf("E\n");
    return;
}

// Memory Type
write_single_debug_byte(0x000c, 0xFF);
if (!read_single_debug_byte_timeout(0x000c, &type, 50))
{
    printf("# timeout waiting memory type\n");
    write_single_debug_byte(0x00b3, 0x00);
    flash_cs_high();
    printf("E\n");
    return;
}

// Capacity
write_single_debug_byte(0x000c, 0xFF);
if (!read_single_debug_byte_timeout(0x000c, &cap, 50))
{
    printf("# timeout waiting capacity\n");
    write_single_debug_byte(0x00b3, 0x00);
    flash_cs_high();
    printf("E\n");
    return;
}

    write_single_debug_byte(0x00b3, 0x00);

    flash_cs_high();

    printf("# JEDEC ID = %02X %02X %02X\n",
        mfr, type, cap);

    // Beberapa flash umum
    if (mfr == 0xC8)
        printf("# Manufacturer : GigaDevice\n");
    else if (mfr == 0xEF)
        printf("# Manufacturer : Winbond\n");
    else if (mfr == 0x20)
        printf("# Manufacturer : Micron/ST\n");
    else if (mfr == 0x1C)
        printf("# Manufacturer : EON\n");
    else
        printf("# Manufacturer : Unknown\n");

    printf("S\n");
}

static void flash_write_enable()
{
    flash_cs_low();

// SPI Flash: Write Enable  
    write_single_debug_byte(0x000c, 0x06);  

    flash_cs_high();

}
static uint8_t flash_read_status()
{
    uint8_t status;

    flash_cs_low();  

// Read Status Register-1  
    write_single_debug_byte(0x000c, 0x05);  

// FIFO read mode  
    write_single_debug_byte(0x00b3, 0x80);  

// Dummy byte  
    write_single_debug_byte(0x000c, 0xff);  

    status = read_single_debug_byte(0x000c);  

    write_single_debug_byte(0x00b3, 0x00);  

    flash_cs_high();  

    return status;

}
static void flash_wait_busy()
{
while (flash_read_status() & 0x01);
}

static void flash_page_program(uint32_t addr, const uint8_t* data, uint16_t len)
{
    flash_write_enable();

    flash_cs_low();

    // Page Program
    write_single_debug_byte(0x000c, 0x02);

    // Address
    write_single_debug_byte(0x000c, (addr >> 16) & 0xff);
    write_single_debug_byte(0x000c, (addr >> 8) & 0xff);
    write_single_debug_byte(0x000c, addr & 0xff);

    // FIFO mode (sesuai implementasi Python)
    write_single_debug_byte(0x00b3, 0x80);

    // Kirim seluruh data dalam SATU transaksi SWS
    write_multiple_debug_bytes(0x000c, data, len);

    // Kembali ke RAM mode
    write_single_debug_byte(0x00b3, 0x00);

    flash_cs_high();

    flash_wait_busy();
}

static void flash_program(uint32_t addr, const uint8_t* data, uint32_t len)
{
while (len)
{
uint16_t page_remaining = 256 - (addr & 0xff);

uint16_t chunk =  
        (len < page_remaining) ? len : page_remaining;  

    flash_page_program(addr, data, chunk);  

    addr += chunk;  
    data += chunk;  
    len -= chunk;  
}

}

static void flash_chip_erase(void)
{
flash_write_enable();

flash_cs_low();  

// CHIP ERASE  
write_single_debug_byte(0x000c, 0xC7);  

flash_cs_high();  

flash_wait_busy();

}

static void flash_program_buffer(uint32_t addr, uint16_t len)
{
printf("# programming %u bytes at %06X\n",
(unsigned)len,
(unsigned)addr);

flash_program(addr, page_buffer, len);  

bytes_programmed += len;  

printf("# total programmed = %lu bytes\n",  
(unsigned long)bytes_programmed);  

printf("# done\n");

}

static char get_hex_char(void)
{
    char c;

    do
    {
        c = getchar();
    }
    while (c == '\r' || c == '\n');

    return c;
}

static uint8_t read_hex_byte(void)
{
    char buffer[3];

    buffer[0] = get_hex_char();
    buffer[1] = get_hex_char();
    buffer[2] = 0;

    return strtoul(buffer, nullptr, 16);
}

static uint16_t read_hex_word()
{
uint8_t hi = read_hex_byte();
uint8_t lo = read_hex_byte();
return lo | (hi << 8);
}

static uint32_t read_hex_addr24()
{
uint8_t b2 = read_hex_byte();
uint8_t b1 = read_hex_byte();
uint8_t b0 = read_hex_byte();

return ((uint32_t)b2 << 16) |  
       ((uint32_t)b1 << 8) |  
        b0;

}

static uint32_t read_hex_dword()
{
uint8_t b3 = read_hex_byte();
uint8_t b2 = read_hex_byte();
uint8_t b1 = read_hex_byte();
uint8_t b0 = read_hex_byte();

return ((uint32_t)b3 << 24) |  
       ((uint32_t)b2 << 16) |  
       ((uint32_t)b1 << 8)  |  
        b0;

}

static bool flash_verify_buffer(uint32_t addr, uint16_t len)
{
printf("# verifying %u bytes at %06X\n",
(unsigned)len,
(unsigned)addr);

flash_read_start(addr);  

for (uint16_t i = 0; i < len; i++)  
{  
    uint8_t actual = flash_read_next();  

    if (actual != page_buffer[i])  
    {  
        flash_read_end();  

        printf("# verify failed\n");  
        printf("# addr     = %06X\n", (unsigned)(addr + i));  
        printf("# expected = %02X\n", page_buffer[i]);  
        printf("# actual   = %02X\n", actual);  

        return false;  
    }  
}  

flash_read_end();  

printf("# verify ok\n");  

return true;

}

static bool flash_program_stream(uint32_t addr, uint32_t len)
{
    while (len)
    {
        uint16_t chunk = (len > 256) ? 256 : len;

        for (uint16_t i = 0; i < chunk; i++)
            page_buffer[i] = read_hex_byte();
        
        printf("# %lu / %lu\n",
       (unsigned long)bytes_programmed,
       (unsigned long)(bytes_programmed + len));
    
uint32_t current_page = (addr >> 8) + 1;
uint32_t total_pages = (FLASH_DUMP_TOTAL >> 8);

printf("# Page %lu/%lu  Addr=%06X\n",
       (unsigned long)current_page,
       (unsigned long)total_pages,
       (unsigned)addr);
        

        flash_program(addr, page_buffer, chunk);

        if (!flash_verify_buffer(addr, chunk))
        {
            printf("# verify failed at %06X\n", (unsigned)addr);
            return false;
        }

        bytes_programmed += chunk;

        addr += chunk;
        len -= chunk;
    }

    return true;
}

static void flash_chip_erase_test()
{
printf("# WARNING: CHIP ERASE\n");
printf("# This may take several seconds...\n");

flash_chip_erase();  

printf("# chip erase done\n");

}

static void halt_target()
{
write_single_debug_byte(reg_debug_runstate, 0x05);
}

static void banner()
{
printf(
"Telink Bridge RP2040 Firmware\n"
"-\n"
"Original Author : David Given\n"
"Fork : Novan24\n"
"Firmware mod version : 3.3\n"
" \n"
"A : Address, L : Length, D : Hex Data\n"
"Main Commands  :\n"
"i            Initialize Connection\n"
"J            Read JEDEC flash ID\n"
"A   Stream Program (Auto Verify)\n"
"       Usage : A AAAAAALLLLLLDD.. (No space)\n"
"D   Dump Flash\n"
"       Usage : DAAAAALLLL\n"
"F   Dump Entire Flash\n"
"C   Chip Erase (Erase Entire Flash!)\n"
"  \n"

"Advanced Commands :\n"
"B   Read Status Register\n"
"E   Flash Write Enable\n"
"rX  X=[0, 1] Set status of reset pin\n"
"g   Pulse reset\n"
"s   Read SoC ID\n"
"U   Program Flash Bytes\n"
"       Usage : UAAAAAALLLLDD..\n"
"Y   Verify Flash Bytes\n"
"       Usage : YAAAAAALLLLDD..\n"
"R   Read Debug Memory\n"
"       Usage : RXXXXYYYY\n"
"       (Read YYYY bytes from XXXX (Hex))\n"
"W   Write Debug Memory\n"
"       Usage : WXXXXYYYYDD.. \n"
"       (Write YYYY bytes to XXXX, followed by Hex data)\n"
"  \n"
"Info\n"
"Always Run : 'i' Initialize Connection, After first BOOT before using other commands.\n"
"S For success, E for error, # are comments\n"
"If no S or E returned, The program has DeadLocked. Reconnect the USB device.\n"
"Good luck! you'll need it. :)\n");
}

static void init_cmd()
{
printf("# init\n");

gpio_put(RST_PIN, false);  
sleep_ms(20);  
gpio_put(RST_PIN, true);  
sleep_ms(20);  
  
printf("# reset done\n");  
  
halt_target();  
  
printf("# halted\n");  

uint16_t socid = read_single_debug_word(reg_soc_id);  
printf("# socid = 0x%04x\n", socid);  
if (socid == 0x5316)  
{  
    printf("S\n");  
    is_connected = true;  

    /* Disable the watchdog timer. */  

    write_single_debug_quad(reg_tmr_ctl, 0);  
    return;  
}  

printf("E\n# init failed\n");

}

void set_tx_clock(double clock_hz)
{
sws_tx_program_init(pio0, SM_TX, sws_tx_program_offset, SWS_PIN, clock_hz);
pio_sm_set_enabled(pio0, SM_TX, true);
}
static void flash_dump_range(uint32_t addr, uint32_t count, bool print_status)
{
flash_read_start(addr);

for (uint32_t i = 0; i < count; i++)  
{  
    if ((i % 16) == 0)  
        printf("%06X: ", (unsigned)(addr + i));  

    printf("%02X ", flash_read_next());  

    if ((i % 16) == 15 || i == count - 1)  
        printf("\n");  
}  

flash_read_end();  

if (print_status)  
    printf("S\n");

}

static void flash_dump(uint32_t addr, uint16_t count)
{
printf("# flash dump addr=0x%06X len=%u\n",
(unsigned)addr,
(unsigned)count);

flash_dump_range(addr, count, true);

}

static void flash_dump_all(void)
{
printf("# full flash dump start\n");
flash_dump_range(0x000000, FLASH_DUMP_TOTAL, false);
printf("S\n");
}

int main(void)
{
usb_bridge_init();
stdio_queue_init();

gpio_init(RST_PIN);  
gpio_set_dir(RST_PIN, true);  
gpio_put(RST_PIN, false);  

gpio_set_pulls(RST_PIN, false, false);  
gpio_set_pulls(SWS_PIN, true, false);  
gpio_set_pulls(DBG_PIN, false, false);  

sws_tx_program_offset = pio_add_program(pio0, &sws_tx_program);  
set_tx_clock(10.0e6);  

sws_rx_program_offset = pio_add_program(pio1, &sws_rx_program);  
sws_rx_program_init(pio1, SM_RX, sws_rx_program_offset, SWS_PIN);  
pio_sm_set_enabled(pio1, SM_RX, true);  

banner();  
for (;;)  
{  
    int c = getchar();  
    switch (c)  
    {  
        case '\r':  
        case '\n':  
            break;  
        case 'i':  
            init_cmd();  
            break;  

        case 'r':  
        {  
            int i = getchar() == '1';  
            printf("# reset <- %d\n", i);  
            gpio_put(RST_PIN, i);  
            if (i == 0)  
                is_connected = false;  
            printf("S\n");  
            break;  
        }

case 'B':
{
if (!is_connected)
{
printf("E\n");
break;
}

printf("# status = %02X\n", flash_read_status());  
printf("S\n");  
break;

}

case 'g':  
        {  
            gpio_put(RST_PIN, 0);  
            sleep_us(100);  
            gpio_put(RST_PIN, 1);  
            sleep_us(100);  
            break;  
        }  

        case 's':  
        {  
            uint16_t socid = read_single_debug_word(reg_soc_id);  
            printf("# socid = %04x\nS\n", socid);  
            break;  
        }  

        case 'D':  
            {  
            if (!is_connected)  
        {  
            printf("# not connected\nE\n");  
            break;  
        }  

            uint32_t addr = read_hex_addr24();  
            uint16_t count = read_hex_word();  

            flash_dump(addr, count);  
        break;  
        }  

        case 'F':  
            {  
            if (!is_connected)  
        {  
            printf("# not connected\nE\n");  
            break;  
        }  

            flash_dump_all();  
            break;  
        }  

        case 'E':

{
if (!is_connected)
{
printf("# not connected\n");
printf("E\n");
break;
}

flash_write_enable();

sleep_us(20);

printf("# status = %02X\n", flash_read_status());

printf("S\n");  
break;
}

case 'C':
{
if (!is_connected)
{
printf("E\n");
break;
}

flash_chip_erase_test();  

printf("S\n");  
break;

}
case 'U':
{
if (!is_connected)
{
printf("E\n");
break;
}

uint32_t addr = read_hex_addr24();  
uint16_t len = read_hex_word();  

if (len > 256)  
{  
    printf("# max 256 bytes\n");  
    printf("E\n");  
    break;  
}  

for (uint16_t i = 0; i < len; i++)  
    page_buffer[i] = read_hex_byte();  

flash_program_buffer(addr, len);  

printf("S\n");  
break;

}

case 'Y':
{
if (!is_connected)
{
printf("E\n");
break;
}

uint32_t addr = read_hex_addr24();  
uint16_t len = read_hex_word();  

if (len > 256)  
{  
    printf("# max 256 bytes\n");  
    printf("E\n");  
    break;  
}  

for (uint16_t i = 0; i < len; i++)  
    page_buffer[i] = read_hex_byte();  

if (flash_verify_buffer(addr, len))  
    printf("S\n");  
else  
    printf("E\n");  

break;

}

case 'A':
{
if (!is_connected)
{
printf("# not connected\n");
printf("E\n");
break;
}

uint32_t addr = read_hex_addr24();  
uint32_t len  = read_hex_dword();  

//check
if (len == 0)
{
    printf("# invalid length\n");
    printf("E\n");
    break;
}

printf("# stream program\n");  
printf("# addr  = %06X\n", (unsigned)addr);  
printf("# bytes = %lu\n", (unsigned long)len);  

bytes_programmed = 0;  

//protection
if (addr + len > FLASH_DUMP_TOTAL)
{
    printf("# address out of range\n");
    printf("E\n");
    break;
}

if (flash_program_stream(addr, len))  
{  
    printf("# total programmed = %lu bytes\n",  
           (unsigned long)bytes_programmed);  

    printf("S\n");  
}  
else  
    printf("E\n");  

break;

}
case 'J':
{
    if (!is_connected)
    {
        printf("E\n");
        break;
    }

    flash_read_jedec_id();
    break;
}

case 'R':  
    {  
        uint16_t address = read_hex_word();  
        uint16_t count = read_hex_word();  

   if (count)  
    {  
        uint8_t b = read_first_debug_byte(address);  
        printf("%02x", b);  
        count--;  

        while (count--)  
    {  
        b = read_next_debug_byte();  
        printf("%02x", b);  
    }  

        finish_reading_debug_bytes();  
        printf("\n");  
    }  
        printf("S\n");  
    break;  
    }  

        case 'W':  
        {  
            uint16_t address = read_hex_word();  
            uint16_t count = read_hex_word();  

            if (count)  
            {  
                uint8_t b = read_hex_byte();  
                write_first_debug_byte(address, b);  
                count--;  

                while (count--)  
                {  
                    b = read_hex_byte();  
                    write_next_debug_byte(b);  
                }  

                finish_writing_debug_bytes();  
            }  

            printf("S\n");  
            break;  
        }  

        case '?':  
            banner();  
            break;  

        default:  
            printf("?\n");  
            printf("# unknown command\n");  
    }  
}

}