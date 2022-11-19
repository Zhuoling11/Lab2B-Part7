#include "hardware/pio.h"                                                               //Standard RP2040 library
#include "pico/stdlib.h"                                                                //Standard C library
#include <stdio.h>                                                                      //Standard C library
#include "stdlib.h"      
#include "hardware/dma.h"                                                               //Standard C library
#include "hardware/gpio.h"                                                              //Standard RP2040 library
#include "hardware/structs/sio.h"                                                       //Standard RP2040 library                                                         
#include "neopixel.h"    
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/pwm.h"
#include <piosequencer.pio.h>
#include <math.h>
                                                  

#define QTPY_BOOT_PIN 21

typedef          uint32_t   VALUE;
typedef volatile uint32_t* ADDRESS;


VALUE register_read(ADDRESS  address) {
    return *address;                                    //Reading and returing the register value
}
void register_write(ADDRESS address, VALUE value) {
    *address = value;                                   //Writing value to register
}

const uint CAPTURE_PIN_BASE = 21;
const uint CAPTURE_PIN_COUNT = 1;
const uint CAPTURE_N_SAMPLES = 8192;


void play(bool arr[], int seconds, char color)
{
  stdio_init_all();
  gpio_init(23);
  gpio_set_dir(23,GPIO_OUT);                                        //Initialization functions
    uint32_t light_color;

    switch(color){
            case 'r':
                light_color = 0x00ff0000;
                break;
            case 'g':
                light_color = 0x0000ff00;
                break;
            case 'b':
                light_color = 0x000000ff;
                break;
                default : 
                light_color = 0x000000ff;
    
    }

for(int j=0;j<=8192;j=j+1)                            
            {
                if(arr[j]==true)
                {
                    neopixel_set_rgb(light_color);
                    gpio_put(23,1);
                    sleep_us(500);
                }

                else
                {
                   neopixel_set_rgb(0x00000000);
                    gpio_put(23,0);
                    sleep_us(500);
                }
            }

neopixel_set_rgb(0x00000000);
gpio_put(23,0);
}

static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr = pio_encode_in(pio_pins, pin_count);
    struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, div);
    // Note that we may push at a < 32 bit threshold if pin_count does not
    // divide 32. We are using shift-to-right, so the sample data ends up
    // left-justified in the FIFO in this case, with some zeroes at the LSBs.
    sm_config_set_in_shift(&c, true, true, bits_packed_per_word(pin_count));
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}

void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);
}

void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples, uint state_m) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    printf("Capture:\n");
    bool buffer[10000];
    int count = 0;
    // Each FIFO record may be only partially filled with bits, depending on
    // whether pin_count is a factor of 32.
    uint record_size_bits = bits_packed_per_word(pin_count);
    for (int pin = 0; pin < pin_count; ++pin) {
        printf("%02d: ", pin + pin_base);
        for (int sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            uint word_index = bit_index / record_size_bits;
            uint word_mask = 1u << (bit_index % record_size_bits + 32 - record_size_bits);
            printf(buf[word_index] & word_mask ? "-" : "_");
            buffer[count] = !(buf[word_index] & word_mask);
            count++;
        }
        printf("\n");
    }
    play(buffer,4,'r');
}
int main(){
 
    stdio_init_all();     
    neopixel_init();                                                              
    uint total_sample_bits = CAPTURE_N_SAMPLES * CAPTURE_PIN_COUNT;
    total_sample_bits += bits_packed_per_word(CAPTURE_PIN_COUNT) - 1;
    uint buf_size_words = total_sample_bits / bits_packed_per_word(CAPTURE_PIN_COUNT);
    uint32_t *capture_buf = malloc(buf_size_words * sizeof(uint32_t));
    hard_assert(capture_buf);                                             
    while (!stdio_usb_connected()) {
      printf(".");
      sleep_ms(500);
    }

     bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    PIO pio = pio0;
    uint sm = 0;
    uint dma_chan = 0;

    logic_analyser_init(pio, sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 65536.f);

    
    uint offset1 = pio_add_program(pio1, &piosequencer_program);
    uint sm1 = 1;
    pio_seq_init(pio0, sm1, offset1, 23);
    printf("\nYou can press the boot button now\n");
    while(true){
    sleep_ms(1);
    if(!(gpio_get(QTPY_BOOT_PIN))){
    // printf("pressed already\n");
    logic_analyser_arm(pio, sm, dma_chan, capture_buf, buf_size_words, CAPTURE_PIN_BASE, true);
    printf("boot pressed! recording in progress\n");
    dma_channel_wait_for_finish_blocking(dma_chan);
    logic_analyser_arm(pio, sm, dma_chan, capture_buf, buf_size_words, CAPTURE_PIN_BASE, false);
    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES,sm1);
    }}

    
}

