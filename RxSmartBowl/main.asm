;
; RxSmartBowl.asm
;
; Created: 2022-12-30 10:41:14 AM
; Author : hagen
;
	.cseg
	.org	0x00
	
	
;; registers used by the program	
	.def    r_zero = r0             ; always zero
	.def    r_unprotect = r1        ; set to unprotect signature
    .def    r_tick_count = r2       ; 1ms tick count
    .def    r_rssi = r3             ; current rssi
    .def    r_motor_v = r4          ; motor current sense (voltage)
    .def    r_battery_v = r5        ; battery voltage
    .def    r_buttons = r6          ; button press sensor
    .def    r_tag_voltage = r7      ; battery voltage of tag detected
    .def    r_motor_off_diff = r8   ; voltage diff with motor off
    .def    r_motor_on_diff = r9    ; voltage diff with motor on
	.def    r_tmp = r16             ; temporary register
	.def    r_tmp_l = r16
	.def    r_tmp_h = r17
    .def    r_adc_count = r18       ; round robin counter for reading Analog in pins
    .def    r_uart_count = r19      ; count of how many bytes have been read from UART
    .def    r_motor_count = r20     ; count how long motor current was high
    .def    r_mode = r21
    .def    r_cat_search_timer = r22
    .def    r_cat_detected_timer_l = r24
    .def    r_cat_detected_timer_h = r25

    .equ    A_RSSI = 0x07
    .equ    A_MOTOR = 0x03
    .equ    A_BAT = 0x05
    .equ    A_BUTTON = 0x0A

;; macros
    .macro  unlock
    out     CPU_CCP,r_unprotect
    .endmacro

    .macro  cat_led_on
    sbi     VPORTA_OUT,1
    .endmacro

    .macro  cat_led_off
    cbi     VPORTA_OUT,1
    .endmacro

    .macro  battery_led_on
    sbi     VPORTB_OUT,0
    .endmacro

    .macro  battery_led_off
    cbi     VPORTB_OUT,0
    .endmacro

    .macro  open_lid
    sbi     VPORTA_OUT,4
    cbi     VPORTB_OUT,2
    sbr     r_mode,0b00010000
    cbr     r_mode,0b00101100
    .endmacro

    .macro  lid_now_open
    cbi     VPORTA_OUT,4
    cbi     VPORTB_OUT,2
    sbr     r_mode,0b00000100
    cbr     r_mode,0b00111000
    .endmacro

    .macro  close_lid
    cbi     VPORTA_OUT,4
    sbi     VPORTB_OUT,2
    sbr     r_mode,0b00100000
    cbr     r_mode,0b00011100
    .endmacro

    .macro  lid_now_closed
    cbi     VPORTA_OUT,4
    cbi     VPORTB_OUT,2
    sbr     r_mode,0b00001000
    cbr     r_mode,0b00110100
    .endmacro

    .macro  CAT_OPENS_LID
    sbr     r_mode,0b00000010
    sbi     VPORTA_OUT,2        ; turn on door led
    .endmacro

    .macro  CAT_CLOSES_LID
    cbr     r_mode,0b00000010
    cbi     VPORTA_OUT,2        ; turn off door led
    .endmacro

    .macro SKIP_IF_CAT_CLOSES_LID
    sbrc    r_mode,1
    .endmacro

    .macro SKIP_IF_CAT_OPENS_LID
    sbrs    r_mode,1
    .endmacro

    .macro SKIP_IF_LID_NOT_OPENING
    sbrc    r_mode,4
    .endmacro

    .macro SKIP_IF_LID_NOT_CLOSING
    sbrc    r_mode,5
    .endmacro

    .macro SKIP_IF_LID_OPEN
    sbrs    r_mode,2
    .endmacro

    .macro SKIP_IF_LID_CLOSED
    sbrs    r_mode,3
    .endmacro

    .macro BUTTON_PRESSED
    sbr     r_mode,0b10000000
    .endmacro

    .macro BUTTON_NOT_PRESSED
    cbr     r_mode,0b10000000
    .endmacro

    .macro SKIP_IF_BUTTON_NOT_PRESSED
    sbrc    r_mode,7
    .endmacro

    .macro CAT_DETECTED
    sbr     r_mode,0b01000000
    .endmacro

    .macro CAT_NOT_DETECTED
    cbr     r_mode,0b01000000
    .endmacro

    .macro  SKIP_IF_CAT_DETECTED
    sbrs    r_mode,6
    .endmacro

; reset and interupt vector table
    rjmp    power_on_reset          ; RESET 
    rjmp    not_implemented         ; CRCSCAN_NMI     
    rjmp    not_implemented         ; BOD_VLM
    rjmp    not_implemented         ; PORTA_PORT
    rjmp    not_implemented         ; PORTB_PORT
    rjmp    not_implemented         ; not used???
    rjmp    not_implemented         ; RTC_CNT
    rjmp    not_implemented         ; RTC_PIT
    rjmp    not_implemented         ; TCA0_LUNF / TCA0_OVF
    rjmp    not_implemented         ; TCA_HUNF
    rjmp    not_implemented         ; TCA0_LCMP0 / TCA_CMP0
    rjmp    not_implemented         ; TCA0_LCMP1 / TCA_CMP1
    rjmp    not_implemented         ; TCA0_LCMP2 / TCA_CMP2
    rjmp    not_implemented         ; TCB0_INT
    rjmp    not_implemented         ; TCD0_OVF
    rjmp    not_implemented         ; TCD0_TRIG
    rjmp    tx_detected             ; AC0_AC
    rjmp    adc_result_ready        ; ADC0_RESRDY
    rjmp    not_implemented         ; ADC0_WCOMP
    rjmp    not_implemented         ; TWI0_TWIS
    rjmp    not_implemented         ; TWI0_TWIM
    rjmp    not_implemented         ; SPI0_INT
    rjmp    uart_read_byte          ; USART0_RXC
    rjmp    not_implemented         ; USART0_DRE
    rjmp    not_implemented         ; USART0_TXC
    rjmp    not_implemented         ; NVMCTRL_EE

;; initialize and setup
not_implemented:
power_on_reset:
    eor     r_zero,r_zero 
    out     CPU_SREG,r_zero             ; clear status register
    ldi     r_tmp,0xD8                  ; for ccp unprotect registers
    mov     r_unprotect,r_tmp
    out     GPIO_GPIOR0,r_zero          ; reset state machine
    mov     r_motor_count,r_zero        ; reset motor current count

    ldi     XL,LOW(INTERNAL_SRAM_START)
    ldi     XH,HIGH(INTERNAL_SRAM_START)
    ldi     r_tmp,0x00
    ldi     r_tmp_h,LOW(INTERNAL_SRAM_END)
clear_memory:
    st      x+,r_tmp
    cpse    XL,r_tmp_h
    rjmp    clear_memory

    
    ldi     r_tmp_l,LOW(INTERNAL_SRAM_END)  ; set stack pointer to top of memory
    ldi     r_tmp_h,HIGH(INTERNAL_SRAM_END)
    out     CPU_SPL,r_tmp_l
    out     CPU_SPH,r_tmp_h

    ldi     YL,LOW(INTERNAL_SRAM_START+120)  ; end of median buffer
    ldi     YH,HIGH(INTERNAL_SRAM_START+120)
    ldi     ZL,LOW(INTERNAL_SRAM_START+16)  ; median result in buffer
    ldi     ZH,HIGH(INTERNAL_SRAM_START+16)

;; configure clock
;; The clock is set to use the internal 20Mhz clock, with a divide by 10
;; prescaler, so the CPU will be running at 2Mhz

    unlock
    sts     CLKCTRL_MCLKCTRLA,r_zero
    unlock
    ldi     r_tmp,0b00010011            ; clock prescaler enabled, div by 10
    sts     CLKCTRL_MCLKCTRLB,r_tmp   

;; Port A configuration
;; PA0 - not used (used as UPDI)
;; PA1 - Digital Out - Cat LED
;; PA2 - Digital Out - Door LED
;; PA3 - Analog in - Motor current sense
;; PA4 - Digital Out - Motor Controller
;; PA5 - Analog In - Battery voltage
;; PA6 - DAC output
;; PA7 - Analog in - RSSI value

    ldi     r_tmp,0b00010110            ; Set digital output pins
    sts     PORTA_DIR,r_tmp             
    sts     PORTA_OUT,r_zero            ; outputs are set low

    ldi     XL,LOW(PORTA_PIN0CTRL)
    ldi     XH,HIGH(PORTA_PIN0CTRL)
    ldi     r_tmp,0x04                  ; disable input

    st      x+,r_tmp                    ; Pin A0 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A1 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A2 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A3 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A4 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A5 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A6 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin A7 NO ISR, input disabled

;; Port B configuration
;; PB0 - Digital Out, Battery LED
;; PB1 - Analog In, Button sense
;; PB2 - Digital Out, Motor Controller
;; PB3 - Rx Input from Rf receiver
    ldi     r_tmp,0b00000101            ; set digital output pins
    sts     PORTB_DIR,r_tmp
    sts     PORTB_OUT,r_zero            ; outputs are set low

    ldi     XL,LOW(PORTB_PIN0CTRL)
    ldi     XH,HIGH(PORTB_PIN0CTRL)
    ldi     r_tmp,0x04                  ; disable input

    st      x+,r_tmp                    ; Pin B0 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin B1 NO ISR, input disabled
    st      x+,r_tmp                    ; Pin B2 NO ISR, input disabled
    st      x+,r_zero                   ; Pin B3 NO ISR, input enabled

;; UART configuration
    ldi     r_tmp,LOW(1666)             ; 4800 Baud
    ldi     r_tmp_h,HIGH(1666)
    sts     USART0_BAUDL,r_tmp
    sts     USART0_BAUDH,r_tmp_h
    ldi     r_tmp,0b00001011            ; Asynchronous, 2 stop bits, 8 data bits, no parity
    sts     USART0_CTRLC,r_tmp
    ldi     r_tmp,0b00000000            ; disable Rx until we get a tx detection on AC
    sts     USART0_CTRLB,r_tmp
    ldi     r_tmp,0b10000000            ; receive complete interrupt
    sts     USART0_CTRLA,r_tmp

;; VREF setup - 2.5V enable ADC and DAC
    ldi     r_tmp,0x22
    sts     VREF_CTRLA,r_tmp
    ldi     r_tmp,0x03
    sts     VREF_CTRLB,r_tmp

;; DAC setup
    ldi     r_tmp,0b01000001
    sts     DAC0_CTRLA,r_tmp
    ldi     r_tmp,0x8F                  ; new bowl was 0x8f, old bowl was 0x80
    sts     DAC0_DATA,r_tmp

;; AC setup
    ldi     r_tmp,0b00111001            ; Interrupt on positive edge, 50mv hysteresis, low power mode
    sts     AC0_CTRLA,r_tmp
    ldi     r_tmp,0x03                  ; compare with DAC
    sts     AC0_MUXCTRLA,r_tmp
    ldi     r_tmp,0b00000001            ; enable interrupts
    sts     AC0_INTCTRL,r_tmp

;; ADC setup
    ldi     r_tmp,0b00000101            ; 8 bit resolution, enable ADC
    sts     ADC0_CTRLA,r_tmp
    ldi     r_tmp,0b01000010            ; smaller sample cap, clk/4
    sts     ADC0_CTRLC,r_tmp
    ldi     r_tmp,0x1F                  ; sample length
    sts     ADC0_SAMPCTRL,r_tmp
    ldi     r_tmp,A_RSSI                ; start with RSSI 
    sts     ADC0_MUXPOS,r_tmp
    ldi     r_tmp,0x01                  ; enable interrupt for result ready
    sts     ADC0_INTCTRL,r_tmp
    eor     r_adc_count,r_adc_count
    sts     ADC0_COMMAND,r_tmp          ; start a conversion

    ldi     r_tmp,0xFF
    mov     r_tag_voltage,r_tmp
    mov     r_motor_off_diff,r_zero
    mov     r_motor_on_diff,r_zero

    mov     r_mode,r_zero

    ldi     r_tmp,0x01                  ; enable sleep mode = idle
    sts     SLPCTRL_CTRLA,r_tmp

    sei
loop:
    sleep
    rjmp    loop

;; AC interupt, we get this when a Tx Tag has been detected
;; we start the cat search timer at 100ms, flush the Rx buffer
;; and turn on the Rx UART to start receiving bytes from the Tx
tx_detected:
    lds     r_tmp,AC0_STATUS
    sts     AC0_STATUS,r_tmp
    or      r_cat_search_timer,r_cat_search_timer
    brne    already_searching
    cat_led_on                              ; turn on cat led
    ldi     r_cat_search_timer,200          ; search for a cat, maximum 100ms
    ldi     r_uart_count,0                  ; number of bytes read from UART

flush_rx_buffer:
    lds     r_tmp_h,USART0_RXDATAH
    lds     r_tmp,USART0_RXDATAL
    andi    r_tmp_h,0x80
    brne    flush_rx_buffer
    ldi     r_tmp,0x80
    sts     USART0_CTRLB,r_tmp              ; enable Rx
already_searching:
    reti

;; UART Rx byte interupt
uart_read_byte:
    lds     r_tmp_h,USART0_RXDATAH
    lds     r_tmp,USART0_RXDATAL
    andi    r_tmp_h,0x46                    ; test for any errors
    brne    stop_cat_detection
    ldi     r_tmp_h,0xAA
    cpi     r_uart_count,0
    breq    test_byte
    ldi     r_tmp_h,0x55
    cpi     r_uart_count,1
    breq    test_byte
    ldi     r_tmp_h,0x00
    cpi     r_uart_count,2
    breq    test_byte
read_voltage_of_tag:
    mov     r_tag_voltage,r_tmp
    mov     r_tmp,r_rssi
    cpi     r_tmp,0xBC                         ; new bowl was 0xBC, old bowl is 0xA0
    brcs    cat_out_of_threshold                    ; rssi value has to be high enough to start
    CAT_DETECTED
    ldi     r_cat_detected_timer_l,LOW(5000)       ; wait 5 seconds after last cat detection
    ldi     r_cat_detected_timer_h,HIGH(5000)
    rjmp    stop_cat_detection

test_byte:
    cp      r_tmp,r_tmp_h
    brne    stop_cat_detection
    inc     r_uart_count
    reti

cat_out_of_threshold:
    CAT_NOT_DETECTED

stop_cat_detection:
    cat_led_off
    sts     USART0_CTRLB,r_zero
    mov     r_cat_search_timer,r_zero
    reti

;; ADC interupt, this is the main loop
;; all analog inputs are read, when a pass is completed (approx every 1ms)
;; then the state logic is preformed to determine what to do next
adc_result_ready:
    lds     r_tmp,ADC0_RESL
    lds     r_tmp_h,ADC0_RESH
    cpi     r_adc_count,0
    brne    is_it_motor_v
    mov     r_rssi,r_tmp
    ldi     r_tmp,A_MOTOR                  ; read motor voltage next
    rjmp    adc_result_done
is_it_motor_v:
    cpi     r_adc_count,1
    brne    is_it_battery_v
    mov     r_motor_v,r_tmp
    ldi     r_tmp,A_BAT                    ; read battery voltage next
    rjmp    adc_result_done
is_it_battery_v:
    cpi     r_adc_count,2
    brne    it_is_button
    mov     r_battery_v,r_tmp
    ldi     r_tmp,A_BUTTON                 ; read buttons next
    rjmp    adc_result_done
it_is_button:
    mov     r_buttons,r_tmp
    ldi     r_tmp,A_RSSI                  ; go back and read rssi threshold next
    ldi     r_adc_count,0xFF
adc_result_done:
    sts     ADC0_MUXPOS,r_tmp
    ldi     r_tmp,0x01
    sts     ADC0_COMMAND,r_tmp          ; start a new conversion
    inc     r_adc_count
    breq    main_logic_loop
    rjmp    adc_result_exit

;; main logic loop - run every ~1ms
main_logic_loop:
    inc     r_tick_count
    mov     r_tmp,r_tick_count
    andi    r_tmp,0x80
    breq    tag_battery_ok

    mov     r_tmp,r_tag_voltage
    cpi     r_tmp,0xBE
    brcc    tag_battery_ok
    battery_led_on
    rjmp    battery_test_done
tag_battery_ok:
    mov     r_tmp,r_battery_v
    cpi     r_tmp,0xC0
    brcc    battery_ok
    battery_led_on
    rjmp    battery_test_done
battery_ok:
    battery_led_off
battery_test_done:

    or      r_cat_search_timer,r_cat_search_timer
    breq    is_motor_on
    dec     r_cat_search_timer
    brne    is_motor_on
    cat_led_off                 ;; turn off rx, cat was not found
    sts     USART0_CTRLB,r_zero

is_motor_on:
    SKIP_IF_LID_NOT_OPENING
    rjmp    test_lid_open
    SKIP_IF_LID_NOT_CLOSING
    rjmp    test_lid_closed

    mov     r_tmp,r_battery_v           ; get difference between battery v and motor v
    sub     r_tmp,r_motor_v
    mov     r_motor_off_diff,r_tmp

    or      r_mode,r_mode               ; if r_mode is zero, then open the lid
    brne    test_button_press

    CAT_CLOSES_LID
    open_lid

    rjmp    test_button_press

test_lid_open:
    rcall   test_motor_current
    brne    test_lid_open_reti
    lid_now_open
test_lid_open_reti:
    reti

test_lid_closed:
    rcall   test_motor_current
    brne    test_lid_closed_reti
    lid_now_closed
test_lid_closed_reti:
    reti

test_motor_current:
    mov     r_tmp,r_battery_v               ;; if there is a significant difference between battery
    sub     r_tmp,r_motor_v                 ;; voltage and motor voltage, then the motor has hit the
    mov     r_motor_on_diff,r_tmp           ;; the end (either closed or open)

    sub     r_tmp,r_motor_off_diff
    cpi     r_tmp,6
    brsh    test_for_shut_motor_off
    mov     r_motor_count,r_zero            ;; reset motor current count
    clz
    ret

test_for_shut_motor_off:
    inc     r_motor_count
    cpi     r_motor_count,50                ;; over current should  last for 50ms
    brne    test_motor_ret
    mov     r_motor_count,r_zero
test_motor_ret:
    ret


test_button_press:
    mov     r_tmp,r_buttons
    cpi     r_tmp,0xFF
    breq    no_buttons_pressed
    SKIP_IF_BUTTON_NOT_PRESSED
    rjmp    test_cat_nearby
    BUTTON_PRESSED
    SKIP_IF_CAT_CLOSES_LID
    rjmp    cat_will_close_lid
    CAT_OPENS_LID
    close_lid
    reti
cat_will_close_lid:
    CAT_CLOSES_LID
    open_lid
    reti

no_buttons_pressed:
    BUTTON_NOT_PRESSED

test_cat_nearby:
    SKIP_IF_CAT_DETECTED
    rjmp    test_for_reopen_lid
    CAT_NOT_DETECTED                ;; toggle it off
    SKIP_IF_CAT_CLOSES_LID
    rjmp    cat_detected_open_lid
cat_detected_close_lid:
    SKIP_IF_LID_OPEN
    reti
    close_lid
    reti

cat_detected_open_lid:
    SKIP_IF_LID_CLOSED
    reti
    open_lid
    reti

test_for_reopen_lid:
    mov     r_tmp,r_cat_detected_timer_l
    or      r_tmp,r_cat_detected_timer_h
    breq    adc_result_exit
    sbiw    r_cat_detected_timer_l,0x01
    brne    adc_result_exit

    SKIP_IF_CAT_CLOSES_LID
    rjmp    reclose_the_lid
    open_lid
    reti

reclose_the_lid:
    close_lid

adc_result_exit:
    reti


