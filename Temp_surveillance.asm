.include "m328pdef.inc"

.def temp = r16
;;Delayregister
.def delayreg = r17
.def delayreg2 = r18
.def delayreg3 = r19
.def temp2 = r20
;;temperaturhantering
.def maxtemp = r21
.def mintemp = r22
.def Tempvlu = r24
.def TempMSB = r5
.def TempLSB = r6
;;Siffror
.def c100 = r7
.def c10 = r8
.def c1 = r9
;;Frekvensberäkning
.def freq_counter = r10
.def quarters_counter = r15
.Equ Thirtyhz = 244
;;EEPROM
.Equ EEmaxtemp = 10
.Equ EEmintemp = 10
;;Uart
.EQU B4800 = 12





.cseg
.org 0
	rjmp reset
.org OC2Aaddr
	rjmp screen_update
.org ICP1addr
	rjmp freq_counting

reset:
	rcall init_portar
	rcall init_skarm
	rcall init_timer0
	rcall init_timer2
	rcall init_timer1
	rcall init_uart
	rcall temperatur_string
	rcall read_temp
	rcall decode_temp
	rcall send_numbers
	rcall maxtemp_string
	rcall mintemp_string
	rcall Varvtal_string
	rcall Celsius_tecken
	rcall mineeprom_read
	rcall write_mintemp
	rcall maxeeprom_read
	rcall write_maxtemp
	rcall uart_cseg2dseg
	rcall rps_string
	sei
	rjmp main_loop

main_loop:
	rcall read_temp
	rcall decode_temp
	rcall send_numbers
	rcall check_temp
	rcall check_buttons
	rjmp main_loop

;;EEPROM READ/WRITE;;;
;;;;;;;;;;;;;;;;;;;;;;
;;Skrivning till EEPROM med data som ligger i temp och adress som ligger i pekare Y
write_eeprom:
	out EEDR,temp
	out EEARL,YL
	out EEARH,YH
	ldi temp, (1<<EEMPE)|(0<<EEPE)
	out EECR,temp
	ori temp, (1<<EEPE)
	out EECR,temp
EEwrite_wait:
	sbic EECR, EEPE
	rjmp EEwrite_wait
	ret
;;Läser in sparat värde på maxtemp från EEprom och lägger i maxtemp-register 

MaxEEPROM_read:
; Wait for completion of previous write
	sbic EECR,EEPE
	rjmp maxEEPROM_read
; Set up address (YH:YL) in address register
	ldi YH,high(smaxtemp)
	ldi YL,low(smaxtemp)
	out EEARH, YH
	out EEARL, YL
; Start eeprom read by writing EERE
	sbi EECR,EERE
; Read data from Data Register
	in maxtemp,EEDR
	ret

;;Läser in sparat värde på mintemp från EEprom och lägger i mintemp-register 
MinEEPROM_read:
	; Wait for completion of previous write
	sbic EECR,EEPE
	rjmp MinEEPROM_read
	; Set up address (YH:YL) in address register
	ldi YH,high(smintemp)
	ldi YL,low(smintemp)
	out EEARH, YH
	out EEARL, YL
	; Start eeprom read by writing EERE
	sbi EECR,EERE
	; Read data from Data Register
	in mintemp,EEDR
	ret
	
;;HANTERING AV TEMP/MAX/MIN;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;Kollar tempvärde som läses in mot det satta max/minvärdet och kallar på subrutiner för att slå av/på larm beroende på resultat
Check_temp:
	cp tempvlu, maxtemp
	brsh high_on
	cp mintemp, tempvlu 
	brsh low_on
	sbic PORTB,7
	rjmp turn_off
	rjmp check_done
high_on:
	rcall alarm_high_on
	rjmp check_done
low_on:
	rcall alarm_low_on
	rjmp check_done
turn_off:
	rcall alarm_off
check_done:
	ret

;Kollar vilken knapp som är nedtryckt och debouncar, addar eller subbar sedan från max/mintemp
check_buttons:
	rcall btn_dbnc
	in temp,PIND
	sbrs temp,0
	rcall add_mintemp
	sbrs temp,2
	rcall sub_mintemp
	sbrs temp,3
	rcall add_maxtemp
	sbrs temp,4
	rcall sub_maxtemp
	ret
;;debounce på knappar
btn_dbnc:
	push temp
	in temp,PIND
	rcall delay
	rcall delay
	rcall delay
	in temp2,PIND
	cp temp,temp2
	breq done_dbnc
	ldi temp,0b00001111
done_dbnc:
	pop temp
	ret

;;Ökar maxtemp och skriver till SRAMbuffert samt eeprom med nytt värde
add_maxtemp:
	cpi maxtemp,70
	breq Maxadd_done
	push temp
	inc maxtemp
	rcall write_maxtemp
	ldi YH,high(Smaxtemp)
	ldi YL,low(Smaxtemp)
	mov temp,maxtemp
	rcall write_eeprom
	pop temp
Maxadd_done:
	ret

;;Minskar maxtemp och skriver till SRAMbuffert samt eeprom med nytt värde
sub_maxtemp:
	cp maxtemp,mintemp
	breq Maxsub_done
	push temp
	dec maxtemp
	rcall write_maxtemp
	ldi YH,high(Smaxtemp)
	ldi YL,low(Smaxtemp)
	mov temp,maxtemp
	rcall write_eeprom
	pop temp
Maxsub_done:
	ret
;Skickar värde i maxtemp till Srambuffert efter att ha omvandlat värdet till decimala siffror
write_maxtemp:
	push temp
	mov temp,maxtemp
	ldi XH,high(SRAM_buffert+21)
	ldi XL,low(SRAM_buffert+21)
	rcall number_temp
	rcall send_numbers
	pop temp
	ret
;;Ökar mintemp och skriver till SRAMbuffert samt eeprom med nytt värde
add_mintemp:
	cp mintemp,maxtemp
	breq minadd_done
	push temp
	inc mintemp
	rcall write_mintemp
	ldi YH,high(Smintemp)
	ldi YL,low(Smintemp)
	mov temp,mintemp
	rcall write_eeprom
	pop temp
	Minadd_done:
	ret
;;Minskar mintemp och skriver till SRAMbuffert samt eeprom med nytt värde, kontrollerar att mintemp inte blir mindre än 10
sub_mintemp:
	cpi mintemp,10
	breq minsub_done
	push temp
	dec mintemp
	rcall write_mintemp
	ldi YH,high(Smintemp)
	ldi YL,low(Smintemp)
	mov temp,mintemp
	rcall write_eeprom
	pop temp
Minsub_done:
	ret
;Skickar värde i mintemp till Srambuffert efter att ha omvandlat värdet till decimala siffror
write_mintemp:
	push temp
	mov temp,mintemp
	ldi XH,high(SRAM_buffert+29)
	ldi XL,low(SRAM_buffert+29)
	rcall number_temp
	rcall send_numbers
	pop temp
	ret

;Skiftar ut MSB från tempmsb och skiftar in msb från templsb och flyttar till tempvlu. Skickar sedan via temp detta värde till number_temp för att omvandla till decimalt värde
decode_temp:
	lsl templsb
	rol tempmsb
	mov Tempvlu,Tempmsb
	mov Temp,tempvlu
	ldi XH,high(SRAM_buffert + 11)
	ldi XL,low(SRAM_buffert + 11)
	rcall number_temp
	ret
;Konverterar innehållet som skickas in via temp till 3 decimala siffror, hundratal,tiotal och ental. kallar conv ascii för att konvertera decimaltalen till ascii 
number_temp:
	clr c100
	clr c10
	clr c1
check_hundred:
	cpi Temp,100
	brsh add_hundred
check_ten:
	cpi temp,10
	brsh add_ten
check_one:
	cpi Temp,1
	brsh add_one
number_done:
	rcall conv_ascii
	ret
;Inkrementerar hundratal/tiotal/entalsregister beroende på värdet som ligger i temp
add_hundred:
	inc c100
	subi Temp,100
	rjmp check_hundred
add_ten:
	inc c10
	subi Temp,10
	rjmp check_ten
add_one:
	inc c1
	subi Temp,1
	rjmp check_one

;;Konverterar innehållet i ental tiotal och hundratal till asciikoder
conv_ascii:
	mov temp,c100
	subi temp,-48
	mov c100,temp
	mov temp,c10
	subi temp,-48
	mov c10,temp
	mov temp,c1
	subi temp,-48
	mov c1,temp
	ret


;;;SKRIVNINGAR TILL SRAMBUFFERT;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;Skickar siffror till SRAM-buffert, position skickas med via x-pekare och ascii-kod via ten och one
send_numbers:
;laddar ascii-siffra för hundratal till sram, behövdes ej så har kommenterat bort
;;send_hundred:
	;;mov temp,c100
	;;cpi temp,48
	;;breq send_ten
	;;st X+,c100
;laddar ascii-siffra för tiotal till sram
send_ten:
	st X+,c10
;laddar ascii-siffra för ental till sram
send_one:
	st X,c1
	ret

;Laddar texten Temperatur till sram buffert via load_str
Temperatur_string:
	ldi XH,high(SRAM_buffert)
	ldi XL,low(SRAM_buffert)
	push ZH
	push ZL
	ldi ZH,high(Temp_text * 2)
	ldi ZL,low(Temp_text * 2)
	rcall load_str
	pop ZL
	pop ZH
	ret
;Laddar texten max : till sram-buffert
Maxtemp_string:
	ldi XH,high(SRAM_buffert+16)
	ldi XL,low(SRAM_buffert+16)
	push ZH
	push ZL
	ldi ZH,high(Temp_text2 * 2)
	ldi ZL,low(Temp_text2 * 2)
	rcall load_str
	pop ZL
	pop ZH
	ret
;Laddar texten min : till sram-buffert
Mintemp_string:
	ldi XH,high(SRAM_buffert+24)
	ldi XL,low(SRAM_buffert+24)
	push ZH
	push ZL
	ldi ZH,high(Temp_text3 * 2)
	ldi ZL,low(Temp_text3 * 2)
	rcall load_str
	pop ZL
	pop ZH
	ret
;laddar texten Varvtal: till SRAM-buffert
Varvtal_string:
	ldi XH,high(SRAM_buffert+32)
	ldi XL,low(SRAM_buffert+32)
	push ZH
	push ZL
	ldi ZH,high(Varv_text * 2)
	ldi ZL,low(Varv_text * 2)
	rcall load_str
	pop ZL
	pop ZH
	ret
;;ASCII för celsiustecken till srambuffert
Celsius_tecken:
	ldi XH,high(SRAM_buffert+13)
	ldi XL,low(SRAM_buffert+13)
	push ZH
	push ZL
	ldi ZH,high(Gradtecken * 2)
	ldi ZL,low(Gradtecken * 2)
	rcall load_str
	pop ZL
	pop ZH
	ret
;;Sträng för RP/S laddas till srambuffert
Rps_string:
	ldi XH,high(SRAM_buffert+44)
	ldi XL,low(SRAM_buffert+44)
	push ZH
	push ZL
	ldi ZH,high(Rps * 2)
	ldi ZL,low(Rps * 2)
	rcall load_str
	pop ZL
	pop ZH
	ret

;; UART-subrutiner;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;

;;Laddar alarmtexter till sram 
;;Laddar text för varning hög temp/låg temp
Uart_cseg2dseg:
	ldi XH,high(high_alarm)
	ldi XL,low(high_alarm)
	ldi ZH,high(Larm_text * 2)
	ldi ZL,low(Larm_text * 2)
	rcall load_str
	ldi XH,high(low_alarm)
	ldi XL,low(low_alarm)
	ldi ZH,high(LowLarm_text * 2)
	ldi ZL,low(LowLarm_text * 2)
	rcall load_str
	ret

;;Laddar X-pekare med SRAM-adress till larmsträngar och kallar på uart-subrutin för att skicka den till pc-terminal
Uart_highalarm:
	ldi XH,high(high_alarm)
	ldi XL,low(high_alarm)
	rcall Uartsend_char
	ret
Uart_lowalarm:
	ldi XH,high(low_alarm)
	ldi XL,low(low_alarm)
	rcall Uartsend_char
	ret
;Laddar från SRAM beroende på vad som skickats in via pekare X, kollar efter 0 för att sluta skicka via UART
Uartsend_char:
	ld temp,x+
	cpi temp,0
	breq Udone_send
	sts UDR0,temp
send_wait:
	rcall delay
	rjmp Uartsend_char
Udone_send:
	ret


;Ladda text/siffror från programminnet till SRAM_buffert eller annan minnesposition beroende på vad som skickas in via pekare Z och X
load_str:
	lpm temp,z+
	cpi temp,0
	breq done_str
	st X+,temp
	rjmp load_str
done_str:
	ret
;;LÄS/SKRIV SPI;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;

;Läser in 2 byte från tempsensor och spara undan dessa i TEMPMSB icg TEMPLSB
read_temp:
;cleara interruptflagga och sätt SS för temp till låg, ladda temp2 för att ta emot 2 bytes
	cli
	cbi PORTB,7
	ldi temp2,2
spi_txrx:
	out SPDR, temp ; start tx/rx cycle by writing to SPDR
	dec temp2
wait_txrx:
	in r16, SPSR ; read status register
	sbrs r16, SPIF ; skip if transfer complete
	rjmp wait_txrx
	sbrc temp2,0
	in TempMSB, SPDR
	sbrs temp2,0
	in TempLSB, SPDR
done_transfer:
	cpi temp2,0
	brne spi_txrx
	sbi PORTB,7
	sei
	ret
;;SKICKAR SRAMBUFFERT TILL SKÄRM VIA SPI

;Laddar alla 48 värden från SRAM_buffert och skickar till skärm via SPI.  
send_check:
	push temp
	in temp,SREG
	push temp
	push temp2
	ldi temp2,48
	ldi XH,high(SRAM_buffert)
	ldi XL,low(SRAM_buffert)
send_rdy:
	ld r4,X+
	out SPDR, r4 ; start tx/rx cycle by writing to SPDR
	dec temp2
check:
	in temp, SPSR ; read status register
	sbrs temp, SPIF ; skip if transfer complete
	rjmp check
done_send:
	cpi temp2,0
	brne send_rdy
	pop temp2
	pop temp
	out SREG,temp
	pop temp
	ret
;;LARMSUBRUTINER;;;;
;;;;;;;;;;;;;;;;;;;;

;;Enablar ljudalarm via att ettställa bitar i TCR0A för att enabla ctc och toggle OC0A pin på compare match
;;Enablar lampa för att indikera hög temp, ettställer PORTD 7
;;Laddar tecken för att indikera hög temp-larm till sram-buffert
;;Skickar via Uart_highalarm larmmeddelande till pcterminal
alarm_high_on:
	cbi PORTD,5
	sbi PORTD,7
	ldi temp, (1<<WGM01)| (1<<COM0A0)
	out TCCR0A, temp
	ldi temp, 155
	out OCR0A, temp
	ldi XH,high(SRAM_buffert+23)
	ldi XL,low(SRAM_buffert+23)
	ldi temp,0b00100001
	st X,temp
	rcall Uart_highalarm
	ret

;;Enablar ljudalarm via att ettställa bitar i TCR0A för att enabla ctc och toggle OC0A pin på compare match
;;Enablar lampa för att indikera hög temp, ettställer PORTD 5
;;Laddar tecken för att indikera låg temp-larm till sram-buffert
;;Skickar via Uart_lowalarm larmmeddelande till pcterminal
alarm_low_on:
	sbi PORTB,6
	ldi temp, (1<<WGM01)| (1<<COM0A0)
	out TCCR0A, temp
	ldi temp, 75
	out OCR0A, temp
	ldi XH,high(SRAM_buffert+31)
	ldi XL,low(SRAM_buffert+31)
	ldi temp,0b00100001
	st X,temp
	rcall uart_lowalarm
	ret
;;Disablar ljudalarm via att nollställa bitarna TCR0A som enablar ctc och toggle OC0A pin på compare match
;;Disablar lampor för att indikera hög/låg temp, nollställer PORTD 5 och PORTD 7
;;Skickar via Uart_lowalarm larmmeddelande till pcterminal
alarm_off:
	sbi PORTD,5
	cbi PORTB,6
	cbi PORTD,7
	ldi temp, (0<<WGM01)| (0<<COM0A0)
	out TCCR0A, temp
	ldi XH,high(SRAM_buffert+23)
	ldi XL,low(SRAM_buffert+23)
	ldi temp,0b00000000
	st X,temp
	ldi XH,high(SRAM_buffert+31)
	ldi XL,low(SRAM_buffert+31)
	ldi temp,0b00000000
	st X,temp
	ret

;INTERRUPTRUTINER
;;
;;

;Interruptrutin skickar innehållet i SRAM_buffert till skärmen via SPI
screen_update:
	push temp
	in temp,SREG
	push temp
	push XL
	push XH
	sbi PORTB,7
	cbi PORTB,2
	rcall send_check
	sbi PORTB,2
	;;Håller koll på tid för att beräkna frekvens, har interrupt med ca 62 millisekunders intervall och behöver köras 4 ggr för att 
	;; calc_freq ska kallas vilket gör att frekvensen blir korrekt då den som läses in från fläkten är 4ggr det aktuella varvtalet enligt datablad 
	inc quarters_counter
	sbrc quarters_counter,2
	;;
	rcall calc_freq
	pop XH
	pop XL
	pop temp
	out SREG,temp
	pop temp
	reti
;;Varje gång vi får in en input capture så inkrementeras freq_counter
freq_counting:
	inc freq_counter
	reti

;;;;;;;;;;;;;VARVTALSBERÄKNING;;;;;;;;;;;;;;;;
;;Skickar värdet i freq counter till skärmen och rensar de register som hållt koll på tid och antal inputs från fläkten
calc_freq:
	mov temp,freq_counter
	rcall number_temp
	ldi XH,high(sram_buffert+40)
	ldi XL,low(sram_buffert+40)
	rcall send_numbers
	clr freq_counter
	clr quarters_counter
	ret

	

;;INIT SEKVENSER;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;Initierar 8-bitars timern som används för att skapa pulsvåg för larmsignaler, ;ldi temp, (1<<WGM01)| (1<<COM0A0) enablas när larm för hög/låg temp kommer
init_timer0:
	;init rakneregister
	;init ctc
	;ldi temp, (1<<WGM01)| (1<<COM0A0)
	;out TCCR0A, temp
	ldi temp, (1<<CS01) | (1<<CS00)
	out TCCR0B, temp
	;init interrupt compare
	;ldi temp, (1<<OCIE0A)
	;sts TIMSK0, temp
	;counter top_varde
	ldi temp, 155
	out OCR0A, temp
	ret
;Initierar timern i mode CTC med interrupt. Används för att uppdatera skärminnehållet via sram-buffert
;
init_timer2:
;	;init timer skarmuppdat	
;	;OverflowA flagga
	ldi temp,(1<<OCIE2A)
	sts TIMSK2,temp
;	;Enabla CTC
	ldi temp,(1<<CS22)|(1<<WGM21)|(1<<CS21);|(1<<CS20)
	sts TCCR2B,temp
	ldi temp,(1<<WGM21)
	sts TCCR2A,temp
	rcall delay
;	;Ladda med compare
	ldi temp,thirtyhz
	sts OCR2A,temp
	ret
;;Init timer 1 för input capture
init_timer1:
	;init timer skarmuppdat	
	;OverflowA flagga
	ldi temp,(1<<ICIE1)
	sts TIMSK1,temp
	;Ej ctc normal mode noisce cancel och pos edge
	ldi temp,(1<<CS12);|(1<<ICNC1);|(1<<ICES1)
	sts TCCR1B,temp
	rcall delay
	ret


;Initierar skärmen utan cursor och med 3 rader
init_skarm:
	cbi PORTB,2
	cbi PORTB,1
	rcall delay
	ldi temp,0b00111001
	out SPDR,temp
	rcall delay
	ldi temp,0x1D
	out SPDR,temp
	rcall delay
	ldi temp,0x48
	out SPDR,temp
	rcall delay
	ldi temp,0x6C
	out SPDR,temp
	rcall delay
	ldi temp,0x7C
	out SPDR,temp
	rcall delay
	ldi temp,0x38
	out SPDR,temp
	rcall delay
	ldi temp,0x0C
	out SPDR,temp
	rcall delay
	ldi temp,0x01
	out SPDR,temp
	rcall delay
	ldi temp,0x06
	out SPDR,temp
	rcall delay
	sbi PORTB,2
	sbi PORTB,1
	ret
;;
init_portar:
	ldi temp,0b11101110
	out DDRB,temp
	sbi PORTB,0
	sbi PORTB,7
	sbi PORTB,2
	ldi temp,(1<<MSTR)|(1<<SPE)|(1<<SPR0)
	out SPCR,temp
	ldi temp, 0b11100010
	out DDRD,temp
	ldi temp, 0b00111101
	out PORTD,temp
	ret

init_uart:
	ldi temp,B4800
	sts UBRR0L,temp
	clr temp
	sts UBRR0H,temp
	ldi temp,(1<<TXEN0)
	sts UCSR0B,temp
	ret

;;Delaysubrutin
delay:
	push delayreg
	push delayreg2
	push delayreg3
	push temp
	in temp,SREG
	push temp
	cpi temp,0
	ldi delayreg,30
loop:
	ldi delayreg3,17
	ldi delayreg2,17
	dec delayreg
	tst delayreg
	breq done
loop2:
	ldi delayreg3,17
	dec delayreg2
	tst delayreg2
	breq loop
loop3:
	dec delayreg3
	tst delayreg3
	breq loop2
	rjmp loop3
done:
	pop temp
	out SREG,r16
	pop temp
	pop delayreg3
	pop delayreg2
	pop delayreg
	ret






.cseg
Temp_text:
.db "Temperatur",0,0
Temp_text2:
.db "Max :",0
Temp_text3:
.db "Min :",0
Varv_text:
.db "Varvtal :",0
Larm_text:
.db " RED ALERT! TEMP HIGH! ",0
LowLarm_text:
.db " SUPER ALERT! TEMP LOW! ",0
Gradtecken:
.db 0b11011111,0b01000011,0
Rps:
.db "RP/S",0

.dseg
SRAM_buffert:
.byte 48
High_alarm:
.byte 28
low_alarm:
.byte 28

.eseg
Smaxtemp:
.byte 2
Smintemp:
.byte 2