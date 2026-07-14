; 8250 UART Serial I/O Driver for 8088
; Implements serial communication functions for COM1 (port 0x3F8)
;
; Baud Rate Configuration (1.8432 MHz crystal):
;   115200 baud: divisor = 1
;    57600 baud: divisor = 2
;    38400 baud: divisor = 3
;    19200 baud: divisor = 6
;     9600 baud: divisor = 12
;     4800 baud: divisor = 24
;     2400 baud: divisor = 48
;
; Change BAUD_DIVISOR below to set desired baud rate

	;.model	small
	;.8086

	PUBLIC	_port_uart_base
	PUBLIC	_port_slip_last_reason
	PUBLIC	_port_slip_last_lsr
	PUBLIC	_port_rx_isr
	PUBLIC	_port_rx_isr_count
	PUBLIC	_port_rx_isr_bytes
	PUBLIC	_port_flush_rx
	PUBLIC	_port_wait_tx_empty

	.data

; Global variable to store UART base address
_port_uart_base	DW	3F8h		; Default to COM1
_port_slip_last_reason DB 0
_port_slip_last_lsr DB 0
_port_rx_head DB 0
_port_rx_tail DB 0
_port_rx_buf DB 16 dup(?)
_port_rx_isr_count DW 0
_port_rx_isr_bytes DW 0

	.code

	; Baud rate divisor - change this to set baud rate
BAUD_DIVISOR	EQU	1		; 115200 baud

	; 8250 UART Register Offsets (relative to base)
UART_RBR_OFF	EQU	0		; Receiver Buffer Register (read)
UART_THR_OFF	EQU	0		; Transmitter Holding Register (write)
UART_IER_OFF	EQU	1		; Interrupt Enable Register
UART_IIR_OFF	EQU	2		; Interrupt Identification Register
UART_FCR_OFF	EQU	2		; FIFO Control Register (write)
UART_LCR_OFF	EQU	3		; Line Control Register
UART_MCR_OFF	EQU	4		; Modem Control Register
UART_LSR_OFF	EQU	5		; Line Status Register
UART_MSR_OFF	EQU	6		; Modem Status Register
UART_DLL_OFF	EQU	0		; Divisor Latch Low (when DLAB=1)
UART_DLH_OFF	EQU	1		; Divisor Latch High (when DLAB=1)

	; Line Status Register bits
LSR_DR		EQU	01h		; Data Ready
LSR_OE		EQU	02h		; Overrun Error
LSR_PE		EQU	04h		; Parity Error
LSR_FE		EQU	08h		; Framing Error
LSR_BI		EQU	10h		; Break Interrupt
LSR_THRE	EQU	20h		; Transmitter Holding Register Empty
LSR_TEMT	EQU	40h		; Transmitter Empty

	; Line Control Register bits
LCR_DLAB	EQU	80h		; Divisor Latch Access Bit
LCR_8N1		EQU	03h		; 8 data bits, no parity, 1 stop bit

	; Modem Control Register bits
MCR_DTR		EQU	01h		; Data Terminal Ready
MCR_RTS		EQU	02h		; Request To Send
MCR_OUT2	EQU	08h		; OUT2 (enables interrupts on PC)

	; FIFO Control Register bits (16550 and newer)
FCR_ENABLE	EQU	01h		; Enable FIFOs
FCR_CLEAR_RX	EQU	02h		; Clear receive FIFO
FCR_CLEAR_TX	EQU	04h		; Clear transmit FIFO
FCR_TRIGGER_1	EQU	00h		; Receive trigger level: 1 byte

	; BIOS Data Area
BIOS_DATA_SEG	EQU	40h
BIOS_TICK_OFFSET EQU	6Ch

	; SLIP Protocol Constants
SLIP_END	EQU	0C0h
SLIP_ESC	EQU	0DBh
SLIP_ESC_END	EQU	0DCh
SLIP_ESC_ESC	EQU	0DDh

PORT_SLIP_REASON_NONE	EQU	0
PORT_SLIP_REASON_TIMEOUT EQU	1
PORT_SLIP_REASON_BUFFER_FULL EQU 2
PORT_SLIP_REASON_LINE_STATUS EQU 3

	.code

;-----------------------------------------------------------------------------
; UART receive interrupt handler.
; Drains all currently available RX bytes into a small resident ring buffer.
;-----------------------------------------------------------------------------
_port_rx_isr PROC NEAR
	push	ax
	push	bx
	push	dx
	push	ds

	push	cs
	pop	ds

	inc	word ptr _port_rx_isr_count

	mov	dx, _port_uart_base
	add	dx, UART_LSR_OFF

rx_isr_loop:
	in	al, dx
	mov	ah, al
	and	al, LSR_OE OR LSR_PE OR LSR_FE OR LSR_BI
	jz	rx_isr_no_lsr_error
	or	_port_slip_last_lsr, al
	mov	_port_slip_last_reason, PORT_SLIP_REASON_LINE_STATUS

rx_isr_no_lsr_error:
	test	ah, LSR_DR
	jz	rx_isr_done

	mov	dx, _port_uart_base
	add	dx, UART_RBR_OFF
	in	al, dx
	inc	word ptr _port_rx_isr_bytes

	mov	ah, _port_rx_head
	inc	ah
	and	ah, 0Fh
	cmp	ah, _port_rx_tail
	je	rx_isr_drop

	xor	bx, bx
	mov	bl, _port_rx_head
	mov	_port_rx_buf[bx], al
	mov	_port_rx_head, ah
	jmp	rx_isr_next

rx_isr_drop:

rx_isr_next:
	mov	dx, _port_uart_base
	add	dx, UART_LSR_OFF
	jmp	rx_isr_loop

rx_isr_done:
	mov	al, 20h
	out	20h, al

	pop	ds
	pop	dx
	pop	bx
	pop	ax
	iret
_port_rx_isr ENDP

	include port_init.asm
	include port_flush_rx.asm
	include port_wait_tx_empty.asm
	include port_getbuf_slip_dual.asm
	include port_putc.asm
	include port_putbuf_slip.asm

	END
