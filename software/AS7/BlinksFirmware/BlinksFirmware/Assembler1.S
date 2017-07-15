
/*
 * Assembler1.S
 *
 * Created: 7/14/2017 9:57:52 PM
 *  Author: passp
 */ 


 16c:	8f 93       	push	r24
 16e:	8d e3       	ldi	r24, 0x3D	; 61
 170:	86 b9       	out	0x06, r24	; 6
 172:	86 b9       	out	0x06, r24	; 6
 174:	8f 91       	pop	r24
 176:	0f 90       	pop	r0
 178:	0f be       	out	0x3f, r0	; 63
 17a:	0f 90       	pop	r0
 17c:	1f 90       	pop	r1
 17e:	18 95       	reti
