/* Inicio minimo do CH592: o chip comeca a executar no endereco 0 */
	.section .init, "ax"
	.global _start
_start:
	la	sp, _estack
	call	main
1:	j	1b
