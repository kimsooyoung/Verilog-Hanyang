;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;; Instruction ;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; a, b, c, d: $a0, $a1, $a2, $a3 values for results: $v0

; a => $4 : 9
; b => $5 : 9
; c => $6 : 9
; d => $7 : 9
; e => $2(v0)
; $ra => $31

; main : 0x00000
; sum  : 0x10000

; int sum (int a, int b, int c, int d)
; {
;     Int e;
;     e = (a-b)+(c-d);
;     return e; 
; }

main:
    jal sum
    lw $1, 4($2)

sum: 
    sub $9, $4, $5
    sub $10, $6, $7
    add $2, $9, $10
    jr $ra