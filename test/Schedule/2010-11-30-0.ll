; RUN: %llc-with-config %s -o - | FileCheck %s

define i32 @f(i32 %a, i32 %b) nounwind readnone {
entry:
  %x = sub nsw i32 %b, %a                         ; <i32> [#uses=1]
  ret i32 %x
}

; CHECK:   assign entryBB0_wire1 = ~ reg2[31:0];
; CHECK:   assign {entryBB0_wire3, entryBB0_wire2} = reg3[31:0] + entryBB0_wire1 + 1'hf;
