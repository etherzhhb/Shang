; RUN: %llc-with-config %s | FileCheck %s

define i32 @f(i32 %a, i32 %b) nounwind readnone {
entry:
  %0 = lshr i32 %b, 1                             ; <i32> [#uses=1]
  %1 = and i32 %0, 3                              ; <i32> [#uses=1]
  %2 = add nsw i32 %1, %a                         ; <i32> [#uses=1]
  ret i32 %2
}

; CHECK: fin <= 1'b1;
