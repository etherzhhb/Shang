; RUN: %llc-with-config %s -o -  | FileCheck %s
; XFAIL: *

define void @f(i32* nocapture %a) nounwind {
entry:
  %0 = load i32* %a, align 4                      ; <i32> [#uses=1]
  %1 = add nsw i32 %0, -2                         ; <i32> [#uses=1]
  %2 = getelementptr inbounds i32* %a, i64 1      ; <i32*> [#uses=1]
  store i32 %1, i32* %2, align 4
  ret void
}

; CHECK:  mem0in + 64'h
; CHECK:  mem0out <= 32'h0;
