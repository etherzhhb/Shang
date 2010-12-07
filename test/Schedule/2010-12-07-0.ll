; RUN: %llc-with-config %s

target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64"
target triple = "x86_64-linux-gnu"

define i32 @op_asr(i32 %a, i32 %b) nounwind readnone {
entry:
  %0 = icmp sgt i32 %a, %b                        ; <i1> [#uses=1]
  %1 = zext i1 %0 to i32                          ; <i32> [#uses=1]
  ret i32 %1
}
