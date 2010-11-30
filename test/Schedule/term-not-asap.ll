; RUN: %llc-with-config %s -o - | FileCheck %s

define i32 @f(i32 %x, i32 %y) nounwind readnone {
entry:
  %0 = icmp sgt i32 %x, 0                         ; <i1> [#uses=1]
  br i1 %0, label %bb, label %bb1

bb:                                               ; preds = %entry
  %1 = add nsw i32 %x, 4                          ; <i32> [#uses=1]
  ret i32 %1

bb1:                                              ; preds = %entry
  %2 = sub nsw i32 %x, %y                         ; <i32> [#uses=1]
  %3 = ashr i32 %2, 3                             ; <i32> [#uses=1]
  ret i32 %3
}

; CHECK:      NextFSMState <= bb1BB
; CHECK-NOT:  cur_entryBB0_enable
; CHECK:      NextFSMState <= bbBB
