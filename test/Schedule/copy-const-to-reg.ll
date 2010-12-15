; RUN: %llc-with-config %s

define void @cfg_loop() nounwind noinline {
bb.nph:
  br label %bb

bb:                                               ; preds = %bb, %bb.nph
  %i.04 = phi i32 [ 0, %bb.nph ], [ %0, %bb ]
  %0 = add nsw i32 %i.04, 1
  %exitcond = icmp eq i32 %0, 16
  br i1 %exitcond, label %bb2, label %bb

bb2:                                              ; preds = %bb
  ret void
}
