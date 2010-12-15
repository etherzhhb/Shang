; RUN: %llc-with-config %s

define void @cfg_loop() nounwind noinline {
bb.nph:
  br label %bb

bb:                                               ; preds = %bb, %bb.nph
  br i1 undef, label %bb2, label %bb

bb2:                                              ; preds = %bb
  ret void
}

