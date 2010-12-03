; RUN: %llc-with-config %s -o - 

define void @loop(i32* nocapture %a, i32 %n) nounwind {
entry:
  %0 = icmp sgt i32 %n, 0                         ; <i1> [#uses=1]
  br i1 %0, label %bb.nph, label %return

bb.nph:                                           ; preds = %entry
  %tmp = sext i32 %n to i64                       ; <i64> [#uses=1]
  br label %bb

bb:                                               ; preds = %bb, %bb.nph
  %i.03 = phi i64 [ 0, %bb.nph ], [ %3, %bb ]     ; <i64> [#uses=3]
  %scevgep = getelementptr i32* %a, i64 %i.03     ; <i32*> [#uses=1]
  %tmp4 = add i64 %i.03, 4                        ; <i64> [#uses=1]
  %scevgep5 = getelementptr i32* %a, i64 %tmp4    ; <i32*> [#uses=1]
  %1 = load i32* %scevgep, align 4                ; <i32> [#uses=1]
  %2 = add nsw i32 %1, -2                         ; <i32> [#uses=1]
  store i32 %2, i32* %scevgep5, align 4
  %3 = add nsw i64 %i.03, 1                       ; <i64> [#uses=2]
  %exitcond = icmp eq i64 %3, %tmp                ; <i1> [#uses=1]
  br i1 %exitcond, label %return, label %bb

return:                                           ; preds = %bb, %entry
  ret void
}

