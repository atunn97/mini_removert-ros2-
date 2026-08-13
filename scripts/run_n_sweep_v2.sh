#!/usr/bin/env bash
# N-sweep hoàn chỉnh (fix v2): map_idx được CHỌN TỰ ĐỘNG theo khoảng cách
# không gian thật (mét), không theo số frame — tránh lỗi parallax/occlusion
# đã phát hiện ở lần chạy trước (xem NOTES_viec4_N_sweep_debug.md).
set -e

# ---- CHỈNH LẠI CÁC PATH NÀY CHO ĐÚNG MÁY BẠN NẾU CẦN ----
BINARY="/home/atun/mini_removert/build/mini_removert/mini_removert"
PCD_DIR="/home/atun/kitti_data/dataset/sequences/04/pcd"
POSES="/home/atun/kitti_data/dataset/poses/04.txt"
CALIB="/home/atun/kitti_data/dataset/sequences/04/calib.txt"
LABEL_DIR="/home/atun/kitti_data/dataset/sequences/04/labels"
EVALUATE_PY="/home/atun/mini_removert/scripts/evaluate.py"
SELECT_MAPS_PY="/home/atun/mini_removert/scripts/select_maps_by_distance.py"
# ---------------------------------------------------------

SCAN_IDX=150
THRESHOLD=1.0     # doi 0.5 -> 1.0 ngay 2026-08-13 sau khi sweep: F1 TB 0.629 -> 0.691
                  # tren 5 scan, tang 5/5. Xem HANDOFF_2026-08-13 muc 12.
MAX_DISTANCE_M=4.0     # ban kinh nho hon 8.0 cu - xac nhan qua maxdist-sweep tren 5 scan
                        # (50/100/150/200/250): F1 tang 5/5, xem results_maxdist_sweep/
RESULTS_DIR="/home/atun/mini_removert/results_n_sweep_v2"

mkdir -p "$RESULTS_DIR"
cd "$RESULTS_DIR"

LABEL_FILE=$(printf "%s/%06d.label" "$LABEL_DIR" "$SCAN_IDX")

SUMMARY_FILE="$RESULTS_DIR/summary.txt"
> "$SUMMARY_FILE"
printf "%-4s %-10s %-10s %-10s %-30s\n" "N" "Precision" "Recall" "F1" "map_idx" >> "$SUMMARY_FILE"

for N in 2 4 6 8 10; do
    echo ""
    echo "===================================================="
    echo "=== N=$N | scan_idx=$SCAN_IDX | max_distance_m=$MAX_DISTANCE_M ==="
    echo "===================================================="

    SELECT_LOG="$RESULTS_DIR/select_N${N}.log"
    python3 "$SELECT_MAPS_PY" "$POSES" "$CALIB" "$SCAN_IDX" "$N" "$MAX_DISTANCE_M" | tee "$SELECT_LOG"

    MAP_IDX=$(tail -1 "$SELECT_LOG")
    echo ""
    echo ">>> map_idx duoc chon: $MAP_IDX"

    RUN_LOG="$RESULTS_DIR/run_N${N}.log"
    "$BINARY" "$PCD_DIR" "$POSES" "$CALIB" "$SCAN_IDX" "$THRESHOLD" $MAP_IDX | tee "$RUN_LOG"

    DEFAULT_OUT="$RESULTS_DIR/dynamic_indices_scan${SCAN_IDX}.txt"
    PER_N_OUT="$RESULTS_DIR/dynamic_indices_scan${SCAN_IDX}_N${N}.txt"
    mv "$DEFAULT_OUT" "$PER_N_OUT"

    echo ""
    echo "--- Evaluate N=$N ---"
    EVAL_LOG="$RESULTS_DIR/eval_N${N}.log"
    python3 "$EVALUATE_PY" "$LABEL_FILE" "$PER_N_OUT" | tee "$EVAL_LOG"

    PRECISION=$(grep "Precision:" "$EVAL_LOG" | awk '{print $2}')
    RECALL=$(grep "Recall:" "$EVAL_LOG" | awk '{print $2}')
    F1=$(grep "F1 score:" "$EVAL_LOG" | awk '{print $3}')

    printf "%-4s %-10s %-10s %-10s %-30s\n" "$N" "$PRECISION" "$RECALL" "$F1" "$MAP_IDX" >> "$SUMMARY_FILE"
done

echo ""
echo "===================================================="
echo "=== BẢNG TỔNG HỢP P/R/F1 THEO N (baseline nhỏ, $MAX_DISTANCE_M m) ==="
echo "=== Xem lại tại $SUMMARY_FILE ==="
echo "===================================================="
cat "$SUMMARY_FILE"