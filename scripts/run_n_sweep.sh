#!/usr/bin/env bash
# Vòng lặp N = 2,4,6,8,10 cho Việc 4: kiểm giả thuyết F1 tăng đơn điệu theo N.
# Window cố định ±60 quanh scan_idx=150, tăng mật độ map theo N (không tăng khoảng cách xa nhất).
set -e

# ---- CHỈNH LẠI 4 PATH NÀY CHO ĐÚNG MÁY BẠN NẾU CẦN ----
BINARY="/home/atun/mini_removert/build/mini_removert/mini_removert"
PCD_DIR="/home/atun/kitti_data/dataset/sequences/04/pcd"
POSES="/home/atun/kitti_data/dataset/poses/04.txt"
CALIB="/home/atun/kitti_data/dataset/sequences/04/calib.txt"
LABEL_DIR="/home/atun/kitti_data/dataset/sequences/04/labels"
EVALUATE_PY="/home/atun/mini_removert/scripts/evaluate.py"
# ---------------------------------------------------------

SCAN_IDX=150
THRESHOLD=0.5
RESULTS_DIR="/home/atun/mini_removert/results_n_sweep"

mkdir -p "$RESULTS_DIR"
cd "$RESULTS_DIR"

LABEL_FILE=$(printf "%s/%06d.label" "$LABEL_DIR" "$SCAN_IDX")

# Danh sách map_idx cho từng N (window cố định ±60, cách đều, tăng mật độ)
declare -A MAP_IDX_FOR_N
MAP_IDX_FOR_N[2]="90 210"
MAP_IDX_FOR_N[4]="90 120 180 210"
MAP_IDX_FOR_N[6]="90 110 130 170 190 210"
MAP_IDX_FOR_N[8]="90 105 120 135 165 180 195 210"
MAP_IDX_FOR_N[10]="90 102 114 126 138 162 174 186 198 210"

SUMMARY_FILE="$RESULTS_DIR/summary.txt"
> "$SUMMARY_FILE"
printf "%-4s %-10s %-10s %-10s\n" "N" "Precision" "Recall" "F1" >> "$SUMMARY_FILE"

for N in 2 4 6 8 10; do
    MAP_IDX="${MAP_IDX_FOR_N[$N]}"
    echo ""
    echo "===================================================="
    echo "=== N=$N | scan_idx=$SCAN_IDX | map_idx: $MAP_IDX ==="
    echo "===================================================="

    RUN_LOG="$RESULTS_DIR/run_N${N}.log"
    "$BINARY" "$PCD_DIR" "$POSES" "$CALIB" "$SCAN_IDX" "$THRESHOLD" $MAP_IDX | tee "$RUN_LOG"

    # Output mặc định của binary: dynamic_indices_scan<SCAN_IDX>.txt trong cwd hiện tại (RESULTS_DIR)
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

    printf "%-4s %-10s %-10s %-10s\n" "$N" "$PRECISION" "$RECALL" "$F1" >> "$SUMMARY_FILE"
done

echo ""
echo "===================================================="
echo "=== BẢNG TỔNG HỢP P/R/F1 THEO N (xem lại tại $SUMMARY_FILE) ==="
echo "===================================================="
cat "$SUMMARY_FILE"
