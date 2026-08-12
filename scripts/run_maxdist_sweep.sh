#!/usr/bin/env bash
# So sanh anh huong cua max_distance_m (baseline nho hon) len precision/recall/F1,
# o N co dinh (thay vi sweep N o max_distance_m co dinh nhu run_n_sweep_v2.sh).
# Muc dich: kiem tra gia thuyet "baseline nho hon -> precision cao hon" (HANDOFF_VIEC4.md muc 10.2).
set -e

BINARY="/home/atun/mini_removert/build/mini_removert/mini_removert"
PCD_DIR="/home/atun/kitti_data/dataset/sequences/04/pcd"
POSES="/home/atun/kitti_data/dataset/poses/04.txt"
CALIB="/home/atun/kitti_data/dataset/sequences/04/calib.txt"
LABEL_DIR="/home/atun/kitti_data/dataset/sequences/04/labels"
EVALUATE_PY="/home/atun/mini_removert/scripts/evaluate.py"
SELECT_MAPS_PY="/home/atun/mini_removert/scripts/select_maps_by_distance.py"

SCAN_IDX=150
THRESHOLD=0.5
RESULTS_DIR="/home/atun/mini_removert/results_maxdist_sweep"

mkdir -p "$RESULTS_DIR"
cd "$RESULTS_DIR"

LABEL_FILE=$(printf "%s/%06d.label" "$LABEL_DIR" "$SCAN_IDX")

SUMMARY_FILE="$RESULTS_DIR/summary.txt"
> "$SUMMARY_FILE"
printf "%-4s %-8s %-10s %-10s %-10s %-30s\n" "N" "max_d" "Precision" "Recall" "F1" "map_idx" >> "$SUMMARY_FILE"

run_one() {
    local N=$1
    local MAXD=$2
    local TAG="N${N}_d${MAXD}"

    SELECT_LOG="$RESULTS_DIR/select_${TAG}.log"
    if ! python3 "$SELECT_MAPS_PY" "$POSES" "$CALIB" "$SCAN_IDX" "$N" "$MAXD" > "$SELECT_LOG" 2>&1; then
        echo "SKIP N=$N max_distance_m=$MAXD (khong du frame trong ban kinh)"
        return
    fi

    MAP_IDX=$(tail -1 "$SELECT_LOG")
    echo ""
    echo "=== N=$N max_distance_m=$MAXD -> map_idx: $MAP_IDX ==="

    RUN_LOG="$RESULTS_DIR/run_${TAG}.log"
    "$BINARY" "$PCD_DIR" "$POSES" "$CALIB" "$SCAN_IDX" "$THRESHOLD" $MAP_IDX > "$RUN_LOG" 2>&1

    DEFAULT_OUT="$RESULTS_DIR/dynamic_indices_scan${SCAN_IDX}.txt"
    PER_TAG_OUT="$RESULTS_DIR/dynamic_indices_scan${SCAN_IDX}_${TAG}.txt"
    mv "$DEFAULT_OUT" "$PER_TAG_OUT"

    EVAL_LOG="$RESULTS_DIR/eval_${TAG}.log"
    python3 "$EVALUATE_PY" "$LABEL_FILE" "$PER_TAG_OUT" | tee "$EVAL_LOG"

    PRECISION=$(grep "Precision:" "$EVAL_LOG" | awk '{print $2}')
    RECALL=$(grep "Recall:" "$EVAL_LOG" | awk '{print $2}')
    F1=$(grep "F1 score:" "$EVAL_LOG" | awk '{print $3}')

    printf "%-4s %-8s %-10s %-10s %-10s %-30s\n" "$N" "$MAXD" "$PRECISION" "$RECALL" "$F1" "$MAP_IDX" >> "$SUMMARY_FILE"
}

# N=2: baseline nho nhat co the, ca voi radius rat nho
for MAXD in 1.5 2 3 4 5 6 8; do
    run_one 2 "$MAXD"
done

# N=4: giu du 2 map moi ben de voting co y nghia hon, xem baseline nho co con giup khong
for MAXD in 3 4 5 6 8; do
    run_one 4 "$MAXD"
done

echo ""
echo "===================================================="
echo "=== BANG TONG HOP: anh huong max_distance_m len P/R/F1 ==="
echo "===================================================="
cat "$SUMMARY_FILE"
