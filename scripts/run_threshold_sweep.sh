#!/usr/bin/env bash
# Sweep `threshold` cua discrepancy (tham so dong lenh thu 5 cua binary), o cau hinh
# da chot N=4 / max_distance_m=4.0. Lam viec so 2 trong muc 7 cua HANDOFF_2026-08-12
# ("threshold va vote_threshold dang co dinh 0.5, chua he sweep voi baseline moi").
#
# Vi sao sweep nay HOP LE bat chap phat hien "F1 cuc nhay voi ground mask"
# (HANDOFF_2026-08-13 muc 11): ground mask KHONG phu thuoc threshold, nen moi nhanh
# so sanh dung chung dung mot mask -> so sanh co cap (paired), nhieu mask bi khu.
set -e

BINARY="/home/atun/mini_removert/build/mini_removert/mini_removert"
PCD_DIR="/home/atun/kitti_data/dataset/sequences/04/pcd"
POSES="/home/atun/kitti_data/dataset/poses/04.txt"
CALIB="/home/atun/kitti_data/dataset/sequences/04/calib.txt"
LABEL_DIR="/home/atun/kitti_data/dataset/sequences/04/labels"
EVALUATE_PY="/home/atun/mini_removert/scripts/evaluate.py"
SELECT_MAPS_PY="/home/atun/mini_removert/scripts/select_maps_by_distance.py"

N=4
MAX_DISTANCE_M=4.0
SCANS="50 100 150 200 250"
THRESHOLDS="0.2 0.3 0.4 0.5 0.7 0.8 0.9 1.0 1.1 1.2 1.3 1.5"
RESULTS_DIR="/home/atun/mini_removert/results_threshold_sweep"

mkdir -p "$RESULTS_DIR"
cd "$RESULTS_DIR"

SUMMARY_FILE="$RESULTS_DIR/summary.txt"
> "$SUMMARY_FILE"
printf "%-6s %-6s %-10s %-10s %-10s %-8s\n" "scan" "thr" "Precision" "Recall" "F1" "n_dyn" >> "$SUMMARY_FILE"

for SCAN_IDX in $SCANS; do
    # map_idx chon MOT LAN cho moi scan - khong phu thuoc threshold
    SELECT_LOG="$RESULTS_DIR/select_scan${SCAN_IDX}.log"
    python3 "$SELECT_MAPS_PY" "$POSES" "$CALIB" "$SCAN_IDX" "$N" "$MAX_DISTANCE_M" > "$SELECT_LOG" 2>&1
    MAP_IDX=$(tail -1 "$SELECT_LOG")
    LABEL_FILE=$(printf "%s/%06d.label" "$LABEL_DIR" "$SCAN_IDX")
    echo ""
    echo "=== scan $SCAN_IDX | map_idx: $MAP_IDX ==="

    for THR in $THRESHOLDS; do
        TAG="scan${SCAN_IDX}_thr${THR}"
        "$BINARY" "$PCD_DIR" "$POSES" "$CALIB" "$SCAN_IDX" "$THR" $MAP_IDX \
            > "$RESULTS_DIR/run_${TAG}.log" 2>&1

        OUT="$RESULTS_DIR/dynamic_indices_scan${SCAN_IDX}.txt"
        PER_TAG_OUT="$RESULTS_DIR/dynamic_indices_${TAG}.txt"
        mv "$OUT" "$PER_TAG_OUT"

        EVAL_LOG="$RESULTS_DIR/eval_${TAG}.log"
        python3 "$EVALUATE_PY" "$LABEL_FILE" "$PER_TAG_OUT" > "$EVAL_LOG"

        PRECISION=$(grep "Precision:" "$EVAL_LOG" | awk '{print $2}')
        RECALL=$(grep "Recall:" "$EVAL_LOG" | awk '{print $2}')
        F1=$(grep "F1 score:" "$EVAL_LOG" | awk '{print $3}')
        NDYN=$(grep "Predicted dynamic:" "$EVAL_LOG" | awk '{print $3}')

        printf "  thr=%-5s P=%-7s R=%-7s F1=%-7s (%s diem)\n" "$THR" "$PRECISION" "$RECALL" "$F1" "$NDYN"
        printf "%-6s %-6s %-10s %-10s %-10s %-8s\n" "$SCAN_IDX" "$THR" "$PRECISION" "$RECALL" "$F1" "$NDYN" >> "$SUMMARY_FILE"
    done
done

echo ""
echo "=== F1 trung binh 5 scan theo threshold ==="
for THR in $THRESHOLDS; do
    awk -v t="$THR" '$2==t {s+=$5; n++} END {if(n) printf "  thr=%-5s F1_TB=%.3f  (tren %d scan)\n", t, s/n, n}' "$SUMMARY_FILE"
done
echo ""
echo "Chi tiet: $SUMMARY_FILE"
