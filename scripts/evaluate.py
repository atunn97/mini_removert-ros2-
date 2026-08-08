import numpy as np
import sys

def evaluate(label_path, predicted_indices_path):
    # Ground truth
    labels = np.fromfile(label_path, dtype=np.uint32)
    semantic_labels = labels & 0xFFFF

    moving_classes = {252, 253, 254, 255, 256, 257, 258, 259}
    gt_dynamic_mask = np.isin(semantic_labels, list(moving_classes))

    n_total = len(semantic_labels)
    n_gt_dynamic = gt_dynamic_mask.sum()

    # Predicted
    predicted_indices = np.loadtxt(predicted_indices_path, dtype=int)
    if predicted_indices.ndim == 0:
        predicted_indices = predicted_indices.reshape(1)

    predicted_mask = np.zeros(n_total, dtype=bool)
    valid_indices = predicted_indices[predicted_indices < n_total]
    predicted_mask[valid_indices] = True

    true_positive  = np.sum(predicted_mask & gt_dynamic_mask)
    false_positive = np.sum(predicted_mask & ~gt_dynamic_mask)
    false_negative = np.sum(~predicted_mask & gt_dynamic_mask)

    precision = true_positive / (true_positive + false_positive + 1e-9)
    recall    = true_positive / (true_positive + false_negative + 1e-9)
    f1        = 2 * precision * recall / (precision + recall + 1e-9)

    print(f"Tổng điểm:              {n_total}")
    print(f"GT dynamic (thật):      {n_gt_dynamic}")
    print(f"Predicted dynamic:      {predicted_mask.sum()}")
    print(f"True Positive:          {true_positive}")
    print(f"False Positive:         {false_positive}")
    print(f"False Negative:         {false_negative}")
    print(f"Precision:              {precision:.3f}")
    print(f"Recall:                 {recall:.3f}")
    print(f"F1 score:                {f1:.3f}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 evaluate.py <label_file> <predicted_indices_file>")
        sys.exit(1)
    evaluate(sys.argv[1], sys.argv[2])
