import numpy as np
import os

def check_match(velodyne_dir, labels_dir, n_check=10):
    bin_files = sorted(f for f in os.listdir(velodyne_dir) if f.endswith('.bin'))

    print(f"Kiểm tra {n_check} file đầu tiên...")
    all_ok = True
    for fname in bin_files[:n_check]:
        idx = fname.replace('.bin', '')
        bin_path = os.path.join(velodyne_dir, fname)
        label_path = os.path.join(labels_dir, idx + '.label')

        if not os.path.exists(label_path):
            print(f"[{idx}] THIẾU file label: {label_path}")
            all_ok = False
            continue

        n_points_bin = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4).shape[0]
        n_points_label = np.fromfile(label_path, dtype=np.uint32).shape[0]

        status = "OK" if n_points_bin == n_points_label else "MISMATCH!"
        print(f"[{idx}] bin={n_points_bin}  label={n_points_label}  {status}")
        if n_points_bin != n_points_label:
            all_ok = False

    print("\n=> TẤT CẢ KHỚP" if all_ok else "\n=> CÓ VẤN ĐỀ, xem lại các dòng MISMATCH/THIẾU ở trên")

if __name__ == "__main__":
    check_match(
        "/home/atun/kitti_data/dataset/sequences/04/velodyne",
        "/home/atun/kitti_data/dataset/sequences/04/labels",
        n_check=10
    )
