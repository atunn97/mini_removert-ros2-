#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <vector>

namespace ground_filter
{
    using PointT = pcl::PointXYZ;

    // Trả về mask cùng kích thước với cloud->points:
    // true = điểm thuộc mặt đường (ground), false = không phải ground
    std::vector<bool> detectGroundMask(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        int seed=0,
        float distance_threshold = 0.2f);

    // === Hai hàm dưới đây cho đề bài E2: fit ground MỘT LẦN trên scan rồi dùng CHUNG
    // hệ số cho map (map đã ở hệ scan nên không cần biến đổi mặt phẳng — đề bài C tự
    // tan biến). Đối chiếu `scripts/src_quality.py`: fit_plane_ransac + ground_mask_from_plane.

    // Trả hệ số mặt phẳng (a,b,c,d) đã CHUẨN HOÁ, với a²+b²+c² = 1.
    // Cùng cấu hình RANSAC với detectGroundMask (SACMODEL_PLANE, 200 vòng, cùng cách xáo
    // theo `seed`), nên cùng `seed` thì hai hàm nhìn thấy cùng một mặt phẳng.
    // Ném std::runtime_error nếu RANSAC không ra mặt phẳng — KHÔNG trả mặt phẳng rỗng,
    // vì (0,0,0,0) sẽ làm maskFromPlane gán TOÀN BỘ điểm là ground mà không kêu tiếng nào.
    Eigen::Vector4f detectGroundPlane(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        int seed = 0,
        float distance_threshold = 0.2f);

    // Mask theo khoảng cách điểm→mặt phẳng: |n·p + d| <= distance_threshold.
    // Dùng TRỊ TUYỆT ĐỐI: PCL trả pháp tuyến với dấu tuỳ ý (mặt đường có thể ra normal
    // hướng lên hoặc hướng xuống tuỳ bộ 3 điểm RANSAC bốc trúng), nên mọi luật dựa vào
    // dấu của n·p + d đều lật ngẫu nhiên giữa các lần chạy.
    //
    // ĐÃ KIỂM: maskFromPlane(cloud, detectGroundPlane(cloud, s), t) trùng ĐÚNG TỪNG ĐIỂM
    // với detectGroundMask(cloud, s, t) — đo trên seq04/150, seed 0/1/2, lệch 0 điểm.
    // Không phải trùng do may: với setOptimizeCoefficients(true), PCL refit hệ số rồi
    // CHỌN LẠI inlier bằng selectWithinDistance trên hệ số mới
    // (/usr/include/pcl-1.14/pcl/segmentation/impl/sac_segmentation.hpp:124-125), mà
    // selectWithinDistance của SACMODEL_PLANE chính là |n·p + d| <= threshold.
    // Hệ quả cho E2: đổi sang đường ground mới KHÔNG tự nó làm F1 xê dịch, nên mọi so
    // sánh với mốc 0.707 ± 0.023 vẫn là so sánh CÓ CẶP.
    std::vector<bool> maskFromPlane(
        const pcl::PointCloud<PointT>::Ptr& cloud,
        const Eigen::Vector4f& plane,
        float distance_threshold = 0.2f);
}
