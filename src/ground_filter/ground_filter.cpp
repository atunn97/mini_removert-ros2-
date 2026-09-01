#include "ground_filter/ground_filter.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <random>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ground_filter
{
namespace
{

// Toàn bộ phần RANSAC dùng chung cho detectGroundMask và detectGroundPlane. Tách ra để
// hai hàm KHÔNG THỂ lệch cấu hình: cùng seed thì phải nhìn thấy đúng một mặt phẳng.
void segmentGround(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    int seed,
    float distance_threshold,
    pcl::PointIndices& inliers,
    pcl::ModelCoefficients& coefficients)
{
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setMaxIterations(200);

    seg.setInputCloud(cloud);
    // xáo shuffled_indices_ → PCL bốc bộ 3 khác → mặt phẳng hợp lệ khác;
    if (seed != 0)
    {   pcl::IndicesPtr v(new pcl::Indices(cloud->size()));
        std::iota(v->begin(), v->end(), 0);
        std::mt19937 rng(seed);
        std::shuffle(v->begin(), v->end(), rng);
        seg.setIndices(v);}
    seg.segment(inliers, coefficients);
}

} // namespace

std::vector<bool> detectGroundMask(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    int seed,
    float distance_threshold)
{
    std::vector<bool> is_ground(cloud->points.size(), false);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    segmentGround(cloud, seed, distance_threshold, inliers, coefficients);

    for (int idx : inliers.indices)
        is_ground[idx] = true;

    return is_ground;
}

Eigen::Vector4f detectGroundPlane(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    int seed,
    float distance_threshold)
{
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    segmentGround(cloud, seed, distance_threshold, inliers, coefficients);

    // RANSAC không tìm nổi mặt phẳng (cloud quá nhỏ / suy biến) thì values rỗng. Ném lỗi
    // chứ KHÔNG trả (0,0,0,0): mặt phẳng rỗng làm maskFromPlane gán mọi điểm là ground,
    // tức mọi điểm đều bị ép static — pipeline chạy tiếp, ra F1 = 0, không kêu tiếng nào.
    if (coefficients.values.size() < 4)
        throw std::runtime_error("ground_filter: RANSAC khong ra mat phang (cloud rong hay suy bien?)");

    Eigen::Vector4f plane(coefficients.values[0], coefficients.values[1],
                          coefficients.values[2], coefficients.values[3]);

    // Chuẩn hoá để |n·p + d| đúng bằng khoảng cách MÉT tới mặt phẳng. PCL vốn đã trả
    // hệ số chuẩn hoá, nhưng chia lại ở đây thì công thức trong maskFromPlane đúng
    // theo định nghĩa chứ không đúng nhờ may.
    const float n_norm = plane.head<3>().norm();
    if (n_norm < 1e-9f)
        throw std::runtime_error("ground_filter: phap tuyen mat phang bang 0");
    plane /= n_norm;

    return plane;
}

std::vector<bool> maskFromPlane(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Vector4f& plane,
    float distance_threshold)
{
    const size_t n = cloud->points.size();
    std::vector<bool> is_ground(n, false);

    for (size_t i = 0; i < n; ++i)
    {
        const auto& p = cloud->points[i];
        // Trị tuyệt đối: dấu của n·p + d lật ngẫu nhiên giữa các lần fit (xem header).
        const float dist = std::fabs(plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]);
        is_ground[i] = (dist <= distance_threshold);
    }

    return is_ground;
}

} // namespace ground_filter
