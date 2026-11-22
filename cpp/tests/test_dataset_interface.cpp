#include "../include/lio/io/dataset.hpp"
#include "../include/lio/io/kitti_dataset.hpp"

#include <type_traits>

static_assert(
    std::is_base_of_v<lio::Dataset, lio::KittiDataset>,
    "KittiDataset must implement Dataset"
);

int main()
{
    return 0;
}