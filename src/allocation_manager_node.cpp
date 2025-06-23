#include <rclcpp/rclcpp.hpp>

#include "ugv_casualty_inspection_list_manager/allocation_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto ucilm= std::make_shared<AllocationManager>();
    ucilm->initialize();

    rclcpp::spin(ucilm);
    rclcpp::shutdown();
    return 0;
}
