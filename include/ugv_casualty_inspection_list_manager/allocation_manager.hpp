#ifndef __UGV_CASUALTY_INSPECTION_LIST_ALLOCATION_MANAGER__
#define __UGV_CASUALTY_INSPECTION_LIST_ALLOCATION_MANAGER__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <triage_task_allocation_interface/msg/global_task_allocation.hpp>
#include <triage_task_allocation_interface/srv/give_allocation.hpp>

class AllocationManager : public rclcpp::Node {

public:
    AllocationManager();
    virtual ~AllocationManager();
    void initialize();

private:

  // ==================================================
  // Subscribers.
  // ==================================================
    rclcpp::Subscription<triage_task_allocation_interface::msg::GlobalTaskAllocation>::SharedPtr sub_global_task_allocation;
    rclcpp::Subscription<triage_task_allocation_interface::msg::CasualtyDescription>::SharedPtr sub_working_on_allocation;

  // ==================================================
  // Publishers.
  // ==================================================
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_got_new_allocation;
  rclcpp::Publisher<triage_task_allocation_interface::msg::CasualtyDescription>::SharedPtr pub_working_on_allocation;

  // ==================================================
  // Services.
  // ==================================================
  rclcpp::Service<triage_task_allocation_interface::srv::GiveAllocation>::SharedPtr srv_give_allocation;

  // ==================================================
  // Callbacks.
  // ==================================================
  void callback_allocations(triage_task_allocation_interface::msg::GlobalTaskAllocation msg);
  void callback_give_allocation(
      const std::shared_ptr<triage_task_allocation_interface::srv::GiveAllocation::Request> req,
      const std::shared_ptr<triage_task_allocation_interface::srv::GiveAllocation::Response> res
      );

  // ==================================================
  // Other stuff.
  // ==================================================
  int got_allocation_for_casualty(unsigned int id);

  std::vector<triage_task_allocation_interface::msg::TriageTaskItem> allocations;
    
   bool initialized_ = false;
   std::string _robot_name_;
};
#endif // __UGV_CASUALTY_INSPECTION_LIST_ALLOCATION_MANAGER__
