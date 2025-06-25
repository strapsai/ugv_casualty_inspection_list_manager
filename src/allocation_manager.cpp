#include "ugv_casualty_inspection_list_manager/allocation_manager.hpp"

AllocationManager::AllocationManager()
: Node("ugv_casualty_inspection_list_manager"),
  _robot_name_{"robot_name"}
{
    this->declare_parameter("robot_name", "");
}

AllocationManager::~AllocationManager(){
    RCLCPP_INFO(this->get_logger(), "Casualty Inspection List Allocator Node Destroyed");
}

void AllocationManager::initialize(){
  if (initialized_) {
    RCLCPP_WARN(this->get_logger(), "Already initialized");
  }

  // ==================================================
  // Get parameters.
  // ==================================================
  this->get_parameter("robot_name", _robot_name_);
  RCLCPP_INFO(this->get_logger(), "Robot Name: %s", _robot_name_.c_str());

  // ==================================================
  // Initialize the publishers.
  // ==================================================
  pub_got_new_allocation = create_publisher<std_msgs::msg::Bool>("/got_new_allocation", 10);
  pub_working_on_allocation = create_publisher<triage_task_allocation_interface::msg::CasualtyDescription>("/working_on_allocation", 10);

  // ==================================================
  // Initialize the subscribers.
  // ==================================================
  sub_global_task_allocation = create_subscription<triage_task_allocation_interface::msg::GlobalTaskAllocation>(
      "/global_task_allocation", 10,
      std::bind(&AllocationManager::callback_allocations, this, std::placeholders::_1));

  // ==================================================
  // Service clients.
  // ==================================================
  srv_give_allocation =
    create_service<triage_task_allocation_interface::srv::GiveAllocation>("/give_allocation",
        std::bind(&AllocationManager::callback_give_allocation, this,
          std::placeholders::_1, //request
          std::placeholders::_2 //response
          ));


  timer_working_on_allocation = create_wall_timer(
        std::chrono::milliseconds(100), //TODO: decide or make configurable
        std::bind(&AllocationManager::timer_callback_working_on, this));

  initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Casualty Inspection List Allocator Node Initialized");
}

void AllocationManager::callback_allocations(const triage_task_allocation_interface::msg::GlobalTaskAllocation msg){
  for (auto &allocation : msg.task_allocation){
    if (allocation.robot_name == _robot_name_){
      auto index = got_allocation_for_casualty(allocation.casualty_description.id);
      if (index > 0){
        allocations.at(index) = allocation;
      }
      else  {
        allocations.push_back(allocation);
      }
      std_msgs::msg::Bool msg_out;
      msg_out.data = true;
      pub_got_new_allocation->publish(msg_out);
    }
  }
}


int AllocationManager::got_allocation_for_casualty(unsigned int id){
  int i = 0;
  for (auto & a : allocations){
    if (a.casualty_description.id == id){
      return i;
    }
    i++;
  }
  return -1;
}

  void AllocationManager::callback_give_allocation(
      [[ maybe_unused ]] const std::shared_ptr<triage_task_allocation_interface::srv::GiveAllocation::Request> req,
      std::shared_ptr<triage_task_allocation_interface::srv::GiveAllocation::Response> res
      ){

    if (! allocations.empty()){
      res->task = allocations.front();
      assigned_task = 0;
      is_task_assigned = true;
    }
    else{
      auto emptyres =  triage_task_allocation_interface::msg::TriageTaskItem();
      emptyres.task_id = 666; //signifies failure
      res->task = emptyres;
    }
    return;
  }

void AllocationManager::timer_callback_working_on() {
  if ((!allocations.empty()) && is_task_assigned){
    pub_working_on_allocation->publish(allocations.at(assigned_task).casualty_description);
  }
}
