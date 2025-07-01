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
  pub_working_on_allocation = create_publisher<triage_task_allocation_interface::msg::TriageTaskItem>("/working_on_allocation", 10);

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
  
  srv_finish_allocation =
    create_service<std_srvs::srv::Trigger>("/finish_allocation",
        std::bind(&AllocationManager::callback_finish_allocation, this,
          std::placeholders::_1, //request
          std::placeholders::_2 //response
          ));


  timer_working_on_allocation = create_wall_timer(
        std::chrono::milliseconds(100), //TODO: decide or make configurable
        std::bind(&AllocationManager::timer_callback_working_on, this));

  initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Casualty Inspection List Allocator Node Initialized");
}

void AllocationManager::callback_allocations(const triage_task_allocation_interface::msg::GlobalTaskAllocation & msg){
  pending_allocations.clear();
  for (auto &allocation : msg.task_allocation){
    if (allocation.robot_name == _robot_name_){
      auto index = is_task_finished(allocation);
      if (index < 0){ //not among finished tasks
        pending_allocations.push_back(allocation);
      }
      std_msgs::msg::Bool msg_out;
      msg_out.data = true;
      pub_got_new_allocation->publish(msg_out);
    }
  }
}


int AllocationManager::got_allocation_for_casualty(unsigned int id){
  int i = 0;
  for (auto & a : pending_allocations){
    if (a.casualty_description.id == id){
      return i;
    }
    i++;
  }
  return -1;
}

int AllocationManager::is_task_finished(const triage_task_allocation_interface::msg::TriageTaskItem & task){
  int i = 0;
  for (auto & t : finished_allocations){
    if (
        (t.casualty_description.id = task.casualty_description.id)
        &&
        (t.casualty_description.round = task.casualty_description.round)
       ){
      return i;
    }
    else {
      i++;
    }
  }

  return -1;
}

  void AllocationManager::callback_give_allocation(
      [[ maybe_unused ]] const std::shared_ptr<triage_task_allocation_interface::srv::GiveAllocation::Request> req,
      std::shared_ptr<triage_task_allocation_interface::srv::GiveAllocation::Response> res
      ){

    if (! pending_allocations.empty()){
      assigned_task = pending_allocations.front();
      res->success.data = true;
      res->task = assigned_task.value();
    }
    else{
      auto emptyres =  triage_task_allocation_interface::msg::TriageTaskItem();
      emptyres.task_id = 666; //signifies failure
      res->task = emptyres;
      res->success.data = false;
    }
    return;
  }

  void AllocationManager::callback_finish_allocation(
      [[ maybe_unused ]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res
      ){
    if (assigned_task.has_value()){
      
      std::string message;
      std::stringstream ss;
      ss << "Finishing task with ID: " <<
        assigned_task.value().task_id
        << ", casualty ID: " <<
        assigned_task.value().casualty_description.id
        << ", round: " << 
        assigned_task.value().casualty_description.round;

      res->message = ss.str();
      finished_allocations.push_back(assigned_task.value());
      bool found_in_pending = false;
      for (auto it = pending_allocations.begin(); it != pending_allocations.end();){
        if (
            ((*it).casualty_description.id == assigned_task.value().casualty_description.id)
            &&
            ((*it).casualty_description.round == assigned_task.value().casualty_description.round)
           ){
          it == pending_allocations.erase(it);
          found_in_pending = true;
        }
        else
          it++;
      }

      if (!found_in_pending){
        RCLCPP_ERROR(this->get_logger(), "Task to be finished (task ID: %d, casualty ID: %d, round: %d)", assigned_task.value().task_id, assigned_task.value().casualty_description.id, assigned_task.value().casualty_description.round);
      }


      assigned_task = std::nullopt;

      if (pending_allocations.empty()){
        std_msgs::msg::Bool msg_out;
        msg_out.data = false;
        pub_got_new_allocation->publish(msg_out);
      }

    }
    else {
      std::string message = "Can't finish a task, as none was assigned!";
      res->message = message.c_str();
    RCLCPP_ERROR(this->get_logger(), message.c_str());
      }

  }

void AllocationManager::timer_callback_working_on() {
  if (assigned_task.has_value()){
    pub_working_on_allocation->publish(assigned_task.value());
  }
}
