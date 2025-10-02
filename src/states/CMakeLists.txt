add_fsm_state_simple(Hammering_FSM_Controller_Initial)
add_fsm_state_simple(Get_In_Position_Task)
# add_fsm_state_simple(Hit_Task)
# add_fsm_state_simple(Post_Impact_Task)


# target_link_libraries(Get_In_Position_Task PUBLIC mc_rtc::mc_rtc_ros)
# ament_target_dependencies(Get_In_Position_Task PUBLIC 
#   visualization_msgs
# )
target_link_libraries(Get_In_Position_Task PUBLIC 
    pinocchio::pinocchio
)
#  ament_target_dependencies(Get_In_Position_Task PUBLIC 
#    pinocchio
#  )
add_fsm_data_directory(data)
