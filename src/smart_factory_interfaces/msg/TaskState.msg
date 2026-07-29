uint8 IDLE=0
uint8 ACCEPT_TASK=1
uint8 VALIDATE_TASK=2
uint8 CHECK_LOCALIZATION=3
uint8 GET_PICKUP_STAGING_GOAL=4
uint8 NAVIGATE_TO_PICKUP_STAGING=5
uint8 ARRIVED_PICKUP_STAGING=6
uint8 TASK_FAILED=250

std_msgs/Header header
string task_id
uint8 state
uint8 retry_count
string detail
