"""Internal navigation phases, independent of mission state numbers."""

IDLE = 0
WAIT_LOCALIZATION = 1
ACQUIRE_ROUTE = 2
FOLLOW_ROUTE = 3
FINAL_GOAL = 4
NAVIGATE_POSE = 5
ALIGN_HEADING = 6
ALIGN_FOR_GRASP = 7

# Compatibility names consumed by the migrated route implementation. They map
# to navigation phases and no longer represent mission state-machine values.
NAVIGATE_TO_PICKUP_STAGING = FOLLOW_ROUTE
ARRIVED_PICKUP_STAGING = FINAL_GOAL

NAMES = {
    IDLE: "IDLE",
    WAIT_LOCALIZATION: "WAIT_LOCALIZATION",
    ACQUIRE_ROUTE: "ACQUIRE_ROUTE",
    FOLLOW_ROUTE: "FOLLOW_ROUTE",
    FINAL_GOAL: "FINAL_GOAL",
    NAVIGATE_POSE: "NAVIGATE_POSE",
    ALIGN_HEADING: "ALIGN_HEADING",
    ALIGN_FOR_GRASP: "ALIGN_FOR_GRASP",
}
