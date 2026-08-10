"""Data belonging to one task execution."""


class TaskContext:
    def __init__(self, task_id, target_class):
        self.task_id = task_id
        self.target_class = target_class
        self.retry_count = 0
        self.current_stage = 0
        self.pickup_staging_goal = None
        self.pickup_staging_goals = []
        self.delivery_entry_goal = None
        self.delivery_goal = None
        self.current_waypoint_index = 0
        self.last_error = 0
        self.last_message = ""
