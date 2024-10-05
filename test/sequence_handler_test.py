from ur3_leader_follower.sequence_handler import SequenceHandler
from geometry_msgs.msg import TransformStamped, Pose

t = TransformStamped()
handler = SequenceHandler(t)
handler.read_sequences()

bar_pose = Pose()

while handler.has_next():
    waypoints = handler.get_next(bar_pose)
    print(waypoints[-1].position)