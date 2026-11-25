"""Attach public measured arm samples to a feature's local read cache."""
def attach_feedback(node, reader):
    def received(sample):
        reader.observe_arm(sample.angles, sample.monotonic, sample.wall_time)
    node.mc.set_angle_callback(received)
