LeapVector      prev_joint      # The base of the bone, closer to the heart. The bones origin.
LeapVector      next_joint      # The end of the bone, further from the heart.
float32         width           # The average width of the flesh around the bone in millimeters.
LeapQuaternion  rotation        # Rotation in world space from the forward direction. Convert the quaternion to a matrix to derive the basis vectors.