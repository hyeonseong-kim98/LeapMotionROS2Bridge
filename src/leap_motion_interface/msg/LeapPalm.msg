LeapVector      position            # The center position of the palm in millimeters from the Ultraleap Tracking camera device origin.
LeapVector      stabilized_position # The time-filtered and stabilized position of the palm.
LeapVector      velocity            # The rate of change of the palm position in millimeters per second.
LeapVector      normal              # The normal vector to the palm. If your hand is flat, this vector will point downward, or "out" of the front surface of your palm.
float32         width               # The estimated width of the palm when the hand is in a flat position.
LeapVector      direction           # The unit direction vector pointing from the palm position toward the fingers.
LeapQuaternion  orientation         # The quaternion representing the palm's orientation corresponding to the basis {normal x direction, -normal, -direction}