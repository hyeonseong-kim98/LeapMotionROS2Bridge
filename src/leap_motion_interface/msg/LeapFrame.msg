std_msgs/Header     header      # The timestamp and frame id
uint32              n_hands     # The number of hands tracked in this frame, i.e. the number of elements in the hands array.
LeapHand[]          hands       # The hand informations.
float32             framerate   # The current framerate of the hand tracking.