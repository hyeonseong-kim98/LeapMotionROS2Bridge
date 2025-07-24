uint32          id              # A unique ID for a hand tracked across frames. If tracking of a physical hand is lost, a new ID is assigned when tracking is reacquired.
string          type            # Identifies the chirality of this hand.
uint64          visible_time    # The total amount of time this hand has been tracked, in microseconds.
float32         pinch_distance  # The distance between index finger and thumb.
float32         grab_angle      # The average angle of fingers to palm.
float32         pinch_strength  # The normalized estimate of the pinch pose - a pinch is between the thumb and any other finger. Zero is not pinching; one is fully pinched.
float32         grab_strength   # The normalized estimate of the grab hand pose - a grab is all four fingers (excluding thumb) curled towards the palm. Zero is not grabbing; one is fully grabbing.
LeapPalm        palm            # Additional information associated with the palm.
LeapDigit[5]    digits          # The fingers of this hand. (thumb, index, middle, ring, pinky)
LeapBone        arm             # The arm to which this hand is attached.An arm consists of a single LEAP_BONE struct.