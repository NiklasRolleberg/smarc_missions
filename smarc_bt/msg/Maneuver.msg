uint8 MANEUVER_TYPE_PAUSE = 0
uint8 MANEUVER_TYPE_COURSE = 1
uint8 MANEUVER_TYPE_WP = 2
uint8 MANEUVER_TYPE_LOITER = 3

#name of maneuver
string name

#Type of maneuver
uint8 maneuver_type

#Vehicle parameters
uint8 vehicle_mode

#Pause parameters
uint32 pause_runtime_s

#Couse parameters
uint32 course_runtime_s
float64 course_targetheading
float64 course_rpm
float64 course_targetDepth
float64 course_targetAltitude

#Wp 
float64 wp_goal_tolerance
float64 wp_targetLat
float64 wp_targetLon
float64 wp_rpm
float64 wp_targetDepth
float64 wp_targetAltitude


#Loiter
float64 loiter_inner_radius
float64 loiter_outer_radius
float64 loiter_targetLat
float64 loiter_targetLon
float64 loiter_rpm
#float64 loiter_targetDepth #Maybe later?
#float64 loiter_targetAltitude
