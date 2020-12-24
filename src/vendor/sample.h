#ifndef BATHY_SONAR_ROS_SAMPLE_H
#define BATHY_SONAR_ROS_SAMPLE_H

struct sample {
    // polar coordinates at transducer
    double m_range;					// distance to point - polar in metres
    double m_angle_s;				// sine of angle to point - polar range -1 to +1
    double m_angle_c;				// cosine of angle to point - polar range -1 to +1
    unsigned short int m_amp;		// amplitude value
    bool m_valid;                   // filter status
};

#endif //BATHY_SONAR_ROS_SAMPLE_H
