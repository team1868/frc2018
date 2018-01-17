#ifndef SRC_AUTO_MOTIONPROFILING_MOTIONPROFILE_H_
#define SRC_AUTO_MOTIONPROFILING_MOTIONPROFILE_H_

class MotionProfile {
public:
	virtual int GetLengthOfLeftMotionProfile() {
		return kLeftMotionProfileSz;
	}

	virtual int GetLengthOfRightMotionProfile() {
		return kRightMotionProfileSz;
	}

	virtual double (*GetLeftMotionProfile())[3] {
	    return kLeftMotionProfile;
	}

	virtual double (*GetRightMotionProfile())[3] {
	    return kRightMotionProfile;
	}

	virtual ~MotionProfile() {}
protected:
	int kLeftMotionProfileSz;
	double kLeftMotionProfile[][3];

	int kRightMotionProfileSz;
	double kRightMotionProfile[][3];
};

#endif /* SRC_AUTO_MOTIONPROFILING_MOTIONPROFILE_H_ */
