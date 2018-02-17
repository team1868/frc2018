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

	virtual double (*GetLeftMotionProfile())[8] {
	    return kLeftMotionProfile;
	}

	virtual double (*GetRightMotionProfile())[8] {
	    return kRightMotionProfile;
	}

	virtual ~MotionProfile() {}
protected:
	int kLeftMotionProfileSz;
	double kLeftMotionProfile[][8];

	int kRightMotionProfileSz;
	double kRightMotionProfile[][8];
};

#endif /* SRC_AUTO_MOTIONPROFILING_MOTIONPROFILE_H_ */
