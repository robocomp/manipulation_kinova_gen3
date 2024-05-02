//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: KinovaArm.ice
//  Source: KinovaArm.idsl
//
//******************************************************************
#ifndef ROBOCOMPKINOVAARM_ICE
#define ROBOCOMPKINOVAARM_ICE
module RoboCompKinovaArm
{
	enum ArmJoints {  base, shoulderOne, shoulderTwo, elbowOne, elbowTwo, wristOne, wristTwo  };
	struct TPose
	{
		float x;
		float y;
		float z;
		float rx;
		float ry;
		float rz;
		float qta;
		float qtb;
		float qtc;
		float qtd;
		string parent;
	};
	struct TGripper
	{
		float opening;
		float lforce;
		float ltipforce;
		float rforce;
		float rtipforce;
		float distance;
	};
	struct TJoint
	{
		float angle;
		float velocity;
		float force;
	};
	sequence <TJoint> TJointSeq;
	struct TJoints
	{
		TJointSeq joints;
		long timestamp;
	};
	sequence <float> Speeds;
	struct TJointSpeeds
	{
		Speeds jointSpeeds;
	};
	interface KinovaArm
	{
		void closeGripper ();
		TPose getCenterOfTool (ArmJoints referencedTo);
		TGripper getGripperState ();
		TJoints getJointsState ();
		void moveJointsWithSpeed (TJointSpeeds speeds);
		void openGripper ();
		void setCenterOfTool (TPose pose, ArmJoints referencedTo);
	};
};

#endif
