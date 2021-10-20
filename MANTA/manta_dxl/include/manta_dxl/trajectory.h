

#ifndef TRAJECTORY_H
#define TRAJECTORY_H


class Trajectory {

public:

	Trajectory(float _maxVel = 100000000000, float _setAcc = 2000, float _setDec = -1, float _thresh = 5);

	bool setTargetPos(float _target, float _time);

	float getTarget() { return target; };

	void setMaxVel(float _maxVel) { maxVel = _maxVel; };
	void setAcc(float _acc) { acc = _acc; };
	void setDec(float _dec) { dec = _dec; };

	float getMaxVel() { return maxVel; };
	float getAcc() { return acc; };
	float getDec() { return dec; };

	float getPos() { return curPos; };
	float getVel() { return curVel; };

	void setPos(float newPos = 0);
	
	void reset(float newPos = 0);

	bool ready() { return noTasks; };

	float update(float dT);
	float update();

	~Trajectory(); 

private:

	float threshold;   
	float target;     
	float curPos;     
	float curVel;    
	float maxVel;      
	float velGoal;     
	float acc;         
	float dec;         
	unsigned long oldTime;
	unsigned long startTime;
	float target_time;
	bool noTasks;
};


#endif /* TRAJECTORY_H */