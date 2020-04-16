#ifndef NRF_UTILS_H
#define NRF_UTILS_H

/************* AUXILIAR TYPES ************/

enum class radioFunction
{
	receiver,
	sender
};

enum class networkType
{
	ssl,
	vss,
	deep
};

typedef struct {
	uint8_t 	payload;
	uint64_t 	addr[2];
	bool ack;
	uint8_t receiveChannel;
	uint8_t sendChannel;
	uint8_t 	pipeNum;
	bool			reConfig;
	radioFunction function;
} networkConfig;

typedef struct {
	double m1 = 0;
	double m2 = 0;
	double m3 = 0;
	double m4 = 0;
} motorSpeed;

typedef struct {
	double vx = 0;
	double vy = 0;
	double w = 0;
} vectorSpeed;

typedef struct kickFlags {
	bool front = false;
	bool chip = false;
	bool charge = false;
	float kickStrength = 0;
	bool ball = false;
	bool dribbler = false;
	float dribblerSpeed = 0;

	kickFlags& operator=(const kickFlags& a)
	{
			front = a.front;
			chip = a.chip;
			charge = a.charge;
			kickStrength = a.kickStrength;
			ball = a.ball;
			dribbler = a.dribbler;
			dribblerSpeed = a.dribblerSpeed;
			return *this;
	}
} kickFlags;

typedef struct
{
    int id = -1;
    double m1 = 0;
    double m2 = 0;
    double m3 = 0;
    double m4 = 0;
    double dribbler = 0;
    double kickLoad = 0;
    bool ball = false;
    double battery = 0;
} robotTelemetry;

#endif // NRF_UTILS_H
