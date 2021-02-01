#ifndef NRF_UTILS_H
#define NRF_UTILS_H

/************* AUXILIAR TYPES ************/

enum class RadioFunction
{
	receiver,
	sender
};

enum class NetworkType
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
	RadioFunction function;
} NetworkConfig;

typedef struct {
	double m1 = 0;
	double m2 = 0;
	double m3 = 0;
	double m4 = 0;
} Motors;

typedef struct {
	double x = 0;
	double y = 0;
	double w = 0;
} Vector;

typedef struct KickFlags {
	bool front = false;
	bool chip = false;
	bool charge = false;
	float kickStrength = 0;
	bool ball = false;
	bool dribbler = false;
	float dribblerSpeed = 0;

	KickFlags& operator=(const KickFlags& a)
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
} KickFlags;

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
} RobotTelemetry;

#endif // NRF_UTILS_H
