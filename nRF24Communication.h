#ifndef NRF24_COMMUNICATION_H
#define NRF24_COMMUNICATION_H

#include <mbed.h>
#include <SPI.h>		// Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01P/nRF24L01P.h> // nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <nrf-config.h>
#include <nrf-utils.h>

class nRF24Communication{
public:
	// Class constructor
	nRF24Communication(PinName pinMOSI, PinName pinMISO, PinName pinSCK, PinName pinCE, PinName pinCSN,PinName pinVCC, NetworkType network, RadioFunction function);
	
	// Class destructor
	// ~nRF24Communication();

	int 	setup(int robotSwitches);
	bool	compareChannel(uint8_t channel = SSL_1_ROBOT_RECV_CH);
	int 	updateRobotId(int robotSwIR2_D);
	int 	getRobotId();
	void	printDetails();

	bool	sendTelemetryPacket(RobotTelemetry telemetry);
	bool	updatePacket(bool reconnect = false);
	int 	getTypeOfMessage();
	void 	enable();
	void	disable();


	// VSS Info
	int					getLeftMotorSpeed();
	int					getRightMotorSpeed();
	void				clearVSSData();
	
	// SSL Info
	VectorSpeed 	getRobotVectorSpeed();
	void 					clearSSLData();
	double 				getVx();
	double 				getVy();
	double 				getW();
	KickFlags* 		getKick();
	bool 					getFront();
	bool 					getChip();
	bool 					getCharge();
	float 				getKickStrength();
	bool 					getDribbler();
	float 				getDribblerSpeed();

	// Aux Info
	float 	getKP();
	float 	getKD();
	float 	getAlpha();


private:	
	RF24 _radio;
	DigitalOut _vcc;

  private:
	void	_network(NetworkType network);
	void	_configure();
	void	_resetRadio();
	void	_receive();
	void	_send();
	
	packetVSS 		_mVSS;
	packetSSL		_mSSL;
	packetTelemetrySSL _mSSLTelemetry;
	packetGeneric	_rx;
	
	NetworkConfig _config;
	KickFlags*		_kick;
	VectorSpeed		_robotVectorSpeed;

	int			  	_leftMotorSpeed, _rightMotorSpeed;
	char			_robotId;
	int 			_typeMsg, _flags;
	double			_vx, _vy, _w;
	bool 			_front, _chip, _charge, _dribbler;
	float 			_kickStrength = 0, _dribblerSpeed = 0;
	float 			_kp,_kd,_ki,_alpha;
};

#endif // NRF_COMM
