// Sketch to control AAR Robot via Firmata over Bluetooth

// Do not remove the include below
#include "firmata_aar.h"

#include <Firmata.h>

#define red_line_led 7
#define motor_rf 5
#define motor_rb 6
#define motor_lf 9
#define motor_lb 10
#define led_pin 13

#define SPEED_TIMER_BASE 500

namespace {

int volatile speed_timer = 0;
byte volatile mleft_speed = 0;
byte volatile mright_speed = 0;
int volatile mleft_counter = 0;
int volatile mright_counter = 0;

float left_to_right = 1.0;

byte volatile report_speed = 0;

void motorSet(byte lf, byte lb, byte rf, byte rb)
{
	analogWrite(motor_lf, lf);
	analogWrite(motor_lb, lb);
	analogWrite(motor_rf, rf);
	analogWrite(motor_rb, rb);
}

void motorStop()
{
	motorSet(0, 0, 0, 0);
}

void leftForward(byte spd)  // move left forward speed 0..255
{
	motorSet(spd, 0, 0, 0);
}

void leftBackward(byte spd)  // move left backward speed 0..255
{
	motorSet(0, spd, 0, 0);
}

void rightForward(byte spd)  // move right forward speed 0..255
{
	motorSet(0, 0, spd, 0);
}

void rightBackward(byte spd)  // move right backward speed 0..255
{
	motorSet(0, 0, 0, spd);
}

void forward(byte spd)  // move forward speed 0..255
{
	byte const right_speed = static_cast<byte>(left_to_right * spd);

	motorSet(spd, 0, right_speed, 0);
}

void calibrate_forward(void)  // full power
{
	motorSet(255, 0, 255, 0);
}

void backward(byte spd)  // move backward speed 0..255
{
	motorSet(0, spd, 0, spd);
}

void leftFast(byte spd)  // left forward, right back > fast turn
{
	motorSet(spd, 0, 0, spd);
}

void rightFast(byte spd) // left back, right forward > fast turn
{
	motorSet(0, spd, spd, 0);
}

ISR(TIMER2_OVF_vect)
{
	speed_timer++;

	if (speed_timer > SPEED_TIMER_BASE)
	{
		report_speed = 1;
		digitalWrite(led_pin, digitalRead(led_pin) ^ 1);

		if (mright_counter > 255)
			mright_counter = 255;
		mright_speed = mright_counter;
		if (mleft_counter > 255)
			mleft_counter = 255;
		mleft_speed = mleft_counter;
		mright_counter = 0;
		mleft_counter = 0;
		speed_timer = 0;
	}
}

void
calibrate(void)
{
	motorStop();

	noInterrupts();

	mright_counter = 0;
	mleft_counter = 0;

	interrupts();

	calibrate_forward();

	delay(3000);

	motorStop();

	left_to_right = (float) mleft_speed / (float) mright_speed;
}

void leftsens()  //external interrupt left encoder
{
  detachInterrupt(1);
  mleft_counter++;
  attachInterrupt(1, leftsens, CHANGE);
}

void rightsens()  //external interrupt right encoder
{
  detachInterrupt(0);
  mright_counter++;
  attachInterrupt(0, rightsens, CHANGE);
};


enum commands {
    SYSEX_AAR_MOVE = 0x50,
    SYSEX_AAR_REPORT = 0x51,
    SYSEX_AAR_COMMAND = 0x52,
    SYSEX_AAR_REPLY = 0x53
};

enum SYSEX_MOVE {
    SYSEX_MOVE_STOP = 0,
    SYSEX_MOVE_FORWARD = 1,
    SYSEX_MOVE_BACKWARD = 2,
    SYSEX_MOVE_RIGHT = 3,
    SYSEX_MOVE_LEFT = 4
};

enum SYSEX_REPORT {
	SYSEX_REPORT_FIRMWARE = 0
};

enum SYSEX_COMMAND {
	SYSEX_COMMAND_SENSORLIGHT_ON = 0,
	SYSEX_COMMAND_SENSORLIGHT_OFF = 1,
	SYSEX_COMMAND_CALIBRATE = 2
};

enum SYSEX_REPLY {
	SYSEX_REPLY_SPEED = 0
};

void doMove(byte dir, byte speed)
{
	switch (dir)
	{
	case SYSEX_MOVE_STOP:
		motorStop();
		break; // STOP
	case SYSEX_MOVE_FORWARD:
		forward(speed);
		break; // FOREWARD
	case SYSEX_MOVE_LEFT:
		rightForward(speed);
		break; // LEFT
	case SYSEX_MOVE_RIGHT:
		leftForward(speed);
		break; // RIGHT
	case SYSEX_MOVE_BACKWARD:
		backward(speed);
		break; // BACKWARD
	default:
		String data((int) dir);
		data += " ";
		data +=  String((int) speed);
		String msg = "UNKNOWN MOVE ";
		msg += data;
		Firmata.sendString(msg.c_str());
		break;
	}
}

void
doReport(byte what)
{
	switch (what) {
	case SYSEX_REPORT_FIRMWARE:
		Firmata.printFirmwareVersion();
		break;
	default:
		String data((int) what);
		String msg = "UNKNOWN REPORT ";
		msg += data;
		Firmata.sendString(msg.c_str());
		break;
	}
}

void
doCommand(byte what)
{
	switch (what) {
	case SYSEX_COMMAND_SENSORLIGHT_ON:
		digitalWrite(red_line_led, HIGH);
		break;
	case SYSEX_COMMAND_SENSORLIGHT_OFF:
		digitalWrite(red_line_led, LOW);
		break;
	case SYSEX_COMMAND_CALIBRATE:
		calibrate();
		break;
	default:
		String data((int) what);
		String msg = "UNKNOWN COMMAND ";
		msg += data;
		Firmata.sendString(msg.c_str());
		break;
	}
}

/* Decode from serial protocol back to raw bytes, as it happens for Strings */
void decodeSysex(byte *argc, byte*argv)
{
	byte bufferLength = *argc;
	byte i = 0;
	byte j = 0;
	while (j < bufferLength)
	{
		// The string length will only be at most half the size of the
		// stored input buffer so we can decode the string within the buffer.
		argv[j] = argv[i];
		i++;
		argv[j] += (argv[i] << 7);
		i++;
		j++;
	}
	*argc = bufferLength / 2;
}

void sysexCallback(byte command, byte argc, byte*argv)
{
	decodeSysex(&argc, argv);

	switch (command)
	{
	case SYSEX_AAR_MOVE:
		{
			byte dir = argv[0];
			byte speed = argv[1];
			doMove(dir, speed);
		}
		break;
	case SYSEX_AAR_REPORT:
		{
			doReport(argv[0]);
		}
		break;
	case SYSEX_AAR_COMMAND:
		{
			doCommand(argv[0]);
		}
		break;
	default:
		Firmata.sendString("got unknown command");
		break;
	}
}

} // anonymous namespace

/*
 * The AAR Bluetooth device needs 9600 Baud set!
 */
#define BAUDRATE 9600UL

void setup()
{
	//set pin I/O
#if 0
	pinMode(2, INPUT);	// encoder right
	pinMode(3, INPUT);  // encoder left
#endif
	pinMode(motor_rf, OUTPUT);	//right motor foreward
	pinMode(motor_rb, OUTPUT);	//right motor backward
	pinMode(motor_lf, OUTPUT);	//left motor foreward
	pinMode(motor_lb, OUTPUT); //left motor backward

	pinMode(red_line_led, OUTPUT);	// red line LED

	digitalWrite(red_line_led, LOW);

#if 0
	pinMode(8, OUTPUT);  // status led
	pinMode(A2, INPUT);  // battery sensor
#endif

	pinMode(A6, INPUT); // linesensor left
	pinMode(A7, INPUT); // linesensor right

	motorStop();

	Serial.begin(BAUDRATE);
	Firmata.setFirmwareVersion(0, 1);
	Firmata.attach(START_SYSEX, sysexCallback);
	Firmata.attach(START_SYSEX, sysexCallback);
	Firmata.begin(BAUDRATE);

	noInterrupts();           // disable all interrupts

	// TIMER2 Settings
	TCCR2B = (1 << CS22) | (0 << CS21) | (1 << CS20); // Fosc/64 (prescaler)
	TCCR2A = (0 << WGM20) | (1 << WGM21) | (0 << COM2A0) | (0 << COM2A1); //CTC mode normal port operation
	OCR2A = 255;
	TIMSK2 |= (1 << TOIE2) | (0 << OCIE2A);

	interrupts();             // enable all interrupts

	// set interrupt for encoders
	attachInterrupt(1, leftsens, CHANGE);
	attachInterrupt(0, rightsens, CHANGE);
}

void loop()
{
	if (report_speed)
	{
	    byte const left_sensor_white = analogRead(A6) >= 60;
	    byte const right_sensor_white = analogRead(A7) >= 60;

		byte msg[] =
		{ SYSEX_REPLY_SPEED, mleft_speed, mright_speed, left_sensor_white,
				right_sensor_white };

		Firmata.sendSysex(SYSEX_AAR_REPLY, sizeof(msg), msg);

		report_speed = 0;
	}

	while (Firmata.available())
	{
		Firmata.processInput();
	}
}
