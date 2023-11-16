#include "main.h"
#ifndef DEVICES_H
#include "devices.h"
#endif

class catapult {
private:
	bool is_cata_down = false;
	bool cata_button_held = false;


public:


	void start() {
		cata_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		int cata_angle;
		while (true) {
			// Press right bumper to shoot
			cata_button_held = master.get_digital(E_CONTROLLER_DIGITAL_R1);
			// FIGURE OUT START ANGLE OF CATAPULT (exact value)
			cata_angle = cata_tracker.get_position();
			if (cata_button_held) {
				cata_motor.move(127);
			} else {
				if (cata_angle < 5000) { // Centidegrees
					cata_motor.move(127);
				} else {
					cata_motor.move(0);
				}
			}
			cout << "Cata angle: " << cata_angle << endl;
			delay(10);
		}
	}
};
