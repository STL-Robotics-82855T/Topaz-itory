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
		while (true) {
			cata_button_held = master.get_digital(E_CONTROLLER_DIGITAL_R1);
			// Press right bumper to shoot
			if (cata_button_held) {
				cata_motor.move(127);
			}
			delay(5);
		}
	}

	void rewind_cata() {
		cata_tracker.reset();
		int cata_angle;
		while (true) {
			cata_angle = cata_tracker.get_position();
			// FIGURE OUT START ANGLE OF CATAPULT (exact value)
			if (!cata_button_held) {
				if (cata_angle < 7500) {
					cata_motor.move(0);
				} else {
					cata_motor.move(0);
				}
			}
			delay(5);
		}
	}
};
