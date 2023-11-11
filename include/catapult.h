#include "main.h"
#ifndef DEVICES_H
#include "devices.h"
#endif

class catapult {
private:
	bool is_cata_down = false;


public:

	void start() {
		while (true) {
			// Press right bumper to shoot
			if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
				cata_motor.move(127);
			} else {
				cata_motor.move(0);
			}
			delay(5);
		}
	}

	// void rewind_cata() {
	//     int cata_angle;
	//     while (true) {
	//         cata_angle = cata_tracker.get_angle();
	//         // FIGURE OUT START ANGLE OF CATAPULT (exact value)
	//         if (cata_angle < 88) {
	//             cata_motor.move(127);
	//         }
	//         else {
	//             cata_motor.move(0);
	//         }
	//     }
	// }
};
