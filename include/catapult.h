#pragma once

class catapult {
private:
	bool is_cata_down = false;
	bool cata_button_held = false;
	int degrees = 5250;

public:
	void start() {
		int cata_angle;
		// cata_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		while (true) {
			// Press right bumper to shoot
			cata_button_held = master.get_digital(E_CONTROLLER_DIGITAL_L2);

			// FIGURE OUT START ANGLE OF CATAPULT (exact value)
			if (cata_button_held) {
				// cata_motor.move(105);
				cata_motor.move(95);
			}
			else {
				cata_motor.move(0);
			}
			// }  else {
			// 	cata_motor.move(0);
			// }
			// else {
			// 	if (cata_angle <= degrees) { // Centidegrees
			// 		cata_motor.move(127);
			// 		degrees = 5250;
			// }
			// cout << "Cata angle: " << cata_angle << endl;
			delay(10);
		}
	}
};
