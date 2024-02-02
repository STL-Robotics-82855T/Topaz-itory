#pragma once

class catapult {
private:
	bool is_cata_down = false;
	bool cata_button_held = false;
	bool auto_cata_held = false;
	int degrees = 5250;
	float motor_wattage = 0.0;
	bool drawn_back = false;
	
	bool high_wattage = false;

public:
	void start() {
		int cata_angle;
		// cata_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		while (true) {
			// Press right bumper to shoot
			cata_button_held = master.get_digital(E_CONTROLLER_DIGITAL_L2);
			auto_cata_held = master.get_digital(E_CONTROLLER_DIGITAL_B);
			motor_wattage = cata_motor.get_current_draw();

			// FIGURE OUT START ANGLE OF CATAPULT (exact value)
			if (cata_button_held) {
				// cata_motor.move(105);
				cata_motor.move(110);
			}
			else if (auto_cata_held) {
				if (!drawn_back) {
					cata_motor.move(127);
					delay(700);
					drawn_back = true;
				}
				if (vertical_distance.get() < 11) {
					cata_motor.move(127);
					high_wattage = true;
					delay(100);
					if (high_wattage && motor_wattage < 2) {
						cata_motor.move(0);
						high_wattage = false;
						drawn_back = false;
					}
				}

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
