#pragma once

class catapult {
private:
	bool is_cata_down = false;
	bool cata_button_held = false;
	int degrees = 5200;

public:
	void start() {
		int cata_angle;
		// cata_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		while (true) {
			// Press right bumper to shoot
			cata_button_held = master.get_digital(E_CONTROLLER_DIGITAL_R1);
			// FIGURE OUT START ANGLE OF CATAPULT (exact value)
			cata_angle = cata_tracker.get_position();
			if (cata_button_held) {
				cata_motor.move(127);
			} else {
				if (cata_angle <= degrees) { // Centidegrees
					cata_motor.move(127);
					degrees = 5200;
				} else {
					cata_motor.move(0);
					degrees = 4800;
					// cata_motor.brake();
				}
			}
			// cout << "Cata angle: " << cata_angle << endl;
			delay(10);
		}
	}
};
