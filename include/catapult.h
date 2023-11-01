#ifndef CATAPULT_H
#define CATAPULT_H
#include "main.h"

class catapult {
	private:
		bool is_cata_down = false;
		Controller &master;
		Motor &cata_motor;
		Rotation &cata_tracker;

	public:
		catapult(Controller &controller, Motor &motor, Rotation &tracker);
		void start();
		void rewind_cata();
};
#endif
