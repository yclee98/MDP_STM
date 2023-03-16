#include "path.h"
extern double ultrasonicDistance;
extern uint8_t OLED_row3[20];

extern double memorizedDist;

//left right issue
//right turn need more straight

void path1Right(){
	turnRight(1,31.397);
	forward(1,33.948);
	turnLeft(1,31.397+25);
	turnRight(1,25);
	memorizedDist += 20 + 25;
	memorizedDist+= sensorDistance(50);
	sprintf(OLED_row3, "mem %d", (int)memorizedDist);
//	if (memorizedDist < 60)
//	turnLeft(1, 20);
}

void path1Left(){
	turnLeft(1,31.397);
	forward(1,33.948);
	turnRight(1,31.397+25);
	turnLeft(1,25);
	memorizedDist += 20 + 25;
	memorizedDist += sensorDistance(50);
	sprintf(OLED_row3, "mem %d", (int)memorizedDist);
//	if (memorizedDist < 60)
//	turnRight(1, 20);
}

void path2LeftRight(){ // Good
//	if (memorizedDist < 60)
//		turnLeft(1, 20);

	sensorDistance(50);
	turnRight(1,57.645);
	forward(1,61.298-5);
	turnLeft(1,148.715);
	// forward(1,47.242+7); Indoor
	forward(1,47.242+7);
	turnLeft(1,91.07);
	forward(1, memorizedDist);
	turnLeft(1,65.307);
	forward(1,23.022+5);
	turnRight(1,65.307);
	sensorDistance(10);
}

void path2LeftLeft(){ // Good
//	if (memorizedDist < 60)
//		turnLeft(1, 20);

	sensorDistance(60);
	turnLeft(1,20.794);
	forward(1,71.258);
	turnRight(1,109.671);
	forward(1,50.101);
	turnRight(1,88.877);
	forward(1, memorizedDist+10);
	turnRight(1,65.307);
	forward(1,23.022);
	turnLeft(1,65.307);
	sensorDistance(10);
}

void path2RightRight(){
//	if (memorizedDist < 60)
//		turnRight(1, 20);

	sensorDistance(60);
	turnRight(1,20.794);
	forward(1,71.258);
	turnLeft(1,111.864);
	//forward(1,47.24+4);
	forward(1,47.24+7);
	turnLeft(1,91.07);
	forward(1, memorizedDist+10);
	turnLeft(1,65.307);
	//forward(1,23.022);
	forward(1,23.022+5);
	turnRight(1,65.307);
	sensorDistance(10);
}

void path2RightLeft(){ // Good
//	if (memorizedDist < 60)
//		turnRight(1, 20);

	sensorDistance(50);
	turnLeft(1,57.645);
	forward(1,61.298-5);
	turnRight(1,148.715);
	forward(1,50.101);
	turnRight(1,88.877);
	forward(1, memorizedDist);
	turnRight(1,65.307);
	forward(1,23.022);
	turnLeft(1,65.307);
	sensorDistance(10);
}


