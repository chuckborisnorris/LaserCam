#define MAX_POINTS 600
#define BRIGHTNESS 200
#define PI 3.14159

#include "HeliosDac.h"
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <math.h>


//make frames
HeliosPoint frame[MAX_POINTS];


int idx = 0;
int r, g, b = 0;
int xoff, yoff = 0;
int xs, ys = 0;
int x, y = 0;
float angle = 0;

HeliosDac helios;
int numDevs = helios.OpenDevices();

void HeliosTranslate(int _xoff, int _yoff, int _zoff) {
	xoff = _xoff;
	yoff = _yoff;
	glTranslatef(float(_xoff), float(_yoff), float(_zoff));
}

void HeliosScale(float _xs, float _ys, float _zs) {
	xs = int(_xs);
	ys = int(_ys);
	glScalef(_xs, _ys, _zs);
}

void HeliosRotate( float _angle) {
	angle = _angle * PI / 180;
	glRotatef(angle, 0, 0, 1);
}

void SetHeliosColour(float _r, float _g, float _b) {
	r = int(_r * BRIGHTNESS);
	g = int(_g * BRIGHTNESS);
	b = int(_b * BRIGHTNESS);
	
	glColor3f(_r, _g, _b);
}

void WriteHeliosPoint(int _x, int _y) {
	
	x = (_x*xs*cos(angle)-_y*ys*sin(angle) + xoff)*3+1024;
	y = (_x*xs*sin(angle)+_y*ys*cos(angle) + yoff)*3+1024;
	
	frame[idx].x = 5000-x;
	frame[idx].y = y;			
	frame[idx].r = r;
	frame[idx].g = g;
	frame[idx].b = b;
	frame[idx].i = 0xff;
	
	glVertex2f(_x, _y);
	
	if (idx < MAX_POINTS) idx++;	
}

void WriteHeliosBlank(int _x, int _y) {
	frame[idx].x = x;
	frame[idx].y = y;			
	frame[idx].r = 0x00;
	frame[idx].g = 0x00;
	frame[idx].b = 0x00;
	frame[idx].i = 0x00;
	
	if (idx < MAX_POINTS) idx++;
	
	frame[idx].x = (_x*xs*cos(angle)-_y*ys*sin(angle) + xoff)*3+1024;
	frame[idx].y = (_x*xs*sin(angle)+_y*ys*cos(angle) + yoff)*3+1024;			
	frame[idx].r = 0x00;
	frame[idx].g = 0x00;
	frame[idx].b = 0x00;
	frame[idx].i = 0x00;
	
	if (idx < MAX_POINTS) idx++;
}

void WriteHeliosFrame() {
	for (int j = 0; j < numDevs; j++)
		{
		if (helios.GetStatus(j) == 1)
			helios.WriteFrame(j, 30*idx, HELIOS_FLAGS_DEFAULT, &frame[0], idx); //send the next frame	
		}	
	idx = 0;
}

void CloseHeliosDAC() {
	helios.CloseDevices();
}