#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include "HeliosDacFunctions.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_Z16    // rs2_format identifies how binary data is encoded within a frame      //
#define WIDTH           1280               // Defines the number of columns for each frame or zero for auto resolve//
#define HEIGHT          720                 // Defines the number of lines for each frame or zero for auto resolve  //
#define RANGE			M_PI				
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//g++ -pthread -o dd  main.cpp HeliosDac.cpp -lglfw -lGLU -lGL -lXrandr -lXxf86vm -lXi -lXinerama -lX11 -lrt -ldl -std=c++17 -lusb-1.0 -lrealsense2

using namespace std;
int DILATION = 2;
int CUTOFF = 21;
int LINE_LENGTH = 8;
int BORDER = 120;
int TOP_BORDER = 100;
int tx, ty, rT, gT, bT = 1;
float threshold = 0.02*65.535;
float c , rN, gN, bN = 0;
float im[WIDTH][HEIGHT];
float imG[WIDTH][HEIGHT];
float imP[WIDTH][HEIGHT];
float sbl[WIDTH][HEIGHT];
float ang[WIDTH][HEIGHT];
float median[9];
bool hold = true;
bool l_to_r = true;

void printout() {
	system("clear"); 
cout << "      _____          LINE DANCE?        _____" << endl;
cout << "     |DIST-|         ***********       |LINE-|" << endl;
cout << "      =====                             =====" << endl;
cout << "      _____                             _____" << endl;
cout << "     |DIST+|                           |LINE+|" << endl;
cout << "     _=====_                           _=====_" << endl;
cout << "    / _____ \\                         / _____ \\" << endl;
cout << "   +.-'_____'-.---------------------.-'_____'-.+" << endl;
cout << "  /   |     |  '.                 .'  |  _  |   \\" << endl;
cout << " / ___| TB+ |___ \\               / ___|  G  |___ \\" << endl;
cout << "/ |             | ;  __      __ ; |  _        _ | ;" << endl;
cout << "| | LRB-   LRB+ | | |__|    |__|  |  R        B | |" << endl;
cout << "| |___       ___| ;DILATE   HOLD; |___       ___| ;" << endl;
cout << "|\\    | TB- |    /  _        _   \\    |<-/->|    /|" << endl;
cout << "| \\   |_____|  .',' ' ',  ,' ' ', '.  |_____|  .' |" << endl;
cout << "|  '-.______.-' /       \\/       \\  '-._____.-'   |" << endl;
cout << "|               |       ||       |                |" << endl;
cout << "|              /\\       /\\       /\\               |" << endl;
cout << "|             /  '.___.'  '.___.'  \\              |" << endl;
cout << "|            /                      \\             |" << endl;
cout << " \\          /                        \\           /" << endl;
cout << "  \\________/                          \\_________/" << "\n" << endl;
	 
	cout << "Line Length:\t" << LINE_LENGTH << endl;
	cout << "Border Size (L/R):\t" << BORDER << endl;
	cout << "Border Size (Top):\t" << TOP_BORDER << endl;
	cout << "Distance Threshold:\t" << threshold << endl;
	if (l_to_r) { cout << "Direction: LEFT TO RIGHT" << endl; }
	if (!l_to_r) { cout << "Direction: RIGHT TO LEFT" << endl; }
	cout << "Colour(RGB):\t" << rT%3 << "\t" << gT%3 << "\t" << bT%3 << endl;
	cout << "Dilation Filter:\t" << DILATION << endl;
}

void median_filter (int i, int j) {
	median[0] = imP[i][j];	
	median[1] = imP[i+1][j];	
	median[2] = imP[i+1][j+1];
	median[3] = imP[i][j+1];
	median[4] = imP[i-1][j+1];
	median[5] = imP[i-1][j];
	median[6] = imP[i-1][j-1];
	median[7] = imP[i][j-1];
	median[8] = imP[i+1][j-1];
	
	for (int k = 1; k < 9; ++k) { //insertion sort
        int key = median[k];
        int l = k-1;
        while (l >= 0 && median[l] > key) {
            median[l+1] = median[l];
            l = l-1;
        }
        median[l+1] = key;
    }	
	imG[i][j] = median[4];
}

void dilation_filter (int i, int j) {
	if(im[i][j] == 1) { // apply dilation
		for(int m = 0; m <= DILATION * DILATION; m++) {
			int k = m % DILATION;
			int l = m / DILATION;
			if(i+k >= 0 && i+k < WIDTH && j+l >= 0 && j+l < HEIGHT) {
				imP[i+k][j+l] = 1;
			}
		}
	}	
}

void gaussian_blur(int i, int j) {
	float g[3][3] = {{1,2,1},{2,4,2},{1,2,1}};
	
	im[i][j] = (g[0][0]*imG[i-1][j-1] + g[1][0]*imG[i][j-1] + g[2][0]*imG[i+1][j-1]
			  + g[0][1]*imG[i-1][j]   + g[1][1]*imG[i][j]   + g[2][1]*imG[i+1][j]
		 	  + g[0][2]*imG[i-1][j+1] + g[1][2]*imG[i][j+1] + g[2][2]*imG[i+1][j+1])/16;
}

void sobel_filter(int i, int j) {
	float g[3] = {1,2,1};
	
	float gxIm = g[0]*im[i-1][j-1] - g[0]*im[i+1][j-1]
		 + g[1]*im[i-1][j]   - g[1]*im[i+1][j]
		 + g[2]*im[i-1][j+1] - g[2]*im[i+1][j+1];

	float gyIm = g[0]*im[i-1][j-1] + g[1]*im[i][j-1] + g[2]*im[i+1][j-1]
		 - g[0]*im[i-1][j+1] - g[1]*im[i][j+1] - g[2]*im[i+1][j+1];

	sbl[i][j] = sqrt(gxIm*gxIm + gyIm*gyIm);
	ang[i][j] = atan2(gyIm, gxIm)+M_PI/2;
}

void GetColourType (int t, float& colour) {
	switch (t % 3) {
		case 0:
			colour = 0;
			break;
		case 1:
			colour = 1;
			break;
		default:
			colour = c;
			break;
	}
}

//---Main Program ------
int main()
{
	GLFWwindow* win;
	
    //-----Initialize the library----
    if (!glfwInit()) return -1;
	
    win = glfwCreateWindow(WIDTH, HEIGHT, "DEPTH DANCE", NULL, NULL);
    if (!win) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(win);	
	glClearColor(0.0, 0.0, 0.0, 0.0);
	
	//----Read camera-----
	rs2::config cfg;
	rs2::pipeline pipe;
	rs2::frameset frames;
	
    cfg.enable_stream(STREAM, WIDTH, HEIGHT, FORMAT, 15);
	pipe.start(cfg);
	
	cout << "Press START" << endl; 
	
    while (!glfwWindowShouldClose(win))
    {
		glfwPollEvents();
        frames = pipe.wait_for_frames();
		rs2::depth_frame depth = frames.get_depth_frame();
		
		//----Setup window----
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
		glClear(GL_COLOR_BUFFER_BIT);
		
		glPointSize(1.0f);		
		glPushMatrix();
		glfwGetWindowSize(win, &w, &h);
        glOrtho(0, w, h, 0, -1, +1);
		glBegin(GL_POINTS);
		
		//-----Key Presses-----
		int state = glfwGetKey(win, GLFW_KEY_H);
		if (state == GLFW_PRESS) { 
			hold = !hold; 
			if (hold) { cout << "HOLD" << endl; }
			else { printout(); }
		}
		if (!hold) {
			state = glfwGetKey(win, GLFW_KEY_RIGHT);
			if (state == GLFW_PRESS) { 
				TOP_BORDER++; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_LEFT);
			if (state == GLFW_PRESS) { 
				TOP_BORDER--; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_UP);
			if (state == GLFW_PRESS) { 
				LINE_LENGTH++; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_DOWN);
			if (state == GLFW_PRESS) { 
				LINE_LENGTH--; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_R);
			if (state == GLFW_PRESS) { 
				rT++; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_G);
			if (state == GLFW_PRESS) { 
				gT++;
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_B);
			if (state == GLFW_PRESS) { 
				bT++;
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_EQUAL);
			if (state == GLFW_PRESS) { 
				threshold += 0.001*65.535; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_MINUS);
			if (state == GLFW_PRESS) { 
				threshold -= 0.001*65.535; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_W);
			if (state == GLFW_PRESS) { 
				l_to_r = !l_to_r; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_D);
			if (state == GLFW_PRESS) { 
				BORDER++; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_A);
			if (state == GLFW_PRESS) { 
				BORDER--; 
				printout();
			}
			state = glfwGetKey(win, GLFW_KEY_F);
			if (state == GLFW_PRESS) { 
				DILATION++; 
				if (DILATION > 3) {
					DILATION = 1;
				}
				printout();
			}
		}
		
		//-----Threshold Data-----
		for (int j = 1; j < HEIGHT-1; j++) {
			for (int i = 1; i < WIDTH-1; i++) {
				if (depth.get_distance(i,j) < threshold) 	{ c = 0; }
				else { c = 1; }
				imG[i][j]  = c;
				imP[i][j]  = c;
				im[i][j]  = c;
			}
		}
			
		//----Dilation Filter-----
		for (int n = 0; n < WIDTH * HEIGHT; n++) {
			int j = n / WIDTH + DILATION;
			int i = n % WIDTH + DILATION;
			dilation_filter(i,j);
		}
		
		//-----Median Filter-----
		for (int n = 0; n < (WIDTH-1) * (HEIGHT-1); n++) {
			int j = (n / (WIDTH-1)) + 1;
			int i = (n % (WIDTH-1)) + 1;
			median_filter(i,j);
		}
		
		//-----Gaussian Filter-----
		for (int k = 0; k < (WIDTH-2) * (HEIGHT-2); k++) {
			int j = (k / (WIDTH-2)) + 1;
			int i = (k % (WIDTH-2)) + 1;
			gaussian_blur(i,j);
		}
		
		//----Setup DAC-----
		HeliosTranslate(0, 0, 0); 
		HeliosScale(-1,1,0);
		HeliosRotate(0);
		
		//-----Sobel Filter-----
		#define MAX		2.75f
		for (int k = 0; k < (WIDTH-2) * (HEIGHT-2); k++) {
			int j = (k / (WIDTH-2)) + 1;
			int i = (k % (WIDTH-2)) + 1;
			sobel_filter(i,j);
			if (sbl[i][j] > MAX) {
				c = ang[i][j] / 2*M_PI;
				SetHeliosColour(0.5f, 0.5f, 0.5f);
				glVertex2f(i,j);
			}
		}	
		glEnd();
		
		glLineWidth(3.0f);
		//-----Find Lines-------
		glBegin(GL_LINE_STRIP);
		float max = MAX;
		int start, end;
		if (l_to_r) { 
			start = BORDER;
			end = WIDTH-BORDER;
		}
		else { 
			start = WIDTH-BORDER;
			end = BORDER;
		}		
		tx = start;
		
		for (int k = HEIGHT; k >= TOP_BORDER; k--) {
			if (sbl[tx][k] > max) {
				max = sbl[tx][k];
				ty = k;	
			}
			sbl[tx][ty] = 0;
		}
		WriteHeliosBlank(tx, ty);
		
		int k = 0;
		while(k < MAX_POINTS-2 && tx >= BORDER && tx <= WIDTH-BORDER && ty >= TOP_BORDER) {
			bool isChanged = false;
			float adjMax = MAX;
			int adjTx; 
			int adjTy;
			if (l_to_r) {
				adjTx = tx - round(cos(ang[tx][ty]));			
			 	adjTy = ty - round(sin(ang[tx][ty]));
			} else {
				adjTx = tx + round(cos(ang[tx][ty]));			
			 	adjTy = ty + round(sin(ang[tx][ty]));
			} 
			
			for(int i=-LINE_LENGTH; i<=LINE_LENGTH; i++){
				for(int j=-LINE_LENGTH; j<=LINE_LENGTH; j++){
					int adjX = tx + i;
					int adjY = ty + j;
					if(adjX >= BORDER && adjX < WIDTH-BORDER && adjY >= 0 && adjY < HEIGHT && sbl[adjX][adjY] > adjMax){
						adjMax = sbl[adjX][adjY];
						adjTx = adjX;
						adjTy = adjY;
						sbl[adjX][adjY] = 0;
						isChanged = true;
					}
					if ((i > -LINE_LENGTH/2 && i < LINE_LENGTH/2) || (j > -LINE_LENGTH/2 && j < LINE_LENGTH/2)) {
						sbl[adjX][adjY] = 0;
					}
				}
			}
			
			tx = adjTx;
			ty = adjTy;
			max = adjMax;
			sbl[tx][ty] = 0;
			if (isChanged) {
				c = ang[tx][ty] / 2*M_PI;	
				GetColourType(rT, rN);
				GetColourType(gT, gN);
				GetColourType(bT, bN);			
				SetHeliosColour(rN, gN, bN);
				WriteHeliosPoint(tx, ty);
			}
			k++;
		}
		
		glEnd();
		glPopMatrix();
		glfwSwapBuffers(win);
		WriteHeliosFrame();
		
    }
	glfwDestroyWindow(win);
	glfwTerminate();
	CloseHeliosDAC();
    return 0;
}