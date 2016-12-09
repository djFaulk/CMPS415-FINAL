#include <stdlib.h>
#include <stdio.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <gmtl/gmtl.h> 
#include <stdlib.h>
#include <stdio.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Texture.cpp"

#pragma comment (lib, "opengl32.lib")
#pragma comment (lib, "glew32.lib")
#pragma comment (lib, "glfw3.lib")

//#define FULLSCREEN_MODE

//Class decalaration
class Bird {

	gmtl::Matrix44f rotation;
	float omega;
	float vertVel;
	float radius;

public:
	Bird()
	{
		gmtl::Matrix44f temp;
		gmtl::identity(temp);

		rotation = temp;
		omega = 1;
		vertVel = 1;
		radius = 1.6;
	}
	Bird(gmtl::Matrix44f rot)
	{
		rotation = rot;
		omega = 1;
		vertVel = 1.5;
		radius = 1.6;
	}


	Bird(gmtl::Matrix44f rot, float om)
	{
		rotation = rot;
		omega = om;
		vertVel = 1;
		radius = 1.6;
	}

	Bird(float Vv, bool Vel)
	{
		gmtl::Matrix44f temp;
		gmtl::identity(temp);

		rotation = temp;
		if (Vel)
		{
			vertVel = Vv;
			omega = 1;
		}
		else
		{
			vertVel = 1;
			omega = Vv;
		}

		radius = 1.6;
	}

	//Getters and Setters

	gmtl::Matrix44f getRot()
	{
		return rotation;
	}

	float getRot(int row, int column)
	{
		return rotation[row][column];
	}

	void setRot(gmtl::Matrix44f rot)
	{
		rotation = rot;
	}

	void setRot(int row, int column, float n)
	{
		rotation[row][column] = n;
	}

	float getOmega()
	{
		return omega;
	}

	void setOmega(float om)
	{
		omega = om;
	}

	float getVertVel()
	{
		return vertVel;
	}

	void setVertVel(float Vv)
	{
		vertVel = Vv;
	}

	float getRadius()
	{
		return radius;
	}

	void setRadius(float rad)
	{
		radius = rad;
	}
};

//Global Variables NP, NM, and Radius
const int np = 21;
const int nm = 20;

//int representing number of birds
const int nb = 6;
const int no = 4;
int num = 0;
float meanAccMag, dispAccMag, centAccMag, vmatchAccMag, totalAccMag, meanDispMag, meanCentMag, meanVmatchMag, iterations = 0;
float maxAccMag, maxDispMag, maxCentMag, maxVmatchMag = -100;
float maxAccel = 0.3;
float maxBanking = -100;

const float cRadius = 0.01;
const int height = 10;
const int cnm = 5;

gmtl::Matrix44f M;

//W - World 
gmtl::Matrix44f W;

//R - Point of Rotation
gmtl::Matrix44f R;

//O - Offset from point of rotation
gmtl::Matrix44f O;

//B - Base of the Bird object
gmtl::Matrix44f B;

//j1 - Offset of Bird for Joint 1
gmtl::Matrix44f joint1;

//j2 - Offset of Bird for Joint 2
gmtl::Matrix44f joint2;

//prim1 - Offset for Primitive from Joint 1
gmtl::Matrix44f prim1;

//prim2 - Offset for Primitive from Joint 2
gmtl::Matrix44f prim2;

//cam1 - Position for Camera 1
gmtl::Matrix44f cam1;

//cam2 - Position for Camera 2
gmtl::Matrix44f cam2;

//activeCam - bool describing active camera. 0(false) for camera 1, 1(true) for camera 2
bool activeCam = 0;

//Q - {R} w.r.t. {W}
gmtl::Matrix44f WtR;

//q - Quaternion for {R} w.r.t. {W}
gmtl::Quatf q (0, 0, 0, 1);

gmtl::Matrix44f Q;

gmtl::Quatf move_bird_forward( 0, 0, 0, 1);

gmtl::Quatf turn_bird_right( 0, 0, 0, 1);

gmtl::Quatf turn_bird_left( 0, 0, 0, 1);

//RtO - {O} w.r.t. {R}
gmtl::Matrix44f RtO;

//OtB - {B} w.r.t. {O}
gmtl::Matrix44f OtB;

//BtJ1 - {J1}, or joint 1, w.r.t. {B}
gmtl::Matrix44f BtJ1;

//BtJ2 - {J2} w.r.t. {B}
gmtl::Matrix44f BtJ2;

//J1tP - {J1} to Primitive
gmtl::Matrix44f J1tP;

//J2tP - {J2} to Primitive
gmtl::Matrix44f J2tP;

//WtC1 - relation from {W} to Camera 1
gmtl::Matrix44f WtC1;
 
//OtC2 - relation from {O} to Camera 2
gmtl::Matrix44f OtC2;

gmtl::Matrix44f V;

gmtl::Matrix44f camChange;

gmtl::Matrix44f R_lat;

gmtl::Matrix44f R_long;

gmtl::Matrix44f R_azi;

gmtl::Matrix44f R_ele;

gmtl::Matrix44f R_z;

//Z-Translation for Camera 1
gmtl::Matrix44f T1;

//Z-Translation for Camera 2
gmtl::Matrix44f T2;

//toggle for IH and ID
bool IH, ID = true;

gmtl::Matrix44f Wy;
gmtl::Matrix44f Wx;
gmtl::Matrix44f Wz;
bool showW = false;

gmtl::Matrix44f Ry;
gmtl::Matrix44f Rx;
gmtl::Matrix44f Rz;
bool showR = false;

gmtl::Matrix44f Oy;
gmtl::Matrix44f Ox;
gmtl::Matrix44f Oz;
bool showO = false;

bool toggleMouse = true;

//Mouse Call objects
double x_Beg = 0.0;
double y_Beg = 0.0;
double x_End = 0.0;
double y_End = 0.0;
double x_Diff = 0.0;
double y_Diff = 0.0;
double x_Last = 0.0;
double y_Last = 0.0;

bool captureMouse = false;

float toRadians(float angle);

GLfloat * setColor(int type);

Bird bird1;
Bird bird2;
Bird bird3;
Bird bird4;

Bird * birdArray = new Bird[nb];

//Vector arrays for keeping track of forces on Bird
gmtl::Vec4f * BirdDisp = new gmtl::Vec4f[nb];
gmtl::Vec4f * BirdCenter = new gmtl::Vec4f[nb];
gmtl::Vec4f * BirdVelMatch = new gmtl::Vec4f[nb];

bool showDisp, showCenter, showMatch = false;
bool pause = false;

gmtl::Matrix44f * obstacleList = new gmtl::Matrix44f[no];

gmtl::Matrix44f * birdOtB = new gmtl::Matrix44f[nb];

gmtl::Matrix44f Wreset;
bool wSet = false;

float maxAccelMagAccum = -1.0f;
float avgAccelMagAccum;
float totalAccelMagAccum = 0.0f;

float distAway1 = 1.75;
bool distChange = true;

//gmtl::Vec4f * birdVelocity;// = new gmtl::Vec4f[nb];
//gmtl::Vec4f * birdAccel;// = new gmtl::Vec4f[nb];
//gmtl::Vec4f * birdForce;// = new gmtl::Vec4f[nb];

/* A simple function that will read a file into an allocated char pointer buffer */
char* filetobuf(char *file)
{
	FILE *fptr;
	long length;
	char *buf;

	fopen_s(&fptr, file, "rb"); /* Open file for reading */
	if (!fptr) /* Return NULL on failure */
		return NULL;
	fseek(fptr, 0, SEEK_END); /* Seek to the end of the file */
	length = ftell(fptr); /* Find out how many bytes into the file we are */
	buf = (char*)malloc(length + 1); /* Allocate a buffer for the entire length of the file and a null terminator */
	fseek(fptr, 0, SEEK_SET); /* Go back to the beginning of the file */
	fread(buf, length, 1, fptr); /* Read the contents of the file in to the buffer */
	fclose(fptr); /* Close the file */
	buf[length] = 0; /* Null terminator */

	return buf; /* Return the buffer */
}

void error_callback(int error, const char* description)
{
	fprintf(stderr, "Error: %s\n", description);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	gmtl::Matrix44f change;
	gmtl::identity(change);

	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);

	//Move the bird forward by incrementing Q
	else if (key == GLFW_KEY_W && action == GLFW_PRESS)
	{
		//q = q * move_bird_forward;
		
		showDisp = !showDisp;
		showCenter = !showCenter;
		showMatch = !showMatch;

		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);

		/*change[0][0] = cos(toRadians(10));
		change[0][1] = sin(toRadians(10)) * -1;
		change[1][0] = sin(toRadians(10));
		change[1][1] = cos(toRadians(10));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		WtR = WtR * change;*/
	}
	//Allow the bird to Yaw by small x-rotation
	else if ( key == GLFW_KEY_D && action == GLFW_PRESS ) 
	{
		showDisp = !showDisp;
		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);
		/*change[1][1] = cos(toRadians(22));
		change[1][2] = sin(toRadians(22)) * -1;
		change[2][1] = sin(toRadians(22));
		change[2][2] = cos(toRadians(22));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		WtR = WtR * change;*/

		//q = q * turn_bird_right;
	}
	else if (key == GLFW_KEY_S && action  == GLFW_PRESS)
	{
		showMatch = !showMatch;
		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);

	}
	else if (key == GLFW_KEY_A && action == GLFW_PRESS )
	{
		showCenter = !showCenter;
		printf("ShowDisp: %d\nShowCenter: %d\nShowMatch: %d\n\n", showDisp, showCenter, showMatch);

		/*change[1][1] = cos(toRadians(-22));
		change[1][2] = sin(toRadians(-22)) * -1;
		change[2][1] = sin(toRadians(-22));
		change[2][2] = cos(toRadians(-22));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);
		
		WtR = WtR * change;*/
	
		//q = q * turn_bird_left;
	}
	//Q - positive Banking around {O}'s Y-axis
	else if (key == GLFW_KEY_Q && GLFW_PRESS)
	{
		
		change[0][0] = cos(toRadians(15));
		change[0][2] = sin(toRadians(15)) * -1;
		change[2][0] = sin(toRadians(15));
		change[2][2] = cos(toRadians(15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);
		
		OtB = OtB * change;
	}
	//E - negative banking around {O}'s Y-axis
	else if (key == GLFW_KEY_E && GLFW_PRESS)
	{
		change[0][0] = cos(toRadians(-15));
		change[0][2] = sin(toRadians(-15)) * -1;
		change[2][0] = sin(toRadians(-15));
		change[2][2] = cos(toRadians(-15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		OtB = OtB * change;
	}
	//Z - Joint 1 rotation
	else if (key == GLFW_KEY_Z && GLFW_PRESS)
	{
		change[1][1] = cos(toRadians(15));
		change[1][2] = sin(toRadians(15)) * -1;
		change[2][1] = sin(toRadians(15));
		change[2][2] = cos(toRadians(15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);
		
		BtJ1 = BtJ1 * change;
	}
	//C - Joint 2 rotation
	else if (key == GLFW_KEY_C && GLFW_PRESS)
	{
		change[1][1] = cos(toRadians(15));
		change[1][2] = sin(toRadians(15)) * -1;
		change[2][1] = sin(toRadians(15));
		change[2][2] = cos(toRadians(15));
		change.setState(gmtl::Matrix44f::ORTHOGONAL);

		BtJ2 = BtJ2 * change;
	}
	//F - Switch Active Camera
	else if (key == GLFW_KEY_F && action == GLFW_PRESS)
	{
		activeCam = !activeCam;
		
		if(!activeCam)W = Wreset;
		
	}
	//1 - Activate World Coord for W
	else if (key == GLFW_KEY_1 && action == GLFW_PRESS)
	{
		showW = !showW;
	}
	//2 - Activate World Coord for R
	else if (key == GLFW_KEY_2 && action == GLFW_PRESS)
	{
		showR = !showR;
	}
	//3 - Activate World Coord for O
	else if (key == GLFW_KEY_3 && action == GLFW_PRESS)
	{
		showO = !showO;
	}
	//L - Toggle Mouse options
	else if (key == GLFW_KEY_L && action == GLFW_PRESS)
	{
		toggleMouse = !toggleMouse;
	}
	//Arrow key up - Adjust T to zoom in
	else if (key == GLFW_KEY_UP && action == GLFW_PRESS)
	{

		distChange = true;
		distAway1 -= 0.15;
		//if (!activeCam)
		//{
		//	/*change[2][3] = -0.15;
		//	change.setState(gmtl::Matrix44f::TRANS);

		//	T1 = T1 * change;*/

		//	T1[2][3] -= 0.15;
		//	T1.setState(gmtl::Matrix44f::TRANS);
		//}
		//else
		//{
		//	T2[2][3] -= .15;
		//	T2.setState(gmtl::Matrix44f::TRANS);
		//}
	}
	//Arrow key down - Adjust T to zoom out
	else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
	{
		distAway1 += 0.15;
		distChange = true;
		//if (!activeCam)
		//{
		//	/*change[2][3] = 0.15;
		//	change.setState(gmtl::Matrix44f::TRANS);

		//	T1 = T1 * change;*/

		//	T1[2][3] += .15;
		//	T1.setState(gmtl::Matrix44f::TRANS);
		//}
		//else
		//{
		//	T2[2][3] += .15;
		//	T2.setState(gmtl::Matrix44f::TRANS);
		//}
	}
	else if (GLFW_KEY_SPACE && action == GLFW_PRESS)
	{
		pause = !pause;
	}
	/*else if (key == GLFW_KEY_I && action == GLFW_PRESS)
	{
		ID = !ID;
	}
	else if (key == GLFW_KEY_U && action == GLFW_PRESS)
	{
		IH = !IH;
	}*/
}

//Called when mouse is pressed. Format taken from example 1b provided for Assignment 1
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && toggleMouse) 
	{
		captureMouse = true;
		//if(x_Last == 0.0 && y_Last == 0.0)
		glfwGetCursorPos(window, &x_Beg, &y_Beg);
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		captureMouse = false;
	}
}

/** This function sets up our window, OpenGL context, etc. For assignments, you don't need to know how it works. */
GLFWwindow* setupWindow()
{
	GLFWwindow *window;
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	//glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); //Update later
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	//variable to get current video mode
	const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

#ifdef FULLSCREEN_MODE
	window = glfwCreateWindow(mode->width, mode->height, "Fullscreen example", glfwGetPrimaryMonitor(), NULL);
#else
	window = glfwCreateWindow(1024, 576, "CMPS 415", NULL, NULL);
#endif
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwSetKeyCallback(window, key_callback);
	glfwMakeContextCurrent(window);
	//gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
	glfwSwapInterval(1);

	if (glewInit() != GLEW_OK)
		exit(EXIT_FAILURE);

	glEnable(GL_DEPTH_TEST);

	return window;
}

/** This function sets up shaders on the graphics card. For assignments, you don't need to know how it works. */
GLuint setupShaderProgram()
{
	GLuint vertex_shader, fragment_shader, shader_program;
	int IsCompiled_VS, IsCompiled_FS, IsLinked, max_length;
	char *vertex_shader_log;
	char *fragment_shader_log;
	char *shader_program_log;

	/* Read our shaders into the appropriate buffers */
	char* vertex_source = filetobuf("OpenGL_Example.vert");
	char* fragment_source = filetobuf("OpenGL_Example.frag");

	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader, 1, &vertex_source, NULL);
	glCompileShader(vertex_shader);

	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &IsCompiled_VS);
	if (IsCompiled_VS == GL_FALSE)
	{
		glGetShaderiv(vertex_shader, GL_INFO_LOG_LENGTH, &max_length);

		/* The max_length includes the NULL character */
		vertex_shader_log = (char *)malloc(max_length);

		glGetShaderInfoLog(vertex_shader, max_length, &max_length, vertex_shader_log);
		printf("Error: %s", vertex_shader_log);
		/* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
		/* In this simple program, we'll just leave */
		free(vertex_shader_log);
		free(vertex_source);
		return 0;
	}

	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment_source, NULL);
	glCompileShader(fragment_shader);
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &IsCompiled_FS);
	if (IsCompiled_FS == GL_FALSE)
	{
		glGetShaderiv(fragment_shader, GL_INFO_LOG_LENGTH, &max_length);

		/* The max_length includes the NULL character */
		fragment_shader_log = (char *)malloc(max_length);

		glGetShaderInfoLog(fragment_shader, max_length, &max_length, fragment_shader_log);
		printf("Error: %s", fragment_shader_log);
		/* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
		/* In this simple program, we'll just leave */
		free(fragment_shader_log);
		free(vertex_source);
		free(fragment_source);
		return 0;
	}

	/* If we reached this point it means the vertex and fragment shaders compiled and are syntax error free. */
	/* We must link them together to make a GL shader program */
	/* GL shader programs are monolithic. It is a single piece made of 1 vertex shader and 1 fragment shader. */
	/* Assign our program handle a "name" */
	shader_program = glCreateProgram();

	/* Attach our shaders to our program */
	glAttachShader(shader_program, vertex_shader);
	glAttachShader(shader_program, fragment_shader);

	/* Link our program */
	/* At this stage, the vertex and fragment programs are inspected, optimized and a binary code is generated for the shader. */
	/* The binary code is uploaded to the GPU, if there is no error. */
	glLinkProgram(shader_program);

	/* Again, we must check and make sure that it linked. If it fails, it would mean either there is a mismatch between the vertex */
	/* and fragment shaders. It might be that you have surpassed your GPU's abilities. Perhaps too many ALU operations or */
	/* too many texel fetch instructions or too many interpolators or dynamic loops. */

	glGetProgramiv(shader_program, GL_LINK_STATUS, (int *)&IsLinked);
	if (IsLinked == GL_FALSE)
	{
		/* Noticed that glGetProgramiv is used to get the length for a shader program, not glGetShaderiv. */
		glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &max_length);

		/* The max_length includes the NULL character */
		shader_program_log = (char *)malloc(max_length);

		/* Notice that glGetProgramInfoLog, not glGetShaderInfoLog. */
		glGetProgramInfoLog(shader_program, max_length, &max_length, shader_program_log);
		printf("Error: %s", shader_program_log);
		/* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
		/* In this simple program, we'll just leave */
		free(shader_program_log);
		free(vertex_source);
		free(fragment_source);
		return 0;
	}
	//	glBindAttribLocation(shader_program, SHADER_POSITION_INDEX, "in_position");
	//	glBindAttribLocation(shader_program, SHADER_COLOR_INDEX, "in_color");

	free(vertex_source);
	free(fragment_source);

	return shader_program;

}

/**
This function gets attribute and uniform locations from shaders. For _this_ assignment (A2),
you don't need to know how it works.
*/
void init(GLuint shader_program, GLuint &pos_loc_out, GLuint &color_loc_out, GLuint &vm_loc_out, GLuint &pvm_loc_out, GLuint &norm_loc_out, GLuint &light_pos_out, GLuint &WtE_loc_out, GLuint &Od_loc_out, GLuint &UV_loc_out, GLuint textureID[])//, GLuint &norm_Mat_loc_out
{
	pos_loc_out = glGetAttribLocation(shader_program, "in_position");
	color_loc_out = glGetAttribLocation(shader_program, "in_color");
	norm_loc_out = glGetAttribLocation(shader_program, "in_normal");
	light_pos_out = glGetAttribLocation(shader_program, "light_pos");
	WtE_loc_out = glGetAttribLocation(shader_program, "in_WtE");
	Od_loc_out = glGetAttribLocation(shader_program, "in_Od");
	UV_loc_out = glGetAttribLocation(shader_program, "vertexUV");

	//vm_loc_out = glGetUniformLocation(shader_program, "VM");
	vm_loc_out = glGetUniformLocation(shader_program, "M");
	pvm_loc_out = glGetUniformLocation(shader_program, "PVM");
	//norm_Mat_loc_out = glGetUniformLocation(shader_program, "normalMatrix");

	unsigned int texwidth, texheight;
	unsigned char *imagedata;
	GLuint texture_location;

	glGenTextures(2, textureID);

	LoadPPM("space1.ppm", &texwidth, &texheight, &imagedata, 1);

	texture_location = glGetUniformLocation(shader_program, "texture_Colors");
	glUniform1i(textureID[0], 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texheight, texwidth, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);


	texture_location = glGetUniformLocation(shader_program, "texture_Colors");
	LoadPPM("moon.ppm", &texwidth, &texheight, &imagedata, 1);
	glUniform1i(textureID[1], 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texheight, texwidth, 0, GL_RGB, GL_UNSIGNED_BYTE, imagedata);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);


}

void initMatrices()
{
	gmtl::identity(V);
	V.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(M);
	M.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(WtR);
	WtR.setState(gmtl::Matrix44f::AFFINE);

	//RtO is the relation from {R} to {O} which is merely an X-transform of the Radius + Hovering Distance
	//This means we can simply set the matrix here and we dont need to adjust it again
	gmtl::identity(RtO);
	RtO[0][3] = 1.6;
	RtO.setState(gmtl::Matrix44f::TRANS);

	gmtl::identity(OtB);
	OtB.setState(gmtl::Matrix44f::ORTHOGONAL);

	gmtl::identity(W);
	W.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(B);
	B.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(joint1);
	joint1.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(joint2);
	joint2.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(BtJ1);
	BtJ1[0][3] = 0.07;
	BtJ1[1][3] = -0.07;
	BtJ1[2][3] = -0.07;
	BtJ1.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(BtJ2);
	BtJ2[0][3] = 0.07;
	BtJ2[1][3] = -0.07;
	BtJ2[2][3] = 0.07;
	BtJ2.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(J1tP);
	J1tP[1][3] = 0.05;
	J1tP.setState(gmtl::Matrix44f::TRANS);

	gmtl::identity(J2tP);
	J2tP[1][3] = 0.05;
	J1tP.setState(gmtl::Matrix44f::TRANS);

	gmtl::identity(WtC1);
	WtC1.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(camChange);
	camChange.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_lat);
	R_lat.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_long);
	R_long.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_azi);
	R_azi.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_ele);
	R_ele.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(R_z);
	R_z[0][0] = cos(toRadians(-90));
	R_z[0][1] = sin(toRadians(-90)) * -1;
	R_z[1][0] = sin(toRadians(-90));
	R_z[1][1] = cos(toRadians(-90));
	R_z.setState(gmtl::Matrix44f::ORTHOGONAL);

	gmtl::identity(Wx);
	gmtl::identity(Wy);
	gmtl::identity(Wz);
	Wx.setState(gmtl::Matrix44f::AFFINE);
	Wy.setState(gmtl::Matrix44f::AFFINE);
	Wz.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(Rx);
	gmtl::identity(Ry);
	gmtl::identity(Rz);
	Rx.setState(gmtl::Matrix44f::AFFINE);
	Ry.setState(gmtl::Matrix44f::AFFINE);
	Rz.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(Ox);
	gmtl::identity(Oy);
	gmtl::identity(Oz);
	Ox.setState(gmtl::Matrix44f::AFFINE);
	Oy.setState(gmtl::Matrix44f::AFFINE);
	Oz.setState(gmtl::Matrix44f::AFFINE);

	gmtl::identity(T1);
	T1[2][3] = 1.75;
	T1.setState(gmtl::Matrix44f::TRANS);

	gmtl::identity(T2);
	T2[2][3] = 1.75;
	T2.setState(gmtl::Matrix44f::TRANS);
	
	gmtl::identity(Wreset);
}

void initBird(bool print)
{
	gmtl::Matrix44f change;
	gmtl::identity(change);
	change[0][0] = cos(toRadians(30));
	change[0][1] = sin(toRadians(30)) * -1;
	change[1][0] = sin(toRadians(30));
	change[1][1] = cos(toRadians(30));
	change.setState(gmtl::Matrix44f::ORTHOGONAL);

	bird2.setRot(bird2.getRot() * change * change);

	gmtl::identity(change);
	change[0][0] = cos(toRadians(40));
	change[0][2] = sin(toRadians(40)) * -1;
	change[2][0] = sin(toRadians(40));
	change[2][2] = cos(toRadians(40));
	change.setState(gmtl::Matrix44f::ORTHOGONAL);
	bird3.setRot(bird3.getRot() * change * change);


	gmtl::identity(change);
	change[0][0] = cos(toRadians(180));
	change[0][2] = sin(toRadians(180)) * -1;
	change[2][0] = sin(toRadians(180));
	change[2][2] = cos(toRadians(180));
	change.setState(gmtl::Matrix44f::ORTHOGONAL);
	bird4.setRot(bird4.getRot() * change);


	bird1.setOmega(1);
	bird2.setOmega(2);
	bird3.setOmega(3);
	bird4.setOmega(2.5);

	Bird bird5;
	Bird bird6;

	birdArray[0] = bird1;
	birdArray[1] = bird2;
	birdArray[2] = bird3;
	birdArray[3] = bird4;
	birdArray[4] = bird5;
	birdArray[5] = bird6;


	if (print) {
		for (int b = 0; b < nb; b++)
		{
		for (int o = 0; o < 4; o++)
		{
		for (int i = 0; i < 4; i++)
		{
		printf("%f\t", birdArray[b].getRot(o, i));
		}

		printf("\n");

		}
		printf("\n\n");
		}
	}
}

void initObstacles()
{
	gmtl::Matrix44f ob1;
	gmtl::identity(ob1);

	gmtl::Matrix44f ob2;
	gmtl::identity(ob2);

	ob2[0][0] = cos(toRadians(180));
	ob2[0][2] = sin(toRadians(180)) * -1;
	ob2[2][0] = sin(toRadians(180));
	ob2[2][2] = cos(toRadians(180));

	gmtl::Matrix44f ob3;
	gmtl::identity(ob3);

	ob3[0][0] = cos(toRadians(90));
	ob3[0][2] = sin(toRadians(90)) * -1;
	ob3[2][0] = sin(toRadians(90));
	ob3[2][2] = cos(toRadians(90));

	gmtl::Matrix44f change;
	gmtl::identity(change);

	change[0][0] = cos(toRadians(45));
	change[0][1] = sin(toRadians(45)) * -1;
	change[1][0] = sin(toRadians(45));
	change[1][1] = cos(toRadians(45));

	ob3 = ob3 * change;

	gmtl::Matrix44f ob4;
	gmtl::identity(ob4);

	ob4[0][0] = cos(toRadians(90));
	ob4[0][2] = sin(toRadians(90)) * -1;
	ob4[2][0] = sin(toRadians(90));
	ob4[2][2] = cos(toRadians(90));

	gmtl::identity(change);

	change[0][0] = cos(toRadians(-45));
	change[0][1] = sin(toRadians(-45)) * -1;
	change[1][0] = sin(toRadians(-45));
	change[1][1] = cos(toRadians(-45));

	ob4 = ob4 * change;

	obstacleList[0] = ob1;
	obstacleList[1] = ob2;
	obstacleList[2] = ob3;
	obstacleList[3] = ob4;
}

float toRadians(float angle)
{
	return angle * (M_PI / 180);
}

GLfloat * generateVertexList(float radius, float zRestrict, float xRestrict)
{
	//loop variables should be integers, so we should use degrees
	//We subtract 1 from np and nm + 1 so that we avoid an extra loop

	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = np * (nm + 1);
	static GLfloat * verticesList = new GLfloat[3 * numVert];

	int p = 0;

	for (int o = 0; o <= 180; o += Ostep)
	{
		r = sin(toRadians(o));

		y = cos(toRadians(o)) * -1 * radius;

		for (int i = 0; i <= 360; i += Istep)
		{

			x = r * cos(toRadians(i)) * radius * xRestrict;
			z = r * sin(toRadians(i)) * -1 * radius * zRestrict;

			verticesList[p] = x;
			p++;

			verticesList[p] = y;
			p++;

			verticesList[p] = z;
			p++;
		}
		//printf("p\t:\t%i\n", p);
	}
	//printf("numvert:\t%i\n", 3 * numVert);
	//printf("P:\t%i\n", p);
	return verticesList;

}

GLfloat * generateTrenchVertexList(float radius)
{
	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = ((np + 6) * (nm + 1));
	
	//int numVert = np * (nm + 1);
	
	static GLfloat * verticesList = new GLfloat[3 * numVert];
	int oprint;
	int p = 0;
	//Bottom Half of Sphere
	for (int o = 0; o <= 80; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius;
			z = r * sin(toRadians(i)) * -1 * radius;

			verticesList[p] = x;
			p++;
			verticesList[p] = y;
			p++;
			verticesList[p] = z;
			p++;
		}

		oprint = o;
	}

	//printf("O after first loop\t:\t%i\n", oprint);
	//Outer Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius;
		z = r * sin(toRadians(i)) * -1 * radius;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Inner Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius * 0.75;
		z = r * sin(toRadians(i)) * -1 * radius * 0.75;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Cylinder Middle
	for (int o = oprint; o <= 100; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius * 0.75;
			z = r * sin(toRadians(i)) * -1 * radius * 0.75;

			verticesList[p] = x;
			p++;
			verticesList[p] = y;
			p++;
			verticesList[p] = z;
			p++;
		}

		oprint = o;
	}

	//Inner Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius * 0.75;
		z = r * sin(toRadians(i)) * -1 * radius * 0.75;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Outter Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		x = r * cos(toRadians(i)) * radius;
		z = r * sin(toRadians(i)) * -1 * radius;

		verticesList[p] = x;
		p++;
		verticesList[p] = y;
		p++;
		verticesList[p] = z;
		p++;
	}

	//Upper Half of 
	for (int o = oprint; o <= 180; o += Ostep) 
	{
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius;
			z = r * sin(toRadians(i)) * -1 * radius;

			verticesList[p] = x;
			p++;
			verticesList[p] = y;
			p++;
			verticesList[p] = z;
			p++;
		}
	}

	//printf("NumVerts\t:\t%i\nP\t:\t%i\n\n", 3*numVert, p);

	return verticesList;

}

GLfloat * generateTrenchNormals(float radius)
{
	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = ((np + 6) * (nm + 1));

	static GLfloat * normalsList = new GLfloat[3 * numVert];

	int p = 0;

	//Bottom Half of Sphere
	for (int o = 0; o <= 80; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius;
			z = r * sin(toRadians(i)) * -1 * radius;

			normalsList[p] = x;
			p++;
			normalsList[p] = y;
			p++;
			normalsList[p] = z;
			p++;
		}
	}

	//Outer Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0.0;
		p++;
		normalsList[p] = 1.0;
		p++;
		normalsList[p] = 0.0;
		p++;
	}

	//Inner Ring of Bottom Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0.0;
		p++;
		normalsList[p] = 1.0;
		p++;
		normalsList[p] = 0.0;
		p++;
	}

	//Cylinder Middle
	for (int o = 80; o <= 100; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius * 0.75;
			z = r * sin(toRadians(i)) * -1 * radius * 0.75;

			normalsList[p] = x;
			p++;
			normalsList[p] = y;
			p++;
			normalsList[p] = z;
			p++;
		}
	}

	//Inner Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0.0;
		p++;
		normalsList[p] = -1;
		p++;
		normalsList[p] = 0.0;
		p++;
	}

	//Outter Ring of Upper Wall
	for (int i = 0; i <= 360; i += Istep)
	{
		normalsList[p] = 0.0;
		p++;
		normalsList[p] = -1;
		p++;
		normalsList[p] = 0.0;
		p++;
	}

	//Upper Half of 
	for (int o = 100; o <= 180; o += Ostep) {
		r = sin(toRadians(o));
		y = cos(toRadians(o)) * -1 * radius;
		for (int i = 0; i <= 360; i += Istep)
		{
			x = r * cos(toRadians(i)) * radius;
			z = r * sin(toRadians(i)) * -1 * radius;

			normalsList[p] = x;
			p++;
			normalsList[p] = y;
			p++;
			normalsList[p] = z;
			p++;
		}

		return normalsList;

	}
}

GLfloat * generateSphereNormals(float radius, float zRestrict, float xRestrict)
{
	if (np == 1 || nm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int Ostep = 180 / (np - 1);
	int Istep = 360 / nm;

	float x, y, z, r;

	int numVert = np * (nm + 1);
	//printf("NV\t:\t%i\n", 3 * numVert);
	static GLfloat * normalsList = new GLfloat[3 * numVert];

	int p = 0;

	for (int o = 0; o <= 180; o += Ostep)
	{
		r = sin(toRadians(o));

		y = cos(toRadians(o)) * -1 * radius;

		for (int i = 0; i <= 360; i += Istep)
		{

			x = r * cos(toRadians(i)) * radius * xRestrict;
			z = r * sin(toRadians(i)) * -1 * radius * zRestrict;

			normalsList[p] = x;
			p++;

			normalsList[p] = y;
			p++;

			normalsList[p] = z;
			p++;
		}
		//printf("p\t:\t%i\n", p);
	}
	//printf("numvert:\t%i\n", 3 * numVert);
	//printf("P:\t%i\n", p);
	return normalsList;
}

GLfloat * generateCylinderVertex()
{
	if (height == 1 || cnm == 0)
	{
		printf("You cant divide by Zero or you get points taken off!\n");
		return 0;
	}

	int oStep = 180 / (height - 1);
	int iStep = 360 / cnm;

	float x, y, z, r;

	int numVert = height * (cnm + 1);

	static GLfloat * vertList = new GLfloat[3 * numVert];

	int p = 0;

	for (int o = 0; o <= 180; o += oStep)
	{
		//y = cos(toRadians(o)) * -1 * cRadius;
		y = o;
		for (int i = 0; i <= 360; i += iStep)
		{
			x = cos(toRadians(i)) *cRadius;
			z = sin(toRadians(i)) *-1 * cRadius;

			vertList[p] = x;
			p++;

			vertList[p] = y;
			p++;

			vertList[p] = z;
			p++;
		}

	}

	return vertList;
}

GLuint * generateCylinderIndexList()
{

	int numIndex = (2 * (cnm + 1) + 1) * (height - 1);

	static GLuint * indexList = new GLuint[numIndex];

	int p = 0;

	for (int o = 1; o <= height - 1; o++)
	{
		for (int i = o * (cnm + 1); i < o * (cnm + 1) + (cnm + 1); i++)
		{
			indexList[p] = i;
			p++;

			indexList[p] = i - (cnm + 1);
			p++;
		}

		indexList[p] = 0xFFFF;
		p++;

	}

	return indexList;

}

GLuint * generateIndexList(bool trench)
{
	int numIndices;
	if (trench)
	{
		//numIndices = (nm + 1) * (np + 6);
		numIndices = (2 * (nm + 1) + 1 ) * (np + 6);
	}
	else
	{
		numIndices = (2 * (nm + 1) + 1) * (np - 1);
	}

	//We add np - 1 to the number of indices for the number of primitive restart indices
	static GLuint * indices = new GLuint[numIndices];

	//used to keep track of place in index list
	int p = 0;

	int outerMax;
	if (trench)
	{
		outerMax = np + 6;
	}
	else
	{
		outerMax = np - 1;
	}

	for (int o = 1; o <= outerMax; o++)
	{
		//o is the triangle strip number
		for (int i = o * (nm + 1); i < o * (nm + 1) + (nm + 1); i++)
		{
			indices[p] = i;	
			p++;

			indices[p] = i - (nm + 1);
			p++;
		}
		indices[p] = 0xFFFF;
		p++;
	}

	return indices;
}

GLfloat * generateTextureUV(bool trench)
{
	int numIndices;
	float numParrallel;
	float numMerrid = nm;

	if (trench)
	{
		numParrallel = np + 6;
	}
	else 
	{
		numParrallel = np - 1;
	}
	numIndices = 2 * (numParrallel * (nm + 1));

	static GLfloat * indices = new GLfloat[numIndices];

	int p = 0;

	for (int o = 0; o < numParrallel; o++)
	{
		for (int i = 0; i < nm + 1; i++)
		{
			indices[p] = i / (numMerrid);
			if (indices[p] > 1.0 || indices[p] < 0.0) printf("PROBLEM!\n");
			//printf("i is:\t%i / %f\no is:\t%i / %f\nIndices[%i] is\t:\t%f\t", i, numMerrid-1, o, numParrallel-1, p, indices[p]);
			p++;

			indices[p] = o / (numParrallel - 1);
			if (indices[p] > 1.0 || indices[p] < 0.0) printf("PROBLEM!\n");
			//printf("%f\n", indices[p]);
			p++;
		}

		//printf("\n");
		
	}

	printf("P is \t\t:\t%i\n", p);
	printf("NumVerts is \t:\t%i\n", numIndices);
	printf("Last two items are\t:\t%f\t%f\n", indices[p-2], indices[p - 1]);

	//printf("UV Coordinates:\n");

	//for (int i = 0; i < numIndices; i+=2)
	//{
	//	//printf("%f\t%f\t\t%f\t%f\n", indices[i], indices[i+1], indices[i+2], indices[i+3]);
	//	printf("%f , %f\t", indices[i], indices[i+1]);

	//	if ((i % 3) == 0) printf("\n\n");
	//}

	//printf("\n\n");


	return indices;
}

/**
This function sets up a Vertex Array Object and buffers vertex data on the graphics card.
You should change the vertex and index lists to generate sphere geometry as described in
the assignment handout and in class.
*/
GLuint setupSphereVAO(GLuint position_loc, GLuint color_loc, GLuint  norm_loc, GLuint Od_loc, GLuint UV_loc, float radius, float zRestrict, float xRestrict, int colorType, bool trench, bool texture)
{
	int numVerts, indexLsize, loopParrallel;

	GLfloat * vertices;
	GLfloat * normals;
	GLfloat * UV;
	
	if (trench) 
	{
		loopParrallel = np + 6;
		indexLsize = (2 * (nm + 1) + 1) * (np + 6);
		numVerts = (np + 6) * (nm + 1);
		vertices = generateTrenchVertexList(radius);
		normals = generateTrenchNormals(radius);
	}
	else
	{
		loopParrallel = np;
		indexLsize = (2 * (nm + 1) + 1) * (np - 1);
		numVerts = (np) * (nm + 1);
		vertices = generateVertexList(radius, zRestrict, xRestrict);
		//normals = generateSphereNormals(radius, zRestrict, xRestrict);
		normals = vertices;
	}

	GLuint * index_list = generateIndexList(trench);

	GLfloat * colorsT = new GLfloat[numVerts * 3];

	if (texture) {

		UV = generateTextureUV(trench);

	}
	
	else
	{
		UV = NULL;
		float redLevel = 1.0;
		float blueLevel = -1.0;
		float greenLevel = 0.0;
		float colorLevel = 0.0;

		float cvStep = 1.0 / np - 1;
		float chStep = 1.0 / nm;
		int p = 0;

		if (colorType == 1)
		{

			for (int o = 0; o < loopParrallel; o++)
			{
				for (int i = 0; i < nm + 1; i++)
				{
					colorsT[p] = redLevel;
					redLevel *= -1;
					p++;

					colorsT[p] = 0.0;
					greenLevel += chStep;
					p++;

					colorsT[p] = blueLevel;
					blueLevel *= -1;
					p++;
				}
			}
		}
		else if (colorType == 2)
		{
			blueLevel = 0.0;

			for (int o = 0; o < np; o++)
			{
				for (int i = 0; i < nm + 1; i++)
				{
					colorsT[p] = redLevel * colorLevel;
					redLevel -= 0.15;
					p++;

					colorsT[p] = 0.0;
					p++;

					colorsT[p] = blueLevel * colorLevel;
					blueLevel += 0.15;
					p++;
				}

				redLevel = 1.0;
				blueLevel = 0.0;
				colorLevel += 0.15;
			}
		}
		else
		{
			colorLevel = 1.0;
			for (int o = 0; o < np; o++)
			{
				for (int i = 0; i < nm + 1; i++)
				{
					colorsT[p] = colorLevel;
					p++;

					colorsT[p] = colorLevel;
					p++;

					colorsT[p] = colorLevel;
					p++;

				}

				colorLevel -= 0.25;
			}

		}
	}

	GLuint VAO_out, VBO[4], EAB;  /* Create handles for our Vertex Array Object and two Vertex Buffer Objects */

								  /* Assign ID for a Vertex Array Object to our handle */
	glGenVertexArrays(1, &VAO_out);
	/* Assign IDs for two Vertex Buffer Objects to our handle */
	glGenBuffers(4, VBO); //need to add one for normals
	/* Assign ID for an index buffer to our handle */
	glGenBuffers(1, &EAB);

	/* Bind our Vertex Array Object as the current used object */
	glBindVertexArray(VAO_out);

	/* Bind our first VBO as being the active buffer and storing vertex attributes (coordinates) */
	glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
	/* Copy the vertex data from diamond to our buffer */
	/* num_verts * 3 * sizeof(GLfloat) is the size of the diamond array, since */
	/* it contains num_verts * 3 GLfloat values */
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), vertices, GL_STATIC_DRAW);
	/* Specify that our coordinate data is going into attribute index 0 (SHADER_POSITION_INDEX), and contains three floats per vertex */
	glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	/* Enable attribute index 0 (SHADER_POSITION_INDEX) as being used */
	glEnableVertexAttribArray(position_loc);

	if (!texture) {
		/* Bind our second VBO as being the active buffer and storing vertex attributes (colors) */
		glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
		/* Copy the color data from colors to our buffer */
		/* num_verts * 3 * sizeof(GLfloat) is the size of the diamond array, since */
		/* it contains num_verts * 3 GLfloat values */
		glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), colorsT, GL_STATIC_DRAW);
		/* Specify that our color data is going into attribute index 1 (SHADER_COLOR_INDEX), and contains three floats per vertex */
		glVertexAttribPointer(color_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
		/* Enable attribute index 1 (SHADER_COLOR_INDEX) as being used */
		glEnableVertexAttribArray(color_loc);
	}
	
	//Loading in Normals to Vertex Shader
	glBindBuffer(GL_ARRAY_BUFFER, VBO[2]);
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), normals, GL_STATIC_DRAW);
	glVertexAttribPointer(norm_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(norm_loc);

	if (texture) 
	{
		glBindBuffer(GL_ARRAY_BUFFER, VBO[3]);
		glBufferData(GL_ARRAY_BUFFER, 2 * numVerts * sizeof(GLfloat), UV, GL_STATIC_DRAW);
		glVertexAttribPointer(UV_loc, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(UV_loc);
	}	


	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EAB);
	/* Copy the index data from indices to our buffer */
	/* 4 * sizeof(GLuint) is the size of the indices array, since it contains 4 GLuint values */
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexLsize * sizeof(GLuint), index_list, GL_STATIC_DRAW);

	/* Clear array bindings for to avoid later problems */
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	/*
	These calls just remove the names/IDs from use. The buffered data are still associated
	with the vertex array object. Since they are only scoped to this function, however,
	we would normally remove them here or GL will never use them again within the application.
	This probably wouldn't cause errors for the assignment, so we will omit it here.
	*/
	// glDeleteBuffers(2, VBO);
	// glDeleteBuffers(1, &EAB);

	//delete vertices;
	//delete normals;
	//delete UV;

	return VAO_out;
}

GLuint setupCylinder(GLuint position_loc, GLuint color_loc)
{
	int numVerts = height * (nm + 1);
	int numIndex = (2 * (nm + 1) + 1) * (height - 1);

	GLfloat * cylinderVertices = generateCylinderVertex();
	GLuint * cylinderIndexList = generateCylinderIndexList();
	GLfloat * cylinderColors = new GLfloat[numVerts * 3];

	for (int i = 0; i < (height - 1) * (nm + 1); i++)
	{
		cylinderColors[i] = 1.0;
	}

	GLuint VAO_cylinder, VBO_cylinder[2], EABc;

	glGenVertexArrays(1, &VAO_cylinder);
	glGenBuffers(2, VBO_cylinder);
	glGenBuffers(1, &EABc);

	glBindVertexArray(VAO_cylinder);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_cylinder[0]);
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), cylinderVertices, GL_STATIC_DRAW);
	glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(position_loc);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_cylinder[1]);
	glBufferData(GL_ARRAY_BUFFER, numVerts * 3 * sizeof(GLfloat), cylinderColors, GL_STATIC_DRAW);
	glVertexAttribPointer(color_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(color_loc);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EABc);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndex * sizeof(GLuint), cylinderIndexList, GL_STATIC_DRAW);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	return VAO_cylinder;
}

float influence(float distance, float a, float b, float c, float maxDist)
{

	if (distance > maxDist)
	{
		return 0.0;
	}

	return 1 / (a + (b * distance) + (c * distance * distance));

}

void simulationStep()
{

	iterations++;

	//1 - Convert Bird's omega to world-referenced velocity vector
	//Array of all bird velocities
	gmtl::Vec4f * birdVelocity = new gmtl::Vec4f[nb];
	gmtl::Vec4f * birdAccel = new gmtl::Vec4f[nb];
	gmtl::Vec4f * birdForce = new gmtl::Vec4f[nb];
	
	//Container for Y_axis
	gmtl::Vec4f get_Y = { 0, 1, 0, 0 };
	gmtl::Vec3f get_X = { 1, 0, 0 };
	
	gmtl::Vec4f forceForAccel, disponI, centonI, vmatchonI;
	gmtl::Vec4f bird_Y;
	
	float flightRadius = 1.5;
	float delta = 0.05;
	float theta, lateralDist, accelMagAccum, avgRadius, vertDist, distance;

	gmtl::Vec3f U, X_i, X_j;
	gmtl::Vec3f rotVj, subVi;
	gmtl::Vec3f centering, dispersion, velocityMatching, forcesOnBird, forceOfJ, dispF1, centF1, dispcentX, centX;
	
	float influenceLevel;
	float contributingBirds;// = (nb - 1) + no;

	gmtl::Matrix44f U_rot_mat, birdR;
	gmtl::identity(U_rot_mat);
	gmtl::identity(birdR);
	
	gmtl::Quatf U_rot_quat;
	
	bool continueAccel = true;
	maxAccel = 2.0f;

	float * bankingR = new float[nb];
	float rMax = 1.1;
	
	//1 - Convert Bird's omega to world-referenced velocity vector
	for (int n = 0; n < nb; n++)
	{
		//get Y-axis as a vector
		bird_Y = birdArray[n].getRot() * get_Y;
		birdVelocity[n] = bird_Y * birdArray[n].getRadius() * birdArray[n].getOmega();
	}

	//Step 2 - Compute bird's world referenced acceleration vector based on intrabird forces
	for (int n = 0; n < nb; n++)
	{
		X_i = birdArray[n].getRot() * get_X;
		subVi = { birdVelocity[n][0], birdVelocity[n][1], birdVelocity[n][2] };

		birdR = W * birdArray[n].getRot();
		
		dispcentX =  W * birdArray[n].getRot() * get_X;

		subVi = subVi + (dispcentX * birdArray[n].getVertVel());

		forcesOnBird = { 0,0,0 };
		forceOfJ = { 0, 0, 0 };
		contributingBirds = nb + no - 1;
		continueAccel = true;

		disponI, centonI, vmatchonI = { 0,0,0,0 };
		accelMagAccum = 0;

		//For each bird, calculate dispersion for obstacles
		for (int o = 0; o < no; o++)
		{
			X_j = obstacleList[o] * get_X;

			gmtl::cross(U, X_i, X_j);
			gmtl::normalize(U);

			theta = acos(gmtl::dot(X_i, X_j));

			avgRadius = (birdArray[n].getRadius() + 1.6) * 0.5;

			lateralDist = theta * avgRadius;

			vertDist = birdArray[n].getRadius() - 1.6;

			distance = sqrtf((lateralDist * lateralDist) + (vertDist * vertDist));
			distance *= 0.2;

			gmtl::cross(dispF1, X_i, U);

			dispersion = (lateralDist * dispF1) + (vertDist * dispcentX);
			gmtl::normalize(dispersion);

			dispersion = dispersion * influence(distance, 1, 7, 12, 0.75);

			if (influence(distance, 1, 7, 3, 2.5) == 0) contributingBirds--;

			accelMagAccum += gmtl::length(dispersion);
			if (accelMagAccum < maxAccel)
			{
				disponI += gmtl::Vec4f(dispersion[0], dispersion[1], dispersion[2], 0.0f);
				BirdDisp[n] = BirdDisp[n] + gmtl::Vec4f(dispersion[0], dispersion[1], dispersion[2], 0.0f);
			}
			else
			{
				continueAccel = false;
			}
		}

		//printf("Disp from obj on bird %i is:\t%f\t%f\t%f\n", n, disponI[0], disponI[1], disponI[2], disponI[3]);
		//printf("\t\t\tmag: %f\n", gmtl::length(disponI));
		
		//Calculate dispersion for all birds
		if (continueAccel)
		{
			for (int i = 0; i < nb; i++)
			{
				if (i != n)
				{
					X_j = birdArray[i].getRot() * get_X;

					gmtl::cross(U, X_i, X_j);
					gmtl::normalize(U);

					theta = acos(gmtl::dot(X_i, X_j));

					avgRadius = (birdArray[n].getRadius() + birdArray[i].getRadius()) * 0.5;

					lateralDist = theta * avgRadius;

					vertDist = birdArray[n].getRadius() - birdArray[i].getRadius();

					distance = sqrtf((lateralDist * lateralDist) + (vertDist * vertDist));

					gmtl::cross(dispF1, X_i, U);

					dispersion = (lateralDist * dispF1) + (vertDist * dispcentX);
					gmtl::normalize(dispersion);

					dispersion = dispersion * influence(distance, 1, 2, 4, 1.5);

					if (influence(distance, 1, 7, 3, 2.5) == 0) contributingBirds--;

					accelMagAccum += gmtl::length(dispersion);

					if (accelMagAccum < maxAccel)
					{
						disponI += gmtl::Vec4f(dispersion[0], dispersion[1], dispersion[2], 0.0f);
						BirdDisp[n] = BirdDisp[n] + gmtl::Vec4f(dispersion[0], dispersion[1], dispersion[2], 0.0f);
					}
					else
					{
						continueAccel = false;
					}

				}
			}
		}
		//printf("Disp from bird on bird %i is:\t%f\t%f\t%f\n", n, disponI[0], disponI[1], disponI[2], disponI[3]);
		//printf("\t\t\tmag: %f\n", gmtl::length(disponI));
		
		
		//Calculate Velocity Matching
		if (continueAccel)
		{
			for (int i = 0; i < nb; i++)
			{
				if (i != n)
				{
					X_j = birdArray[i].getRot() * get_X;

					gmtl::cross(U, X_i, X_j);
					gmtl::normalize(U);

					theta = acos(gmtl::dot(X_i, X_j));

					avgRadius = (birdArray[n].getRadius() + birdArray[i].getRadius()) * 0.5;

					lateralDist = theta * avgRadius;

					vertDist = birdArray[n].getRadius() - birdArray[i].getRadius();

					distance = sqrtf((lateralDist * lateralDist) + (vertDist * vertDist));

					rotVj = { birdVelocity[i][0], birdVelocity[i][1], birdVelocity[i][2] };

					//gmtl::AxisAnglef rot_about_U(theta * -1, U);
					U_rot_quat[0] = sin(toRadians(theta * -0.5)) * U[0];
					U_rot_quat[1] = sin(toRadians(theta * -0.5)) * U[1];
					U_rot_quat[2] = sin(toRadians(theta * -0.5)) * U[2];
					U_rot_quat[3] = cos(toRadians(theta * -0.5));

					U_rot_mat = gmtl::make<gmtl::Matrix44f>(U_rot_quat);

					//		rotate vector birdVelocity[i] around U
					rotVj = U_rot_mat * rotVj;

					//		subtract vector birdVelocity[n] from above
					velocityMatching = rotVj - subVi;

					//		scale by custom influence level
					velocityMatching *= influence(distance, 1, 7, 4, 2.5);

					accelMagAccum += gmtl::length(velocityMatching);
					if (accelMagAccum < maxAccel)
					{
						vmatchonI += gmtl::Vec4f(velocityMatching[0], velocityMatching[1], velocityMatching[2], 0.0f);
						forceOfJ += velocityMatching;
						BirdVelMatch[n] = BirdDisp[n] + gmtl::Vec4f(velocityMatching[0], velocityMatching[1], velocityMatching[2], 0.0f);
						//printf("VelMatching magnitude bird %i: \t%f\n", n, gmtl::length(velocityMatching));
					}
					else
					{
						continueAccel = false;
						//printf("vmatch not used on bird%i\n", n);
						//printf("VelMatching magnitude bird %i: \t%f\tWas not added!\n", n, gmtl::length(velocityMatching));
					}
				}
			}
		}
		//printf("vmatch from bird on bird %i is:\t%f\t%f\t%f\n", n, vmatchonI[0], vmatchonI[1], vmatchonI[2], vmatchonI[3]);
		//printf("\t\t\tmag: %f\n", gmtl::length(vmatchonI));
		
		
		//Calculate Centering
		if (continueAccel)
		{
			for (int i = 0; i < nb; i++)
			{
				if (i != n)
				{
					X_j = birdArray[i].getRot() * get_X;

					gmtl::cross(U, X_i, X_j);
					gmtl::normalize(U);

					theta = acos(gmtl::dot(X_i, X_j));

					avgRadius = (birdArray[n].getRadius() + birdArray[i].getRadius()) * 0.5;

					lateralDist = theta * avgRadius;

					vertDist = birdArray[n].getRadius() - birdArray[i].getRadius();

					distance = sqrtf((lateralDist * lateralDist) + (vertDist * vertDist));
					gmtl::cross(centF1, U, X_i);

					centering = (lateralDist * centF1) + (vertDist * dispcentX);
					gmtl::normalize(centering);

					centering = centering * influence(distance, 1.1, 5, 3, 3.0);

					//if (influence(distance, 1, 1, 1) == 0) contributingBirds--;

					accelMagAccum += gmtl::length(centering);
					if (accelMagAccum < maxAccel)
					{
						centonI += gmtl::Vec4f(centering[0], centering[1], centering[2], 0.0f);
						forceOfJ += centering;
						BirdCenter[n] = BirdCenter[n] + gmtl::Vec4f(centering[0], centering[1], centering[2], 0.0f);
						//printf("Centering magnitude bird %i: \t%f\n", n, gmtl::length(centering));
					}
					else
					{
						//printf("Cent not used on bird%i\n", n);
						//printf("Centering was not added to calculations!\n");
					}
				}
			}
		}
		//printf("Cent from bird on bird %i is:\t%f\t%f\t%f\n", n, centonI[0], centonI[1], centonI[2], centonI[3]);
		//printf("\t\t\tmag: %f\n", gmtl::length(centonI));

		forceForAccel = disponI + centonI + vmatchonI;
		birdForce[n] = forceForAccel;
		

		if (gmtl::length(BirdDisp[n]) > maxDispMag)
		{
			maxDispMag = gmtl::length(BirdDisp[n]);
		}

		if (gmtl::length(BirdCenter[n]) > maxCentMag)
		{
			maxCentMag = gmtl::length(BirdCenter[n]);
		}
		if (gmtl::length(BirdVelMatch[n]) > maxVmatchMag)
		{
			maxVmatchMag = gmtl::length(BirdVelMatch[n]);
		}

		dispAccMag += gmtl::length(BirdDisp[n]);
		centAccMag += gmtl::length(BirdCenter[n]);
		vmatchAccMag += gmtl::length(BirdVelMatch[n]);

		meanCentMag = centAccMag / (iterations * nb);
		meanDispMag = dispAccMag / (iterations * nb);
		meanVmatchMag = vmatchAccMag / (iterations * nb);

		//printf("Iterations\t: %f\n\n", iterations);
		//
		//printf("Actual Dispersion Magnitude: %f\n", gmtl::length(BirdDisp[n]));
		//printf("Mean Dispersion Magnitude: %f\n", meanDispMag);
		//printf("Largest Dispersion Magnitude: %f\n\n", maxDispMag);
		//
		//printf("Actual Centering Magnitude: %f\n", gmtl::length(BirdCenter[n]));
		//printf("Mean Centering Magnitude: %f\n", meanCentMag);
		//printf("Largest Centering Magnitude: %f\n\n", maxCentMag);
		//
		//printf("Actual VelMatching Magnitude: %f\n", gmtl::length(BirdVelMatch[n]));
		//printf("Mean VelMatching Magnitude: %f\n", meanVmatchMag);
		//printf("Largest VelMatching Magnitude: %f\n\n", maxVmatchMag);
		//
		gmtl::normalize(BirdDisp[n]);
		BirdDisp[n] *= 0.5;

		gmtl::normalize(BirdCenter[n]);
		BirdCenter[n] *= 0.5;

		gmtl::normalize(BirdVelMatch[n]);
		BirdVelMatch[n] *= 0.5;

		birdAccel[n] = { forceForAccel[0] / contributingBirds, forceForAccel[1] / contributingBirds , forceForAccel[2] / contributingBirds, forceForAccel[3] / contributingBirds };

		
		/*printf("Bird %i:\n", n);
		for (int o = 0; o < 4; o++)
		{
			for (int i = 0; i < 4; i++)
			{
				printf("%f\t", birdArray[n].getRot(o, i));
			}
			printf("\n");
		}
		printf("velo on bird %i is\t: \t%f\t%f\t%f\n", n, birdVelocity[n][0], birdVelocity[n][1], birdVelocity[n][2], birdVelocity[n][3]);
		printf("velo mag is\t\t:\t%f\n", gmtl::length(birdVelocity[n]));
		printf("accel on bird %i is\t: \t%f\t%f\t%f\n", n, birdAccel[n][0] * delta, birdAccel[n][1] * delta, birdAccel[n][2] * delta, birdAccel[n][3] * delta);
		printf("birds contrib to bird %i is:\t%f\n", n, contributingBirds);
		printf("radius on bird %i is\t:\t%f\n", n, birdArray[n].getRadius());
		printf("vervel on bird %i is\t:\t%f\n", n, birdArray[n].getVertVel());
		printf("omega for bird %i is\t: \t%f\n", n, birdArray[n].getOmega());
		*/
		
		if (accelMagAccum > maxAccelMagAccum)
		{
			maxAccelMagAccum = accelMagAccum;
		}

		totalAccelMagAccum += accelMagAccum;
		avgAccelMagAccum = totalAccelMagAccum / (iterations * nb);

		//printf("avgAccelMagAccum\t:\t%f\n", avgAccelMagAccum);
		//printf("maxAccelMagAccum\t:\t%f\n", maxAccelMagAccum);
		//printf("\n\n");
		
		if (gmtl::length(birdAccel[n]) > maxAccMag)
		{
			maxAccMag = gmtl::length(birdAccel[n]);
		}

		totalAccMag += gmtl::length(birdAccel[n]);

		meanAccMag = totalAccMag / iterations;

		//maxAccel = meanAccMag;
		//
		//(meanDispMag + meanCentMag + meanVmatchMag) / (iterations);
		//
		//printf("Max Accel is now: \t%f\n\n\n", maxAccel);
		//
		//printf("Mean Accelerations Magnitude is: %f\n", meanAccMag);
		//
		//printf("Largest Acceleration Magnitude: %f\n\n", maxAccMag);
		//
		//printf("Accel is: \t%f\t%f\t%f\t%f\n\n", birdAccel[n][0] * delta, birdAccel[n][1] * delta, birdAccel[n][2] * delta, birdAccel[n][3] * delta);
		//
		//printf("Bird %i Radius is: \t%f\n\n\n", n, birdArray[n].getRadius());
		//printf("Bird %i VertVel is: \t%f\n\n\n", n, birdArray[n].getVertVel());

	}

	//printf("\n\n\n");
	
	gmtl::Vec4f plus_Y = { 0, 0.02f, 0, 0 };
	gmtl::Matrix44f birdO, birdRtO;
	gmtl::Vec4f get_Z4 = { 0, 0, 1, 0 };
	gmtl::Vec4f ZofO;
	//Step 3 - Update bird's world-referenced velocity by adding the product of step 2 and delta
	for (int n = 0; n < nb; n++)
	{	
		birdVelocity[n] = birdVelocity[n] + (delta * birdAccel[n]);

		//Flight Speed Control
		if (gmtl::length(birdVelocity[n]) < 0.5)
		{
			birdVelocity[n] += 0.2f * birdVelocity[n];//gmtl::Vec4f(birdVelocity[n][0] * 0.2, birdVelocity[n][1] * 0.2, birdVelocity[n][2] * 0.2, birdVelocity[n][3] * 0.2);
		}
		else if (gmtl::length(birdVelocity[n]) > 2.0)
		{
			birdVelocity[n] -= 0.2f * birdVelocity[n];//gmtl::Vec4f(birdVelocity[n][0] * 0.2, birdVelocity[n][1] * 0.2, birdVelocity[n][2] * 0.2, birdVelocity[n][3] * 0.2);;
		}

		//We need
		// O - Z -axis
		// Acceleration
		gmtl::identity(birdO);
		gmtl::identity(birdRtO);

		birdRtO[0][3] = birdArray[n].getRadius();
		birdRtO.setState(gmtl::Matrix44f::TRANS);

		birdO = W * birdArray[n].getRot() * birdRtO;
		ZofO = birdO * get_Z4;

		bankingR[n] = gmtl::dot(ZofO, birdForce[n]) * -1;

		if (bankingR[n] > maxBanking) 
		{
			maxBanking = bankingR[n];
		}

		//I dont feel like looking up the syntax for clamping
		if (bankingR[n] > 1.1) bankingR[n] = 1.1;

		//printf("max banking angle is\t:\t%f\n\n\n ", maxBanking);
	}

	//Step 4 - Rotate bird to point in updated velocity and convert velocity into omega
	gmtl::Matrix44f newRot;
	gmtl::Vec3f newZ, newY, oldZ, oldY, oldX, newV_i;
	float newOmega, newVertVel;
	gmtl::Vec3f get_Z = { 0, 0, 1 };
	gmtl::Vec3f get_Y3 = { 0, 1 ,0 };
	gmtl::Vec4f get_X4 = { 1, 0, 0, 0};
	
	for (int n = 0; n < nb; n++)
	{
		gmtl::identity(newRot);

		oldX = birdArray[n].getRot() * get_X;
		oldY = birdArray[n].getRot() * get_Y3;
		oldZ = birdArray[n].getRot() * get_Z;

		newV_i = { birdVelocity[n][0], birdVelocity[n][1], birdVelocity[n][2] };

		//New Z is normalized cross product of X and Vi from Step 3
		if (gmtl::length(oldX) == gmtl::length(newV_i))
		{
			gmtl::cross(newZ, oldX, oldY);
		}
		else
		{
			gmtl::cross(newZ, oldX, newV_i);
		}
		gmtl::normalize(newZ);
		
		gmtl::cross(newY, newZ, oldX);

		//printf("New Y for bird %i  is: \t%f\t%f\t%f\n", n, newY[0], newY[1], newY[2]);
		//printf("New V for bird %i  is: \t%f\t%f\t%f\n", n, newV_i[0], newV_i[1], newV_i[2]);

		//Set X
		newRot[0][0] = oldX[0];
		newRot[1][0] = oldX[1];
		newRot[2][0] = oldX[2];

		//Set Y
		newRot[0][1] = newY[0];
		newRot[1][1] = newY[1];
		newRot[2][1] = newY[2];

		//Set Z
		newRot[0][2] = newZ[0];
		newRot[1][2] = newZ[1];
		newRot[2][2] = newZ[2];
		
		//Compute new Omega
		newOmega = gmtl::dot(newY, newV_i) / birdArray[n].getRadius();

		newVertVel = gmtl::dot(newV_i, oldX);
		
		/*printf("For bird %i, rotation is:\n");
		for (int o = 0; o < 4; o++)
		{
			for (int i = 0; i < 4; i++)
			{
				printf("%f\t", birdArray[n].getRot(o, i));
			}

			printf("\n");

		}
		printf("\n\n");
		printf("For bird %i x is: \t%f\t%f\t%f\n\n\n", n, temp[0], temp[1], temp[2]);
		printf("Current VertVel for bird %i is: %f\n\n\n", n, newVertVel);*/

		//Update State
		birdArray[n].setOmega(newOmega);
		birdArray[n].setRot(newRot);
		birdArray[n].setVertVel(newVertVel);

		

	}



	//Step 5 - Move the bird forward using local z-rotation of R by an angle that is product of omega and delta
	float rotationAmount, bankingAngle, pitchAngle;
	gmtl::Matrix44f change, move, radius, bank, pitch, currentRot;
	gmtl::Vec4f plus_X = { 0.07f, 0, 0, 0 };
	gmtl::Vec4f normVel, pitchX;

	gmtl::Quatf banking;
	gmtl::Quatf pitching;

	for (int n = 0; n < nb; n++)
	{
		gmtl::identity(change);
		gmtl::identity(radius);
		gmtl::identity(bank);
		gmtl::identity(pitch);
		gmtl::identity(move);

		currentRot = birdArray[n].getRot();

		rotationAmount = birdArray[n].getOmega() * delta;

		birdArray[n].setRadius(birdArray[n].getRadius() + ( birdArray[n].getVertVel() * delta ));

		if (birdArray[n].getRadius() < 1.0)
		{
			birdVelocity[n] += plus_X;
		}
		else if (birdArray[n].getRadius() > 1.8)
		{
			birdVelocity[n] -= plus_X;
		}

		//WtR
		move[0][0] = cos(toRadians(rotationAmount));
		move[0][1] = sin(toRadians(rotationAmount)) * -1;
		move[1][0] = sin(toRadians(rotationAmount));
		move[1][1] = cos(toRadians(rotationAmount));
		move.setState(gmtl::Matrix44f::ORTHOGONAL);
		//change = change * move;


		//RtO
		radius[0][3] = birdArray[n].getRadius();
		radius.setState(gmtl::Matrix44f::TRANS);
		//change = change * radius;

		//OtB
		bankingAngle = asin(toRadians(bankingR[n] / rMax));
		//bankingAngle = toRadians(15);

		banking[3] = cos(bankingAngle * 0.5);
		banking[1] = sin(bankingAngle * 0.5);

		bank = gmtl::make<gmtl::Matrix44f>(banking);

		/*bank[0][0] = cos(bankingAngle);
		bank[0][2] = sin(bankingAngle) * -1;
		bank[2][0] = sin(bankingAngle);
		bank[2][2] = cos(bankingAngle);
		bank.setState(gmtl::Matrix44f::ORTHOGONAL);*/

		

		//pitch
		normVel = birdVelocity[n];
		gmtl::normalize(normVel);

		pitchX = birdArray[n].getRot() * get_X4;
		//pitchAngle = toRadians(15);
		pitchAngle = asin(toRadians(gmtl::dot(pitchX, normVel)));

		/*printf("Bird %i ", n);
		printf("BankAngle\t:\t%f\n\n\n", bankingAngle);*/

		pitching[3] = cos(pitchAngle * 0.5);
		pitching[2] = sin(pitchAngle * 0.5);

		pitch = gmtl::make<gmtl::Matrix44f>(pitching);

		//pitchAngle = asin(toRadians(gmtl::dot(pitchX, normVel)));
		



		//change = change * pitch * bank;

		birdOtB[n] = pitch * bank;
		birdArray[n].setRot(currentRot * move * radius);
	}

	//delete birdVelocity;
	//delete birdAccel;
	//delete birdForce;

}

/*
Call our function that performs opengl operations. This is where your changes for matrix and vertex
manipulation should be. You will also have to send in multiple VAOs (maybe use an array?) and change
the hard-coded count (4) to each index list size.
*/

void display(GLFWwindow* window, GLuint shader_program, GLuint vm_location, GLuint pvm_location, GLuint light_pos, gmtl::Matrix44f LightSourcePosition, GLuint Wte_loc, gmtl::Matrix44f World_to_Eye, gmtl::Matrix44f P[], GLuint VAO[], gmtl::Matrix44f mat[], gmtl::Vec4f O[], GLuint textureID[])
{

	glUseProgram(shader_program);
	int indexLsize;
	indexLsize = (2 * (nm + 1) + 1) * (np + 6);
	//indexLsize = (2 * (nm + 1) + 1) * (np - 1);
	//indexLsize = (2 * (cnm + 1) + 1) * (height - 1);

	/* Make our background black */
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Enable primitive restart index
	glEnable(GL_PRIMITIVE_RESTART);

	//Set primitive restart index
	glPrimitiveRestartIndex(0xFFFF);

	//Bind VAO for Sphere
	glBindVertexArray(VAO[0]);

	//Load in light source location
	glUniformMatrix4fv(light_pos, 1, GL_FALSE, LightSourcePosition.mData);

	//Load in World to Eye location
	glUniformMatrix4fv(Wte_loc, 1, GL_FALSE, World_to_Eye.mData);

	GLuint boolLocation;
	boolLocation = glGetUniformLocation(shader_program, "in_bText");

	glUniform1i(boolLocation, 2);
	
	glBindTexture(GL_TEXTURE_2D, textureID[1]);
	
	//Load M to transform Sphere
	glUniformMatrix4fv(
		vm_location,		// uniform location
		1,					// count
		GL_FALSE,			// transpose (no)
		mat[0].mData		// data
		);

	glUniformMatrix4fv(
		pvm_location,
		1,
		GL_FALSE,
		P[0].mData
		);

	//Draw Sphere
	glDrawElements(
		GL_TRIANGLE_STRIP,
		indexLsize,
		GL_UNSIGNED_INT,
		(void*)0
		);


	//Drawing Skybox
	glBindVertexArray(VAO[4]);
	indexLsize = (2 * (nm + 1) + 1) * (np - 1);
	glUniform1i(boolLocation, 1);

	glBindTexture(GL_TEXTURE_2D, textureID[0]);
	
	glUniformMatrix4fv(
		vm_location,		// uniform location
		1,					// count
		GL_FALSE,			// transpose (no)
		mat[1].mData		// data
		);

	glUniformMatrix4fv(
		pvm_location,
		1,
		GL_FALSE,
		P[1].mData
		);

	glDrawElements(
		GL_TRIANGLE_STRIP,
		indexLsize,
		GL_UNSIGNED_INT,
		(void*)0
		);

	glUniform1i(boolLocation, 0);

	glBindTexture(GL_TEXTURE_2D, textureID[0]);
	

	if (showW)
	{
		//Bind VAO for Y-aligned Cylinder
		glBindVertexArray(VAO[3]);

		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[2].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[2].mData
			);

		//Draw Y-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,		// mode
			indexLsize,				// count, should be updated with your index list size.
			GL_UNSIGNED_INT,		// type
			(void*)0				// element array buffer offset
			);
		//Load in matrix data from matrix array
		// Y_to_X_rot should be the first matrix passed in at mat[0]
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[3].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[3].mData
			);
		//Draw X-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
		//Create Matrix for rotating Y-aligned Cylinder
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[4].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[4].mData
			);
		//Draw Z-Aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
	}

	if (showR)
	{
		//Bind VAO for Y-aligned Cylinder
		glBindVertexArray(VAO[3]);

		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[5].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[5].mData
			);
		//Draw Y-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,		// mode
			indexLsize,				// count, should be updated with your index list size.
			GL_UNSIGNED_INT,		// type
			(void*)0				// element array buffer offset
			);
		//Load in matrix data from matrix array
		// Y_to_X_rot should be the first matrix passed in at mat[0]
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[6].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[6].mData
			);
		//Draw X-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
		//Create Matrix for rotating Y-aligned Cylinder
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[7].mData
			);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[7].mData
			);
		//Draw Z-Aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);

	}

	if (showO)
	{
		//Bind VAO for Y-aligned Cylinder
		glBindVertexArray(VAO[3]);

		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[8].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[8].mData
			);
		//Draw Y-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,		// mode
			indexLsize,				// count, should be updated with your index list size.
			GL_UNSIGNED_INT,		// type
			(void*)0				// element array buffer offset
			);
		//Load in matrix data from matrix array
		// Y_to_X_rot should be the first matrix passed in at mat[0]
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[9].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[9].mData
			);
		//Draw X-aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
		//Create Matrix for rotating Y-aligned Cylinder
		glUniformMatrix4fv(
			vm_location,
			1,
			GL_FALSE,
			mat[10].mData
			);
		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[10].mData
			);
		//Draw Z-Aligned Cylinder
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
			);
	}

	indexLsize = (2 * (nm + 1) + 1) * (np - 1);
	
	if (!showCenter && !showDisp && !showMatch)
	{
		for (int i = 0; i < 3 * nb; i += 3)
		{

			//Bind VAO for bird
			glBindVertexArray(VAO[1]);

			//Load B as location to draw to
			glUniformMatrix4fv(
				vm_location,			// uniform location
				1,						// count
				GL_FALSE,				// transpose (no)
				mat[11 + i].mData		// data
				);

			glUniformMatrix4fv(
				pvm_location,
				1,
				GL_FALSE,
				P[11 + i].mData
				);

			//Draw elements
			glDrawElements(
				GL_TRIANGLE_STRIP,
				indexLsize,
				GL_UNSIGNED_INT,
				(void*)0
				);

			//Bind VAO for joint
			glBindVertexArray(VAO[2]);

			//Load Joint 1 model matrix
			glUniformMatrix4fv(
				vm_location,			// uniform location
				1,						// count
				GL_FALSE,				// transpose (no)
				mat[12 + i].mData		// data
				);

			glUniformMatrix4fv(
				pvm_location,
				1,
				GL_FALSE,
				P[12 + i].mData
				);

			//Draw Joint 1
			glDrawElements(
				GL_TRIANGLE_STRIP,
				indexLsize,
				GL_UNSIGNED_INT,
				(void*)0
				);

			//Load Joint 2 model matrix
			glUniformMatrix4fv(
				vm_location,			// uniform location
				1,						// count
				GL_FALSE,				// transpose (no)
				mat[13 + i].mData		// data
				);

			glUniformMatrix4fv(
				pvm_location,
				1,
				GL_FALSE,
				P[13 + i].mData
				);

			//Draw Joint 2
			glDrawElements(
				GL_TRIANGLE_STRIP,
				indexLsize,
				GL_UNSIGNED_INT,
				(void*)0
				);
		}
	}
	if(showDisp || showCenter || showMatch)
	{
		//printf("One of these things is true\n");
		for (int b = 0; b < nb; b++)
		{
			if (showDisp) 
			{
				glBegin(GL_LINES);
				glColor3f(0.0, 0.0, 0.0);
				glVertex4f(O[b][0], O[b][1], O[b][2], O[b][3]);
				glVertex4f(O[b][0] + BirdDisp[b][0], O[b][1] + BirdDisp[b][1], O[b][2] + BirdDisp[b][2], O[b][3] + BirdDisp[b][3]);
				glEnd();
			}

			if (showCenter) 
			{
				glBegin(GL_LINES);
				glColor3f(0.0, 0.0, 1.0);
				glVertex4f(O[b][0], O[b][1], O[b][2], O[b][3]);
				glVertex4f(O[b][0] + BirdCenter[b][0], O[b][1] + BirdCenter[b][1], O[b][2] + BirdCenter[b][2], O[b][3] + BirdCenter[b][3]);
				glEnd();
			}

			if (showMatch) 
			{
				glBegin(GL_LINES);
				glColor3f(0.0, 0.0, 1.0);
				glVertex4f(O[b][0], O[b][1], O[b][2], O[b][3]);
				glVertex4f(O[b][0] + BirdVelMatch[b][0], O[b][1] + BirdVelMatch[b][1], O[b][2] + BirdVelMatch[b][2], O[b][3] + BirdVelMatch[b][3]);
				glEnd();
			}
		}
	}

	indexLsize = (2 * (nm + 1) + 1) * (np + 6);

	for (int o = 0; o < no; o++)
	{
		//Bind VAO for bird
		glBindVertexArray(VAO[3]);

		//Load B as location to draw to
		glUniformMatrix4fv(
			vm_location,						// uniform location
			1,									// count
			GL_FALSE,							// transpose (no)
			mat[11 + (3 * nb ) + o].mData		// data
		);

		glUniformMatrix4fv(
			pvm_location,
			1,
			GL_FALSE,
			P[11 + (3 * nb) + o].mData
		);

		//Draw elements
		glDrawElements(
			GL_TRIANGLE_STRIP,
			indexLsize,
			GL_UNSIGNED_INT,
			(void*)0
		);
	}

	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

/* Our program's entry point */
int main(int argc, char *argv[])
{
	GLFWwindow* mainwindow = NULL;
	GLuint program = NULL, VAO = NULL, VAOb = NULL, VAOj = NULL, VAOc = NULL, VAOo = NULL, VAOs = NULL;
	GLuint pos_location = NULL, color_location = NULL, vm_location = NULL, pvm_location = NULL, norm_location = NULL, light_pos = NULL, WtE_loc = NULL, Od_loc = NULL, UV_loc = NULL;
	GLuint textureID[2];

	//Array of Matrix elements for passing into display funciont
	gmtl::Matrix44f * modelMat = new gmtl::Matrix44f[11 + (nb * 3) + no];
	gmtl::Matrix44f * perspectiveMat = new gmtl::Matrix44f[11 + (nb * 3) + no];
	gmtl::Vec4f * lineOrigins = new gmtl::Vec4f[nb];

	gmtl::Matrix44f Y_rot;
	gmtl::identity(Y_rot);
	
	//Matrix describing the rotation needed to get the Y-axis into the x-axis space
	gmtl::Matrix44f Y_to_X_rot;
	gmtl::identity(Y_to_X_rot);
	Y_to_X_rot[0][0] = cos(toRadians(-90));
	Y_to_X_rot[0][1] = sin(toRadians(-90)) * -1;
	Y_to_X_rot[1][0] = sin(toRadians(-90));
	Y_to_X_rot[1][1] = cos(toRadians(-90));
	Y_to_X_rot.setState(gmtl::Matrix44f::ORTHOGONAL);
	
	//Matrix descritbing how to rotate the Y-axis into the Z-Axis space
	gmtl::Matrix44f Y_to_Z_rot;
	gmtl::identity(Y_to_Z_rot);
	Y_to_Z_rot[1][1] = cos(toRadians(90));
	Y_to_Z_rot[1][2] = sin(toRadians(90)) * -1;
	Y_to_Z_rot[2][1] = sin(toRadians(90));
	Y_to_Z_rot[2][2] = cos(toRadians(90));
	Y_to_Z_rot.setState(gmtl::Matrix44f::ORTHOGONAL);

	//Perspective Matrix things
	gmtl::Matrix44f P;
	gmtl::identity(P);

	float near, far, top, bottom, left, right;
	near = 0.1;
	far = 100;
	left = -.16;
	right = .16;
	top = .09;
	bottom = -.09;

	P[0][0] = (2 * near) / (right - left);
	P[1][1] = (2 * near) / (top - bottom);
	P[2][2] = ((far + near) * -1) / (far - near);
	P[3][2] = -1;
	P[2][3] = (-2 * far * near) / (far - near);
	P[0][2] = (right + left) / (right - left);
	P[1][2] = (top + bottom) / (top - bottom);

	gmtl::Matrix44f LightSourcePosition;
	gmtl::identity(LightSourcePosition);
	LightSourcePosition[0][3] = 1.5;
	LightSourcePosition[1][3] = 1.5;
	LightSourcePosition[2][3] = 1.5;

	gmtl::Matrix44f World_to_Eye;
	gmtl::identity(World_to_Eye);

	gmtl::Quatf yRot, xRot, camRotChange;

	/* This function sets up our window, OpenGL context, etc. For assignments, you don't need to know how it works. */
	mainwindow = setupWindow();

	/* This function sets up shaders on the graphics card. For assignments, you don't need to know how it works. */
	program = setupShaderProgram();

	/* This function gets attribute and uniform locations from shaders. For _this_ assignment, you don't need to know how it works. */
	init(program, pos_location, color_location, vm_location, pvm_location, norm_location, light_pos, WtE_loc, Od_loc, UV_loc, textureID);
	initMatrices();
	/*
	This function sets up a Vertex Array Object and buffers vertex data on the graphics card.
	You should change the vertex and index lists to generate sphere geometry as described in
	the assignment handout and in class.
	*/
	VAO  = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 1.0, 1.0, 1.0, 1, false, true);
	VAOb = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 0.1, 1.0, 0.5, 2, false, false);
	VAOj = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 0.07, 0.3, 0.3, 3, false, false);
	VAOo = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 0.2, 1.0, 1.0, 1, true, false);
	VAOs = setupSphereVAO(pos_location, color_location, norm_location, Od_loc, UV_loc, 5.0, 1.0, 1.0, 1, false, true);

	//GLuint VAO_cylinder;
	//VAOc = setupCylinder(pos_location, color_location);

	GLuint VAOarray[5];
	VAOarray[0] = VAO;
	VAOarray[1] = VAOb;
	VAOarray[2] = VAOj;
	VAOarray[3] = VAOo;
	VAOarray[4] = VAOs;

	glfwSetKeyCallback(mainwindow, key_callback);
	glfwSetMouseButtonCallback(mainwindow, mouse_button_callback);

	initBird(false);

	initObstacles();

	gmtl::Matrix44f WtS, cam;
	gmtl::identity(cam);
	gmtl::Matrix44f tempX, tempY, tempT, tempR;

	double lastXD = 0.0;
	double lastYD = 0.0;

	while (!glfwWindowShouldClose(mainwindow))
	{	
		//Do Calculations First:
		if (!pause) {
			simulationStep();
			simulationStep();
			simulationStep();
		}

		//Mouse Calculations
		if (captureMouse)
		{
			glfwGetCursorPos(mainwindow, &x_End, &y_End);
			x_Diff = x_End - x_Beg;
			y_Diff = y_End - y_Beg;
			
			x_Diff *= 0.72;
			y_Diff *= 0.72;

			lastYD = y_Diff;
			lastXD = x_Diff;
			if (!activeCam) {
				
				//X-Rotation
				xRot[1] = sin(toRadians(x_Diff * -0.5));
				xRot[3] = cos(toRadians(x_Diff * -0.5));

				//Y-Rotation
				yRot[0] = sin(toRadians(y_Diff * 0.5));
				yRot[3] = cos(toRadians(y_Diff * 0.5));

			
				tempX = gmtl::make<gmtl::Matrix44f>(xRot);
				tempY = gmtl::make<gmtl::Matrix44f>(yRot);

				R_long = tempY * R_long;
				R_lat = tempX * R_lat;

			}
			else if (activeCam)
			{
				//Y-rotation by ele
				R_ele[0][0] = cos(toRadians((float)y_Diff));
				R_ele[0][2] = sin(toRadians((float)y_Diff)) * -1;
				R_ele[2][0] = sin(toRadians((float)y_Diff));
				R_ele[2][2] = cos(toRadians((float)y_Diff));
				R_long.setState(gmtl::Matrix44f::ORTHOGONAL);

				////X-rotation by azi
				R_azi[1][1] = cos(toRadians(-(float)x_Diff));
				R_azi[1][2] = sin(toRadians(-(float)x_Diff)) * -1;
				R_azi[2][1] = sin(toRadians(-(float)x_Diff));
				R_azi[2][2] = cos(toRadians(-(float)x_Diff));
				R_long.setState(gmtl::Matrix44f::ORTHOGONAL);

			}

			glfwGetCursorPos(mainwindow, &x_Beg, &y_Beg);
		}
		
		if (distChange)
		{
			//printf("Distance has changed!\n");
			distChange = false;
			T1[2][3] = distAway1;
		}
		

		//View Calculations
		
		if (!activeCam)
		{
			//camChange = R_long * R_lat * T1;
			camChange = R_lat * R_long* T1;

			V = camChange;
			V.setState(gmtl::Matrix44f::AFFINE);
			gmtl::invert(V);

			World_to_Eye[0][3] = V[0][3];
			World_to_Eye[1][3] = V[1][3];
			World_to_Eye[2][3] = V[2][3];
			World_to_Eye.setState(gmtl::Matrix44f::TRANS);

			WtS[0][3] = V[0][3];
			WtS[1][3] = V[1][3];
			WtS[2][3] = V[2][3];
			WtS.setState(gmtl::Matrix44f::TRANS);
		}
		else
		{
			camChange = R_azi * R_ele * R_z * T2;
			
			gmtl::identity(T2);

			V = W * WtR * RtO * camChange;
			
			V.setState(gmtl::Matrix44f::AFFINE);
			gmtl::invert(V);

			World_to_Eye[0][3] = V[0][3];
			World_to_Eye[1][3] = V[1][3];
			World_to_Eye[2][3] = V[2][3];
			World_to_Eye.setState(gmtl::Matrix44f::TRANS);

			WtS[0][3] = V[0][3];
			WtS[1][3] = V[1][3];
			WtS[2][3] = V[2][3];
			WtS.setState(gmtl::Matrix44f::TRANS);
		}

		//Create MV Matrices and PVM Matrices
		Wy = W * Y_rot;
		Wx = W * Y_to_X_rot;
		Wz = W * Y_to_Z_rot;

		Ry = WtR * Y_rot;
		Rx = WtR * Y_to_X_rot;
		Rz = WtR * Y_to_Z_rot;

		Ox = WtR * RtO * Y_rot;
		Oy = WtR * RtO * Y_to_X_rot;
		Oz = WtR * RtO * Y_to_Z_rot;

		//LightSourcePosition = V * LightSourcePosition;
		
		//Load VM Matrices
		modelMat[0] = V * W;

		modelMat[1] = WtS;

		modelMat[2] = V * Wx;
		modelMat[3] = V * Wy;
		modelMat[4] = V * Wz;

		modelMat[5] = V * Rx;
		modelMat[6] = V * Ry;
		modelMat[7] = V * Rz;

		modelMat[8] = V * Ox;
		modelMat[9] = V * Oy;
		modelMat[20] = V * Oz;

		//Matrices with perspective added
		perspectiveMat[0] = P * V * W;

		perspectiveMat[1] = P * WtS;

		perspectiveMat[2] = P * Wx;
		perspectiveMat[3] = P * Wy;
		perspectiveMat[4] = P * Wz;
		
		perspectiveMat[5] = P * Rx;
		perspectiveMat[6] = P * Ry;
		perspectiveMat[7] = P * Rz;

		perspectiveMat[8] = P * Ox;
		perspectiveMat[9] = P * Oy;
		perspectiveMat[10] = P * Oz;
		
		//Per Bird VM/PVM calculations and insertion into VM/PVM arrays
		for (int i = 0; i < nb; i++)
		{
			B = W * birdArray[i].getRot() * birdOtB[i];
			prim1 = W * birdArray[i].getRot() * birdOtB[i] * BtJ1 * J1tP;
			prim2 = W * birdArray[i].getRot() * birdOtB[i] * BtJ2 * J2tP;

			lineOrigins[i] = gmtl::Vec4f(birdArray[i].getRot(0,3), birdArray[i].getRot(1, 3), birdArray[i].getRot(2,3), birdArray[i].getRot(3,3));// *gmtl::Vec4f(RtO[0][3], RtO[1][3], RtO[2][3], RtO[3][3]);

			modelMat[11 + (3*i)] =  V * B;
			modelMat[12 + (3*i)] =  V * prim1;
			modelMat[13 + (3*i)] =  V * prim2;

			perspectiveMat[11 + (3*i)] = P * V * B;
			perspectiveMat[12 + (3*i)] = P * V * prim1;
			perspectiveMat[13 + (3*i)] = P * V * prim2;
		}

		for (int i = 0; i < no; i++)
		{
			B = W * obstacleList[i] * RtO * OtB;

			modelMat[14 + (3 * (nb - 1)) + i] = V * B;

			perspectiveMat[14 + (3 * (nb - 1)) + i] = P * V * B;

		}

		// Call our function that performs opengl operations
		// This is where your changes for matrix and vertex manipulation should be. 
		display(mainwindow, program, vm_location, pvm_location, light_pos, LightSourcePosition, WtE_loc, World_to_Eye, perspectiveMat, VAOarray, modelMat, lineOrigins, textureID);

		/* Swap our buffers to make our changes visible */
		glfwSwapBuffers(mainwindow);
		glfwPollEvents();

		/*printf("W:\n");
		
			for (int o = 0; o < 4; o++)
			{
				for (int i = 0; i < 4; i++)
				{
					printf("%f\t", W[o][i]);
				}

				printf("\n");

			}
			printf("\n\n");

		printf("V:\n");

			for (int o = 0; o < 4; o++)
			{
				for (int i = 0; i < 4; i++)
				{
					printf("%f\t", V[o][i]);
				}

				printf("\n");

			}
			printf("\n\n");

			printf("camChange:\n");

			for (int o = 0; o < 4; o++)
			{
				for (int i = 0; i < 4; i++)
				{
					printf("%f\t", camChange[o][i]);
				}

				printf("\n");

			}
			printf("\n\n");
		
			printf("XDiff\t:\t%f\nYDiff\t:\t%f\n", lastXD, lastYD);*/

	}

	/* Delete our opengl context, destroy our window, and shutdown SDL */
	glfwDestroyWindow(mainwindow);

	glUseProgram(NULL);

	glDisableVertexAttribArray(pos_location);
	glDisableVertexAttribArray(color_location);

	glDeleteProgram(program);
	glDeleteVertexArrays(1, &VAO);

	return 0;
}