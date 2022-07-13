#include "Dxl.h"
#include "Matrix.h"
#include <ctime>

struct Ori {
	double a = 0.0;//yaw
	double b = 0.0;//pitch
	double r = 0.0;//roll
};

struct Angle {
	double q[6] = { 0, };
};

dynamixel::PortHandler* Dxl::port = dynamixel::PortHandler::getPortHandler("COM6");

Dxl dxl[6] = { Dxl(0), Dxl(1), Dxl(2), Dxl(3), Dxl(4), Dxl(5) };
Hmatrix h, h6g;
double L0 = 37, L1 = 66.25, L2 = 150, L3 = 83.75, L4 = 66.25, L5 = 41.50;

void delay(int ms);
Hmatrix fk(double q1, double q2, double q3, double q4, double q5, double q6);
Angle ik(double x, double y, double z, double a, double b, double r, dynamixel::PacketHandler* packet);
int move(Angle angle, dynamixel::PacketHandler* packet, int ms = 0);
double dpi(double q);
void getXYZABR(Hmatrix fkm);
Hmatrix setTcp(double x, double y, double z);
Ori getrpy(Hmatrix m);

int main(int arg, char* args[]) {
	dynamixel::PacketHandler* packet = dynamixel::PacketHandler::getPacketHandler(2.0);
	Dxl::init();
	h6g = setTcp(0, 0, 58);

	//initialize option
	for (int i = 0; i < 6; i++) {
		int m = dxl[i].read(Operating_Mode, packet);

		int mode = 3;
		if (m != mode) {// 16 - PWM 모드  3 - 위치제어모드
			dxl[i].write(Operating_Mode, mode, packet);
		}
		delay(5);

		//모터 위치 제한 설정
		dxl[i].max_pos_limit = dxl[i].read(Max_Position_Limit, packet);
		delay(5);
		dxl[i].min_pos_limit = dxl[i].read(Min_Position_Limit, packet);
		delay(5);

		//모터 속도 및 가속도 설정
		dxl[i].write(Profile_Velocity, 30, packet);
		dxl[i].write(Profile_Acceleration, 5, packet);
		delay(5);
	}

	//각 모터 PD gain 설정	Kp = Pgain/128	Kd = Dgain/16
	dxl[0].write(Position_P_Gain, 2800, packet);
	dxl[0].write(Position_D_Gain, 300, packet);

	dxl[1].write(Position_P_Gain, 7000, packet);
	dxl[1].write(Position_D_Gain, 3500, packet);

	dxl[2].write(Position_P_Gain, 6000, packet);
	dxl[2].write(Position_D_Gain, 2000, packet);

	dxl[3].write(Position_P_Gain, 2800, packet);
	dxl[3].write(Position_D_Gain, 500, packet);

	dxl[4].write(Position_P_Gain, 4300, packet);
	dxl[4].write(Position_D_Gain, 500, packet);

	dxl[5].write(Position_P_Gain, 1650, packet);
	dxl[5].write(Position_D_Gain, 200, packet);

	delay(100);

	//torque ON
	for (int i = 0; i < 6; i++) {
		dxl[i].write(Torque_Enable, 1, packet);
		delay(10);
	}

	//set initialize pose
	dxl[0].write(Goal_Position, 2048, packet);
	dxl[1].write(Goal_Position, 2048 + 512, packet);
	dxl[2].write(Goal_Position, 1024, packet);
	dxl[3].write(Goal_Position, 2048, packet);
	dxl[4].write(Goal_Position, 2048 - 512, packet);
	dxl[5].write(Goal_Position, 2048, packet);
	delay(1000);
	//-----------------------------------------------------------------------------------
	Angle angle, ikangle;
	std::string mode;

	while (1) {
		std::cout << "\nwork, EE, ik, reset, exit 중 선택 : ";
		std::cin >> mode;
		if (mode.compare("work") == 0) {
			ikangle = ik(0, 200, 280, 0, 0, -90, packet);//시작 자세
			move(ikangle, packet);

			//첫번째 음료 과정---------------------------------------------------
			ikangle = ik(-150, 150, 280, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-200, 200, 150, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-200, 200, 50, 45, 0, -90, packet);
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 15, packet);
			ikangle = ik(-240, 240, 50, 45, 0, -90, packet);
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 30, packet);
			ikangle = ik(-240, 240, 100, 45, 0, -90, packet);//잡아 올리기
			move(ikangle, packet);

			ikangle = ik(-200, 200, 100, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-180, 180, 180, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-150, 150, 240, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(0, 240, 240, 0, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-30, 260, 220, -25, 0, -90, packet);//따라내기 위한 자세
			move(ikangle, packet, 50);

			int pos = dxl[5].read(Present_Position, packet);
			dxl[5].write(Goal_Position, 2200, packet);
			while (1) {
				bool moving = false;
				int rx = dxl[5].read(Moving, packet);
				if (rx != 0) {
					moving = true;
				}

				delay(10);

				if (!moving) {
					break;
				}
			}

			delay(3000);

			dxl[5].write(Goal_Position, pos, packet);
			while (1) {
				bool moving = false;
				int rx = dxl[5].read(Moving, packet);
				if (rx != 0) {
					moving = true;
				}

				delay(10);

				if (!moving) {
					break;
				}
			}

			delay(2000);

			ikangle = ik(0, 240, 240, 0, 0, -90, packet);
			move(ikangle, packet, 50);

			ikangle = ik(-150, 150, 240, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-240, 240, 230, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-240, 240, 180, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-240, 240, 100, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-240, 240, 50, 45, 0, -90, packet);//내려놓기
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 15, packet);
			ikangle = ik(-230, 230, 50, 45, 0, -90, packet);
			move(ikangle, packet);

			delay(1000);

			ikangle = ik(-180, 180, 50, 45, 0, -90, packet);
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 30, packet);
			ikangle = ik(-200, 200, 200, 45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(-100, 100, 300, 45, 0, -90, packet);
			move(ikangle, packet);

			//두번째 음료 과정---------------------------------------------------
			ikangle = ik(150, 150, 280, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(200, 200, 150, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(200, 200, 50, -45, 0, -90, packet);
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 15, packet);
			ikangle = ik(240, 240, 50, -45, 0, -90, packet);
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 30, packet);
			ikangle = ik(240, 240, 100, -45, 0, -90, packet);//잡아 올리기
			move(ikangle, packet);

			ikangle = ik(200, 200, 100, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(180, 180, 180, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(150, 150, 240, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(0, 240, 240, 0, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(30, 260, 220, 25, 0, -90, packet);//따라내기 위한 자세
			move(ikangle, packet, 50);

			pos = dxl[5].read(Present_Position, packet);
			dxl[5].write(Goal_Position, 1800, packet);
			while (1) {
				bool moving = false;
				int rx = dxl[5].read(Moving, packet);
				if (rx != 0) {
					moving = true;
				}

				delay(10);

				if (!moving) {
					break;
				}
			}
			delay(3000);

			dxl[5].write(Goal_Position, pos, packet);
			while (1) {
				bool moving = false;
				int rx = dxl[5].read(Moving, packet);
				if (rx != 0) {
					moving = true;
				}

				delay(10);

				if (!moving) {
					break;
				}
			}

			delay(2000);

			ikangle = ik(0, 240, 240, 0, 0, -90, packet);
			move(ikangle, packet, 50);

			ikangle = ik(150, 150, 240, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(240, 240, 230, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(240, 240, 180, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(240, 240, 100, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(240, 240, 50, -45, 0, -90, packet);//내려놓기
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 15, packet);
			ikangle = ik(230, 230, 50, -45, 0, -90, packet);
			move(ikangle, packet);

			delay(1000);

			ikangle = ik(180, 180, 50, -45, 0, -90, packet);
			move(ikangle, packet);

			dxl[4].write(Profile_Velocity, 30, packet);
			ikangle = ik(200, 200, 200, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(100, 100, 300, -45, 0, -90, packet);
			move(ikangle, packet);

			delay(1500);

			ikangle = ik(0, 200, 300, 0, 0, -90, packet);
			move(ikangle, packet);

			//기본자세로 복귀
			dxl[0].write(Goal_Position, 2048, packet);
			dxl[1].write(Goal_Position, 2048 + 512, packet);
			dxl[2].write(Goal_Position, 1024, packet);
			dxl[3].write(Goal_Position, 2048, packet);
			dxl[4].write(Goal_Position, 2048 - 512, packet);
			dxl[5].write(Goal_Position, 2048, packet);
		}
		else if (mode.compare("EE") == 0) {
			Angle tmp;
			for (int i = 0; i < 6; i++) {
				tmp.q[i] = dxl[i].read(Present_Position, packet);
				tmp.q[i] = pos2angle(tmp.q[i]);
			}
			Hmatrix m = fk(tmp.q[0], tmp.q[1], tmp.q[2], tmp.q[3], tmp.q[4], tmp.q[5]);
			getXYZABR(m);
		}
		else if (mode.compare("ik") == 0) {
			double ik_x, ik_y, ik_z, ik_a, ik_b, ik_r;
			std::cout << "EE x,y,z,a,b,r : ";
			std::cin >> ik_x >> ik_y >> ik_z >> ik_a >> ik_b >> ik_r;

			for (int i = 0; i < 6; i++) {
				std::cout << "ID - " << i << " " << dxl[i].read(Present_Position, packet);
			}

			ikangle = ik(ik_x, ik_y, ik_z, ik_a, ik_b, ik_r, packet);// 시간 재면 143정도 나옴 clock() 기준
			move(ikangle, packet);
		}
		else if (mode.compare("test") == 0) {
			ikangle = ik(0, 300, 250, 0, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(0, 300, 150, 0, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(0, 300, 150, -45, 0, -90, packet);
			move(ikangle, packet);

			ikangle = ik(0, 300, 150, 0, 0, -90, packet);
			move(ikangle, packet);
		}
		else if (mode.compare("reset") == 0) {
			dxl[0].write(Goal_Position, 2048, packet);
			dxl[1].write(Goal_Position, 2048 + 512, packet);
			dxl[2].write(Goal_Position, 1024, packet);
			dxl[3].write(Goal_Position, 2048, packet);
			dxl[4].write(Goal_Position, 2048 - 512, packet);
			dxl[5].write(Goal_Position, 2048, packet);
		}
		else if (mode.compare("exit") == 0) {
			std::cout << "종료" << std::endl;
			break;
		}
		else {
			std::cout << "잘못 입력" << std::endl;
		}
	}

	//end
	for (int i = 0; i < 6; i++) {
		dxl[i].disable(packet);
		delay(10);
	}

	Dxl::close();

	return 0;
}

void delay(int ms) {
	if (ms < 0) return;

	clock_t start = clock();

	long duration = 0;
	do {
		duration = clock() - start;
	} while (duration < ms);
}

Hmatrix fk(double q1, double q2, double q3, double q4, double q5, double q6)
{
	Hmatrix h01 = h.trans(0, 0, L0) * h.rotZ(q1);
	Hmatrix h12 = h.trans(0, 0, L1) * h.rotZ(90) * h.rotX(90) * h.rotZ(q2);
	Hmatrix h23 = h.trans(0, L2, 0) * h.rotZ(q3);
	Hmatrix h34 = h.trans(0, L3, 0) * h.rotY(-90) * h.rotX(-90) * h.rotZ(q4);
	Hmatrix h45 = h.trans(0, 0, L4) * h.rotZ(90) * h.rotX(90) * h.rotZ(q5);
	Hmatrix h56 = h.trans(0, L5, 0) * h.rotY(-90) * h.rotX(-90) * h.rotZ(q6);

	return h01 * h12 * h23 * h34 * h45 * h56 * h6g;
}

void getXYZABR(Hmatrix fkm)
{
	std::cout << "EE info" << std::endl;
	std::cout << "x : " << fkm.e[0][3] << std::endl;
	std::cout << "y : " << fkm.e[1][3] << std::endl;
	std::cout << "z : " << fkm.e[2][3] << std::endl;

	double a = atan2(fkm.e[1][0], fkm.e[0][0]);
	double b = atan2(-1 * fkm.e[2][0], sqrt(pow(fkm.e[0][0], 2) + pow(fkm.e[1][0], 2)));
	double r = atan2(fkm.e[2][1], fkm.e[2][2]);

	std::cout << "a : " << rad2deg(a) << std::endl;
	std::cout << "b : " << rad2deg(b) << std::endl;
	std::cout << "r : " << rad2deg(r) << std::endl;
}

Hmatrix setTcp(double x, double y, double z)
{
	//기본으로 모터 혼에 EE 가 붙음
	//6번축 + 19mm(z축);
	Hmatrix res;
	res.e[0][3] = x;
	res.e[1][3] = y;
	res.e[2][3] = z + 19;

	return res;
}

Ori getrpy(Hmatrix m)
{
	Ori res;

	res.a = atan2(m.e[1][0], m.e[0][0]);
	res.b = atan2(-1 * m.e[2][0], sqrt(pow(m.e[0][0], 2) + pow(m.e[1][0], 2)));
	res.r = atan2(m.e[2][1], m.e[2][2]);

	return res;
}

Angle ik(double x, double y, double z, double a, double b, double r, dynamixel::PacketHandler* packet)
{
	Hmatrix h0g = h.trans(x, y, z) * h.rotZ(a) * h.rotY(b) * h.rotX(r);
	Hmatrix h06 = h0g * h6g.inv();

	x = h06.e[0][3];
	y = h06.e[1][3];
	z = h06.e[2][3];

	Ori tmp = getrpy(h06);

	double Xd = x;
	double Yd = y;
	double Zd = z;
	a = tmp.a;
	b = tmp.b;
	r = tmp.r;
	double R21 = sin(a) * cos(b);
	double R31 = -sin(b);
	double R32 = cos(b) * sin(r);

	double X = 0;
	double Y = 0;
	double Z = 0;
	double r21 = 0;
	double r31 = 0;
	double r32 = 0;

	Angle angle;

	for (int i = 0; i < 6; i++) {
		angle.q[i] = pos2angle(dxl[i].read(Present_Position, packet));
		std::cout << "id - " << i << " angle - " << angle.q[i] << " ";
		angle.q[i] = deg2rad(angle.q[i]);
	}
	std::cout << "\n";

	delay(10);

	Matrix J(6, 6), dq(6, 1), dx(6, 1), inv_J(6, 6);
	double alpha = 0.01;

	for (int i = 0; i < 1000; i++) {
		double c1 = cos(angle.q[0]);
		double c2 = cos(angle.q[1]);
		double c3 = cos(angle.q[2]);
		double c4 = cos(angle.q[3]);
		double c5 = cos(angle.q[4]);
		double c6 = cos(angle.q[5]);

		double s1 = sin(angle.q[0]);
		double s2 = sin(angle.q[1]);
		double s3 = sin(angle.q[2]);
		double s4 = sin(angle.q[3]);
		double s5 = sin(angle.q[4]);
		double s6 = sin(angle.q[5]);
		double s23 = sin(angle.q[1] + angle.q[2]);//sin(q2+q3)

		X = L2 * s1 * s2 + L3 * c2 * s1 * s3 + L3 * c3 * s1 * s2 + L4 * c2 * s1 * s3 + L4 * c3 * s1 * s2 + L5 * c1 * s4 * s5 + L5 * c2 * c5 * s1 * s3 + L5 * c3 * c5 * s1 * s2 - L5 * c4 * s1 * s2 * s3 * s5 + L5 * c2 * c3 * c4 * s1 * s5;
		Y = L5 * s1 * s4 * s5 - L3 * c1 * c2 * s3 - L3 * c1 * c3 * s2 - L4 * c1 * c2 * s3 - L4 * c1 * c3 * s2 - L2 * c1 * s2 - L5 * c1 * c2 * c5 * s3 - L5 * c1 * c3 * c5 * s2 + L5 * c1 * c4 * s2 * s3 * s5 - L5 * c1 * c2 * c3 * c4 * s5;
		Z = L0 + L1 + L2 * c2 + L3 * c2 * c3 + L4 * c2 * c3 - L3 * s2 * s3 - L4 * s2 * s3 + L5 * c2 * c3 * c5 - L5 * c5 * s2 * s3 - L5 * c2 * c4 * s3 * s5 - L5 * c3 * c4 * s2 * s5;

		r21 = c4 * c6 * s1 - c5 * s1 * s4 * s6 + c1 * c2 * c3 * c6 * s4 - c1 * c6 * s2 * s3 * s4 - c1 * c2 * s3 * s5 * s6 - c1 * c3 * s2 * s5 * s6 + c1 * c2 * c3 * c4 * c5 * s6 - c1 * c4 * c5 * s2 * s3 * s6;
		r31 = c2 * c6 * s3 * s4 + c3 * c6 * s2 * s4 + c2 * c3 * s5 * s6 - s2 * s3 * s5 * s6 + c2 * c4 * c5 * s3 * s6 + c3 * c4 * c5 * s2 * s6;
		r32 = c2 * c3 * c6 * s5 - c2 * s3 * s4 * s6 - c3 * s2 * s4 * s6 - c6 * s2 * s3 * s5 + c2 * c4 * c5 * c6 * s3 + c3 * c4 * c5 * c6 * s2;

		J(0, 0) = (c1 * c2 * c5 * s3 - s1 * s4 * s5 + c1 * c3 * c5 * s2 + c1 * c2 * c3 * c4 * s5 - c1 * c4 * s2 * s3 * s5) * L5 + (c1 * s2) * L2 + (c1 * c2 * s3 + c1 * c3 * s2) * L3 + (c1 * c2 * s3 + c1 * c3 * s2) * L4;
		J(0, 1) = (-s1 * (c5 * s2 * s3 - c2 * c3 * c5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5)) * L5 + (s1 * (c2 * c3 - s2 * s3)) * L3 + (s1 * (c2 * c3 - s2 * s3)) * L4 + (c2 * s1) * L2;
		J(0, 2) = (-s1 * (c5 * s2 * s3 - c2 * c3 * c5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5)) * L5 + (s1 * (c2 * c3 - s2 * s3)) * L3 + (s1 * (c2 * c3 - s2 * s3)) * L4;
		J(0, 3) = (s5 * (c1 * c4 - c2 * c3 * s1 * s4 + s1 * s2 * s3 * s4)) * L5;
		J(0, 4) = (c1 * c5 * s4 - c2 * s1 * s3 * s5 - c3 * s1 * s2 * s5 + c2 * c3 * c4 * c5 * s1 - c4 * c5 * s1 * s2 * s3) * L5;
		J(0, 5) = 0;

		J(1, 0) = (c1 * s4 * s5 + c2 * c5 * s1 * s3 + c3 * c5 * s1 * s2 + c2 * c3 * c4 * s1 * s5 - c4 * s1 * s2 * s3 * s5) * L5 + (c2 * s1 * s3 + c3 * s1 * s2) * L3 + (c2 * s1 * s3 + c3 * s1 * s2) * L4 + (s1 * s2) * L2;
		J(1, 1) = (c1 * (c5 * s2 * s3 - c2 * c3 * c5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5)) * L5 + (-c1 * (c2 * c3 - s2 * s3)) * L3 + (-c1 * (c2 * c3 - s2 * s3)) * L4 + (-c1 * c2) * L2;
		J(1, 2) = (c1 * (c5 * s2 * s3 - c2 * c3 * c5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5)) * L5 + (-c1 * (c2 * c3 - s2 * s3)) * L3 + (-c1 * (c2 * c3 - s2 * s3)) * L4;
		J(1, 3) = (s5 * (c4 * s1 + c1 * c2 * c3 * s4 - c1 * s2 * s3 * s4)) * L5;
		J(1, 4) = (c5 * s1 * s4 + c1 * c2 * s3 * s5 + c1 * c3 * s2 * s5 - c1 * c2 * c3 * c4 * c5 + c1 * c4 * c5 * s2 * s3) * L5;
		J(1, 5) = 0;

		J(2, 0) = 0;
		J(2, 1) = (c4 * s2 * s3 * s5 - c3 * c5 * s2 - c2 * c3 * c4 * s5 - c2 * c5 * s3) * L5 + (-c2 * s3 - c3 * s2) * L3 + (-c2 * s3 - c3 * s2) * L4 + (-s2) * L2;
		J(2, 2) = (c4 * s2 * s3 * s5 - c3 * c5 * s2 - c2 * c3 * c4 * s5 - c2 * c5 * s3) * L5 + (-c2 * s3 - c3 * s2) * L3 + (-c2 * s3 - c3 * s2) * L4;
		J(2, 3) = (s23 * s4 * s5) * L5;
		J(2, 4) = (s2 * s3 * s5 - c2 * c3 * s5 - c2 * c4 * c5 * s3 - c3 * c4 * c5 * s2) * L5;
		J(2, 5) = 0;

		J(3, 0) = 0;
		J(3, 1) = c2 * c3 * c6 * s4 - c6 * s2 * s3 * s4 - c2 * s3 * s5 * s6 - c3 * s2 * s5 * s6 + c2 * c3 * c4 * c5 * s6 - c4 * c5 * s2 * s3 * s6;
		J(3, 2) = c2 * c3 * c6 * s4 - c6 * s2 * s3 * s4 - c2 * s3 * s5 * s6 - c3 * s2 * s5 * s6 + c2 * c3 * c4 * c5 * s6 - c4 * c5 * s2 * s3 * s6;
		J(3, 3) = s23 * (c4 * c6 - c5 * s4 * s6);
		J(3, 4) = -s6 * (c5 * s2 * s3 - c2 * c3 * c5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5);
		J(3, 5) = c2 * c3 * c6 * s5 - c2 * s3 * s4 * s6 - c3 * s2 * s4 * s6 - c6 * s2 * s3 * s5 + c2 * c4 * c5 * c6 * s3 + c3 * c4 * c5 * c6 * s2;

		J(4, 0) = 0;
		J(4, 1) = s2 * s3 * s4 * s6 - c2 * c6 * s3 * s5 - c3 * c6 * s2 * s5 - c2 * c3 * s4 * s6 + c2 * c3 * c4 * c5 * c6 - c4 * c5 * c6 * s2 * s3;
		J(4, 2) = s2 * s3 * s4 * s6 - c2 * c6 * s3 * s5 - c3 * c6 * s2 * s5 - c2 * c3 * s4 * s6 + c2 * c3 * c4 * c5 * c6 - c4 * c5 * c6 * s2 * s3;
		J(4, 3) = -s23 * (c4 * s6 + c5 * c6 * s4);
		J(4, 4) = -c6 * (c5 * s2 * s3 - c2 * c3 * c5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5);
		J(4, 5) = s2 * s3 * s5 * s6 - c3 * c6 * s2 * s4 - c2 * c3 * s5 * s6 - c2 * c6 * s3 * s4 - c2 * c4 * c5 * s3 * s6 - c3 * c4 * c5 * s2 * s6;

		J(5, 0) = c1 * c4 * c6 - c1 * c5 * s4 * s6 - c2 * c3 * c6 * s1 * s4 + c6 * s1 * s2 * s3 * s4 + c2 * s1 * s3 * s5 * s6 + c3 * s1 * s2 * s5 * s6 - c2 * c3 * c4 * c5 * s1 * s6 + c4 * c5 * s1 * s2 * s3 * s6;
		J(5, 1) = -c1 * (c2 * c6 * s3 * s4 + c3 * c6 * s2 * s4 + c2 * c3 * s5 * s6 - s2 * s3 * s5 * s6 + c2 * c4 * c5 * s3 * s6 + c3 * c4 * c5 * s2 * s6);
		J(5, 2) = -c1 * (c2 * c6 * s3 * s4 + c3 * c6 * s2 * s4 + c2 * c3 * s5 * s6 - s2 * s3 * s5 * s6 + c2 * c4 * c5 * s3 * s6 + c3 * c4 * c5 * s2 * s6);
		J(5, 3) = c1 * c2 * c3 * c4 * c6 - c4 * c5 * s1 * s6 - c6 * s1 * s4 - c1 * c4 * c6 * s2 * s3 - c1 * c2 * c3 * c5 * s4 * s6 + c1 * c5 * s2 * s3 * s4 * s6;
		J(5, 4) = -s6 * (c1 * c2 * c5 * s3 - s1 * s4 * s5 + c1 * c3 * c5 * s2 + c1 * c2 * c3 * c4 * s5 - c1 * c4 * s2 * s3 * s5);
		J(5, 5) = c1 * s2 * s3 * s4 * s6 - c5 * c6 * s1 * s4 - c1 * c2 * c3 * s4 * s6 - c1 * c2 * c6 * s3 * s5 - c1 * c3 * c6 * s2 * s5 - c4 * s1 * s6 + c1 * c2 * c3 * c4 * c5 * c6 - c1 * c4 * c5 * c6 * s2 * s3;

		inv_J = J.inv();
		/*
		dq = inv_J * dx * alpha;

		dx(0, 0) = Xd - X;
		dx(1, 0) = Yd - Y;
		dx(2, 0) = Zd - Z;
		dx(3, 0) = R31 - r31;
		dx(4, 0) = R32 - r32;
		dx(5, 0) = R21 - r21;
		*/

		dx(0, 0) = Xd - X;
		dx(1, 0) = Yd - Y;
		dx(2, 0) = Zd - Z;
		dx(3, 0) = R31 - r31;
		dx(4, 0) = R32 - r32;
		dx(5, 0) = R21 - r21;

		dq = inv_J * dx * alpha;

		angle.q[0] += dq(0, 0);
		angle.q[1] += dq(1, 0);
		angle.q[2] += dq(2, 0);
		angle.q[3] += dq(3, 0);
		angle.q[4] += dq(4, 0);
		angle.q[5] += dq(5, 0);
	}
	angle.q[0] = dpi(rad2deg(angle.q[0]));
	angle.q[1] = dpi(rad2deg(angle.q[1]));
	angle.q[2] = dpi(rad2deg(angle.q[2]));
	angle.q[3] = dpi(rad2deg(angle.q[3]));
	angle.q[4] = dpi(rad2deg(angle.q[4]));
	angle.q[5] = dpi(rad2deg(angle.q[5]));

	for (int i = 0; i < 6; i++) {
		std::cout << "id - " << i << " angle - " << angle.q[i] << " ";
	}
	std::cout << "\n";

	return angle;
}

int move(Angle angle, dynamixel::PacketHandler* packet, int ms)
{
	Angle pos;
	bool disable = false;

	for (int i = 0; i < 6; i++) {
		pos.q[i] = angle2pos(angle.q[i]);
		if (!dxl[i].checkMove(pos.q[i])) {
			std::cout << "ID - " << i << " 허용되지 않은 각도입니다 " << pos2angle(pos.q[i]) << std::endl;
			disable = true;
		}
	}

	if (disable) {
		return 0;
	}

	dxl[5].write(Goal_Position, pos.q[5], packet);
	delay(ms);
	dxl[4].write(Goal_Position, pos.q[4], packet);
	dxl[3].write(Goal_Position, pos.q[3], packet);
	dxl[2].write(Goal_Position, pos.q[2], packet);
	dxl[1].write(Goal_Position, pos.q[1], packet);
	dxl[0].write(Goal_Position, pos.q[0], packet);
	delay(100);

	while (1) {
		bool moving = false;
		for (int i = 0; i < 6; i++) {
			int rx = dxl[i].read(Moving, packet);
			if (rx != 0) {
				moving = true;
			}
		}

		delay(10);

		if (!moving) {
			break;
		}
	}

	return 1;
}

double dpi(double q)
{
	while (q > 180) {
		q -= 360;
	}
	while (q < -180) {
		q += 360;
	}
	return q;
}