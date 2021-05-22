#include <iostream>
#include <math.h>

int main()
{
  
double PI = 3.14159265358979323846;
double x = 102.0;
double y = 65.0;
double theta = 5* PI / 8;
double yaw_rate = PI / 8;
double delta_t = 0.1;
double velocity = 110;

x = x + (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
y = y + (velocity / yaw_rate) * (cos(theta) -cos (theta + (yaw_rate * delta_t)));
theta = theta + (yaw_rate * delta_t);

std::cout<<"X = "<<x<<" Y = "<<y<<" THETA = "<<theta<<"\n";
  
};
