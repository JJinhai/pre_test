#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include "motor.cpp"

using namespace std;
const float K = 52.63;
const float K_angle = 1.5607;
#define nobs 20 // the number of the obstacle

struct Point {
    float x;
    float y;
};
void go_ahead(int fd, Point point1,Point point2){
  float distance = sqrt(pow((point1.x-point2.x),2)+pow((point1.y-point2.y),2));
  float t = distance / K;
  float diff = 30;
  float leftDistance = distance;
  Motor m1 = Motor(fd);
  int triggle = true;
  while(leftDistance > 0){
    //Avoid avoid1 = Avoid(fd);
    if(leftDistance < 30){
      m1.MotorGo(2650,2650,2200,2200, leftDistance / K*1000);
    }else{
      m1.MotorGo(2650,2650,2200,2200, diff / K*1000);
    }
    leftDistance -= diff;
    // if(leftDistance >0){
    //   if(avoid1.detect()){
    //     cout<< "obstacle front" << endl;
    //     if(triggle){
    //       avoid1.fakeMove();
    //       leftDistance -= 120;
    //     }
    //     triggle = false;
    //   }
    // }
  }
  cout<<  "go ahead distance" << distance << endl;
}

float steer(int fd, Point point1,Point point2,Point point3){
  float tanValueFinal = (point3.y - point2.y)/(point3.x - point2.x);
  float angleFinal = atan(tanValueFinal);
  float tanValueInit;
  float angleInit;
  if(point1.x == -1){ // it is the init state
    tanValueInit = 0;
    angleInit = M_PI/2;
  }else{
    tanValueInit = (point2.y - point1.y)/(point2.x - point1.x);
    angleInit = atan(tanValueInit);
  }

  float angle = angleFinal - angleInit;
  float t = abs(angle) / K_angle;
  if(angle < 0){
    Motor m1 = Motor(fd);
    m1.MotorGo(2200,2200,-2200,-2200,t*1000);
    cout<<  "right time" << t << endl;
  }else{
    Motor m1 = Motor(fd);
    m1.MotorGo(-2200,-2200,2200,2200,t*1000);
    cout<<  "left time" << t << endl;
  }
  return angle;
};

class Guidance{
  public:
   int fd;
    Point start;
    Point end;
    float steerDegree = 0;
   Guidance(int fdReference){
    fd = fdReference; // pca9685Setup(PIN_BASE, 0x40, HERTZ)
    if (fd < 0){
      printf("Error in setup\n");
    }
    start.x = 377;
    start.y = 50;
    end.x = 0;
    end.y = 254;
   }
   void go_to_garden(){
    run(start,end,true);
   }
   void back_home(){
    float angle = M_PI - steerDegree;
    float t = abs(angle) / K_angle;
    Motor m1 = Motor(fd);
    m1.MotorGo(-1000,-1000,1000,1000,t*1000);
    
    steerDegree = 0;
    run(end,start,false);
   }
   void run(Point start,Point end, bool isAhead){
    //// Define the artificial potential field parameters
    // Points of attraction
    int xa = end.x;
    int ya = end.y;

    int xo[nobs] = { 
      45,45,45,45,168,168,168,188,208,228,248,288,328,368,408,448,488,528,568,253
    };  // the position of obstacles
    int yo[nobs] = { 
      80,110,150,180,120,160,200,120,120,120,120,120,120,120,120,120,120,120,120,50
    };  // the position of obstacles
    
    // Size of obstacles (rho_0)
    int rho_0[nobs] = {
      30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,33
    };  // size of the obstacles

    // Constants.
    int Ka = 1; 
    int Kv = 50; 
    int Kr[nobs] = {
        600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600,600
    };
    int mass = 5;

    //// Integration step.
    // initial conditions
    float x0 = start.x; 
    float y0 = start.y;

    // Initialise simulation parameters
    float x = x0;
    float y = y0;
    float vx = 0;
    float vy = 0;
    bool terminate = false;
    float t = 0;
    float dt = 0.05;

    // Store variables for later assessment
    std::vector<std::vector<float> > traj;
    // std::vector<float>  traj = {};
    // float Fac = {}; 
    // float Frc = {}; 
    // float Fdc = {};
    // float Fc = {};
    
    while(!terminate){
        //// Check if reached destination
        float rho = sqrt(pow((xa-x),2)+pow((ya-y),2));

        if(rho < 1)
        {
            terminate = true;
        }
            
        
        //// Calculate the conservative forces
        // Attractive force (Fa)
        int Fa[2][1];
        Fa[0][0] = 0;
        Fa[1][0] = 0;

        Fa[0][0]=-Ka*(x-xa);
        Fa[1][0]=-Ka*(y-ya);

        // Fac = {Fac, Fa};
        // Repulsive force (Fr)
        int Fr[2][1];
        Fr[0][0] = 0;
        Fr[1][0] = 0;

        for(int i=0;i<nobs;i++){
            float rho_r = sqrt(pow((xo[i]-x),2)+pow((yo[i]-y),2));
            if(rho_r < rho_0[i]){
                Fr[0][0]=Fr[1][1]-Kr[i]*(rho_r-rho_0[i])*(x-xo[i])/rho_r;
                Fr[1][0]=Fr[2][1]-Kr[i]*(rho_r-rho_0[i])*(y-yo[i])/rho_r;
            }
        }

        // Frc = [Frc Fr];
        // Damping force (Fd)
        int Fd[2][1];
        Fd[0][0] = 0;
        Fd[1][0] = 0;

        Fd[0][0]=-Kv*vx;
        Fd[1][0]=-Kv*vy;

        // Fdc = [Fdc Fd];
        // Total force (F)

        int F[2][1];
        F[0][0]=Fa[0][0]+Fr[0][0]+Fd[0][0];
        F[1][0]=Fa[1][0]+Fr[1][0]+Fd[1][0];

        // Fc = [Fc F];
        // Velocity and Position (vx, vy, x, y) through integrating the equations

        vx=vx+dt*F[0][0]/mass;
        vy=vy+dt*F[1][0]/mass;
        x=x+vx*dt;
        y=y+vy*dt;
        t=t+dt;

        // traj = [traj [x;y]]; 
        std::vector<float> new_column = {x, y};
        if( traj.size() == 0){
          traj = {{x},{y}};
        }else{
          for (int i = 0; i < traj.size(); i++) {
            traj[i].push_back(new_column[i]);
          }
        }
    }

    // std::cout << "traj:\n";
    // for (size_t i = 0; i < traj[0].size(); ++i) {
    //   const auto& element = traj[0][i];
    //   const auto& element2 = traj[1][i];
    //   if(i%1000 == 0){
    //     std::cout << element << " ";
    //     std::cout << element2 << " " << endl;
    //   }
    // }

    std::vector<std::vector<float> > path;
    for (size_t i = 0; i < traj[0].size(); ++i) {
      if(i%100 == 0){
        auto& x = traj[0][i];
        auto& y = traj[1][i];
        std::vector<float> new_column = {x, y};
        if( path.size() == 0){
          path = {{x},{y}};
        }else{
          for (int i = 0; i < path.size(); i++) {
            path[i].push_back(new_column[i]);
          }
        }
      }
    }
    std::cout << "path:\n";
    for (size_t i = 1; i < path[0].size(); i++) {
      Point point1 = Point();
      if( i >= 2){
        const auto& element1_x = path[0][i-2];
        const auto& element1_y = path[1][i-2];
        point1.x = element1_x;
        point1.y = element1_y;
      }else{
        point1.x = -1;
        point1.y = -1;
      }

      const auto& element2_x = path[0][i-1];
      const auto& element2_y = path[1][i-1];
      Point point2 = Point();
      point2.x = element2_x;
      point2.y = element2_y;

      const auto& element3_x = path[0][i];
      const auto& element3_y = path[1][i];
      Point point3 = Point();
      point3.x = element3_x;
      point3.y = element3_y;

      cout << "position:" << point3.x << " " << point3.y << endl;
      go_ahead(fd,point2,point3);
      float d0 = steer(fd,point1,point2,point3);
      if(isAhead){
        steerDegree += d0;
      }
    }
    
  }
};


int main(void)
{
    if(wiringPiSetup()==-1){
      printf("setup wiringPi failed!\n");
      printf("please check your setup\n");
      return -1;
    }
    
    int fd = pca9685Setup(300, 0x40, 50);
    Guidance guidance1 = Guidance(fd);
    guidance1.go_to_garden();
    // delay(5000);
    // guidance1.back_home();
    return 0;
}
