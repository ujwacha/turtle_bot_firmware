#include "kinematics.cpp"
#include "driver.cpp"

class Robot {
public:
  Driver m0;
  Driver m1;
  Driver m2;
  Driver m3;
  
  Robot() {
    kin = Kinematics();
    
    

  }

private:
  Kinematics kin ;

};
