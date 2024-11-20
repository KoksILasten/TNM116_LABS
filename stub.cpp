
#include <Eigen/Eigen>

#include <opencv2/calib3d.hpp> //doesnt work atm will should be fixed later ;)
#include <opencv2/core/eigen.hpp>
#include "eigenSolvePnP.hh"

#include <iostream>
//#include <vector>
//#include <typeinfo>

// Task 6:
//root mean square error
float RMSE(Eigen::Matrix4f M,
    std::vector<Eigen::Vector3f> realWorldPoints,
    std::vector<Eigen::Vector3f> trackerPoints) {

    float errorSum = 0.0f;
    int numPoints = trackerPoints.size();

    for (int i = 0; i < numPoints;  i++) {

        //pad tracker points
        Eigen::Vector4f trackerPointHomogenius = trackerPoints[i].homogeneous();

        //Transform the tracker points using transform matrix M
        Eigen::Vector4f transformedPoint = M * trackerPointHomogenius;

        //extract pos of transformed point
        Eigen::Vector3f transformedPos = transformedPoint.head<3>();

        //calc squared error 
        float error = (transformedPos - realWorldPoints[i]).squaredNorm();
        errorSum += error;
    }

    return std::sqrt(errorSum / numPoints);
} //RMSE





int main(int argc, char *argv[]) {

  Eigen::Matrix4f M_wand;
  M_wand << 0.853553, 0.146447, 0.5, 1,
            0.146447, 0.853553, -0.5, 1, 
            -0.5, 0.5, 0.707107, 1,
             0, 0, 0, 1;

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix <<   0.6, 0.0, 0.3,
                    0.0, 0.6, 0.3,
                    0.0, 0.0, 1.0;

  Eigen::Matrix4f headMatrix;
  headMatrix << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 1.0, 
                0.0, 0.0, 1.0, -2.0,
                0.0, 0.0, 0.0, 1.0;

  Eigen::Vector2f cameraPos = cameraMatrix.topRightCorner(2, 1);
  //Eigen::Quaternionf cameraRot = (Eigen::Quaternionf)cameraMatrix.block<2, 2>(0, 0);

  Eigen::Vector3f headPos = headMatrix.topRightCorner(3, 1);
  Eigen::Quaternionf headRot = (Eigen::Quaternionf)headMatrix.block<3, 3>(0, 0);


  //Task 1: Extract Pose Representation
  std::cout << "Wand Matrix:\n " << M_wand<< std::endl;

  Eigen::Quaternionf wandRot = (Eigen::Quaternionf)M_wand.block<3, 3>(0, 0); //3x3 in the beginning
  std::cout << "Wand Rotation in quaterions:\n" << wandRot << std::endl;

  Eigen::Vector3f wandPos = M_wand.topRightCorner(3, 1);
  std::cout << "Wand Pos:\n" << wandPos << std::endl;

  std::cout << std::endl;
  
  // Task 2: 
  Eigen::Vector4f wandEndpoint1;
  Eigen::Vector3f wandEndpoint2;

  wandEndpoint1 = M_wand * Eigen::Vector4f(0, 0, -30, 1); //original matrix ver
  wandEndpoint2 = wandPos + wandRot * Eigen::Vector3f(0, 0, -30); //pose matrix ver

  std::cout << "Wand tip:\n" << wandEndpoint1 << std::endl;
  std::cout << "Wand tip2:\n" << wandEndpoint2 << std::endl;
 
  std::cout << std::endl;
  //Task 3: 
  Eigen::Vector3f headtoWand = wandPos - headPos;
  headtoWand.normalize();

  Eigen::Quaternionf rotDiff;
  rotDiff.FromTwoVectors(wandEndpoint2, headtoWand);

  Eigen::Quaternionf ROT = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), headtoWand);
  //auto rotDiff = rot * Q_wand.conjugate()

  Eigen::AngleAxisf rotDiffAngleAxis(rotDiff);

  std::cout << "Head Vector:\n"
            << "Angle: " << rotDiffAngleAxis.angle() << " degrees" << std::endl
            << "Axis:\n" << rotDiffAngleAxis.axis() << std::endl;

  std::cout << std::endl;
  //Task 4:
  float PI = 3.1415;
  // 90 degree rot around Y
  //   ^ y+
  //   |
  //   |____> x+
  //   /
  //  /
  // z+
  auto Y = Eigen::Vector3f::UnitY();

  //Eigen::AngleAxisf adjusttoYRot(PI, Eigen::Vector3f(0, 1, 0));
  //Eigen::Quaternionf adjustedRot = rotDiff * adjusttoYRot;
  Eigen::Quaternionf adjustedRot = Eigen::Quaternionf::FromTwoVectors(ROT * Y, Y);

  // new rotation equals the Y rotation * the rotation differance
  auto newRot = adjustedRot * rotDiff;
  
  std::cout << "Adjusted Rotation:\n" << newRot << std::endl;
  
  std::cout << std::endl;
  //Task 5:
  Eigen::Quaternionf meanOri = Eigen::Quaternionf::Identity();

  for (int i = 0; i < 100; i++) {
    Eigen::Quaternionf randomRot = Eigen::Quaternionf::UnitRandom();
    
    //linear comb
    meanOri *= randomRot;
  }
  meanOri.normalize();
  Eigen::AngleAxisf meanOriAngleAxis(meanOri);

    std::cout << "Mean Rotation:\n"
            << "Angle: " << meanOriAngleAxis.angle() << " degrees" << std::endl
            << "Axis:\n" << meanOriAngleAxis.axis() << std::endl;

  std::cout << std::endl;

  std::vector<Eigen::Vector3f> reg_samples_world {
      {0, 0, 0}, 
      {2, 0, 0}, 
      {0, 0, 2}, 
      {2, 0, 2}};
  std::vector<Eigen::Vector3f> reg_samples_tracker {
      {2.00511, 2.99263, 3.99},
      {3.70149, 4.00066, 4.29207},
      {2.29648, 2.00358, 5.69805},
      {4.00039, 2.99767, 6.00869}};

  std::vector<Eigen::Vector3f> proj_samples_world {
      Eigen::Vector3f(0.0, 0.0, 0.0),
      Eigen::Vector3f(0.0, 0.2, 0.0),
      Eigen::Vector3f(0.2, 0.0, 0.0),
      Eigen::Vector3f(0.2, 0.2, 0.0),
      Eigen::Vector3f(0.0, 0.0, 0.2),
      Eigen::Vector3f(0.0, 0.2, 0.2),
      Eigen::Vector3f(0.2, 0.0, 0.2),
      Eigen::Vector3f(0.2, 0.2, 0.2)};
  std::vector<Eigen::Vector2f> proj_samples_image {{0.310564, 0.302949},
                                                   {0.310872, 0.27505},
                                                   {0.279655, 0.308101},
                                                   {0.279053, 0.279963},
                                                   {0.32098, 0.320068},
                                                   {0.32162, 0.291391},
                                                   {0.288801, 0.325701},
                                                   {0.288454, 0.296777}};

  //Task 6
  float rmse = RMSE(Eigen::Matrix4f::Identity(), reg_samples_tracker, reg_samples_world);
  std::cout << "RMSE(drift): " << rmse << std::endl;
  std::cout << std::endl;

  //Task 7
  //Registration matrix: from tracking to workspace coords. 
  //messy but works :P
  Eigen::Vector3f temp1 = (reg_samples_world[1] - reg_samples_world[0]);
  Eigen::Vector3f temp2 = (reg_samples_world[2] - reg_samples_world[0]);
  auto a = temp1.cross(temp2);
  //std::cout << a;

  Eigen::Vector3f X4World = reg_samples_world[0] + a;

  temp1 = (reg_samples_tracker[1] - reg_samples_tracker[0]);
  temp2 = (reg_samples_tracker[2] - reg_samples_tracker[0]); 
  auto b = temp1.cross(temp2);
  //std::cout << temp1;

  Eigen::Vector3f X4Tracker = reg_samples_tracker[0] + b;
  

  Eigen::Matrix4f B;
  B << reg_samples_world[0].homogeneous(), 
      reg_samples_world[1].homogeneous(),
      reg_samples_world[2].homogeneous(),
      X4World.homogeneous();
  std::cout << "B: World sample matrix:\n" << B << std::endl;

    Eigen::Matrix4f A;
  A << reg_samples_tracker[0].homogeneous(),
      reg_samples_tracker[1].homogeneous(),
      reg_samples_tracker[2].homogeneous(),
      X4Tracker.homogeneous();
  std::cout << "A: Tracker sample matrix:\n" << A << std::endl;

  //possible floating point error -- nvm is tracker noise and it gives scaling
  A.fullPivHouseholderQr().solve(B);

  Eigen::Matrix4f Treg = B * A.inverse();
  std::cout << "Treg:\n" << Treg << std::endl;

  std::cout << std::endl;
  //Task 9

  Eigen::Vector3f ext; //pixel coords

  for (size_t i = 0; i < proj_samples_world.size(); i++) {
    ext = cameraMatrix * (headMatrix * proj_samples_world[i].homogeneous()).hnormalized();
    std::cout << "Real value for index " << i << ":\n" << ext << std::endl;
  }

  //2D hyperplane in 3D space being projected and devided by scaling(w) hnormalized()
  std::cout << std::endl;
  //Task 10: Estimate the Registation Matrix
  Eigen::Vector3f tvec;
  Eigen::Quaternionf rq;

  solvePnP(proj_samples_world,
           proj_samples_image,
           cameraMatrix,
           rq,
           tvec);

  std::cout << "Approx registation pos:\n" << tvec << "\napprox rotation:\n" << rq << std::endl;

  //skipping the last last step

 /* cv::Mat rvec, tvec;

  cv::Mat cvCamMat;
  cv::cv2eigen(cvCamMat, cameraMatrix);

  cv::solvePnP(
      proj_samples_world, proj_samples_image, cvCamMat, cv::Mat() , rvec, tvec);
  std::cout << rvec << "\n" << tvec;*/

} //Main
