/*
Capture a point cloud, with colors, using Zivid SDK, transform it to a Halcon
point cloud and save it using Halcon C++ SDK.
*/

#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <algorithm>
#include <chrono>
#include <iostream>

using namespace std::chrono;

void savePointCloud(const HalconCpp::HObjectModel3D &model,
                    const std::string &fileName) {
  model.WriteObjectModel3d(
      HalconCpp::HString{"ply"}, HalconCpp::HString{fileName.c_str()},
      HalconCpp::HString{"invert_normals"}, HalconCpp::HString{"false"});
}

HalconCpp::HObjectModel3D
zividToHalconPointCloud(const Zivid::PointCloud &pointCloud) {
  auto t0 = steady_clock::now();
  const auto width = pointCloud.width();
  const auto height = pointCloud.height();

  const auto pointsXYZ = pointCloud.copyPointsXYZ();
  const auto colorsRGBA = pointCloud.copyColorsRGBA();
  const auto normalsXYZ = pointCloud.copyNormalsXYZ();

  auto t1 = steady_clock::now();

  int numberOfValidPoints = std::count_if(
      pointsXYZ.data(), pointsXYZ.data() + pointsXYZ.size(),
      [](const Zivid::PointXYZ &point) { return (!point.isNaN()); });

  // Initializing HTuples which are later filled with data from the Zivid point
  // cloud. tupleXYZMapping is of shape [width, height, rows[], cols[]], and is
  // used for creating xyz mapping. See more at:
  // https://www.mvtec.com/doc/halcon/13/en/set_object_model_3d_attrib.html

  std::vector<float> pointsX(numberOfValidPoints);
  std::vector<float> pointsY(numberOfValidPoints);
  std::vector<float> pointsZ(numberOfValidPoints);
  std::vector<float> normalsX(numberOfValidPoints);
  std::vector<float> normalsY(numberOfValidPoints);
  std::vector<float> normalsZ(numberOfValidPoints);

  std::vector<int64_t> colorsR(numberOfValidPoints);
  std::vector<int64_t> colorsG(numberOfValidPoints);
  std::vector<int64_t> colorsB(numberOfValidPoints);

  std::vector<int64_t> xyzMapping(2 * numberOfValidPoints + 2);
  xyzMapping[0] = static_cast<int64_t>(width);
  xyzMapping[1] = static_cast<int64_t>(height);

  int validPointIndex = 0;

  for (size_t i = 0; i < height; ++i) {
    for (size_t j = 0; j < width; ++j) {
      const auto &point = pointsXYZ(i, j);
      const auto &normal = normalsXYZ(i, j);
      const auto &color = colorsRGBA(i, j);

      if (!isnan(point.x)) {
        pointsX[validPointIndex] = point.x;
        pointsY[validPointIndex] = point.y;
        pointsZ[validPointIndex] = point.z;
        colorsR[validPointIndex] = color.r;
        colorsG[validPointIndex] = color.g;
        colorsB[validPointIndex] = color.b;
        xyzMapping[2 + validPointIndex] = i;
        xyzMapping[2 + numberOfValidPoints + validPointIndex] = j;

        if (!isnan(normal.x)) {
          normalsX[validPointIndex] = normal.x;
          normalsY[validPointIndex] = normal.y;
          normalsZ[validPointIndex] = normal.z;
        }

        validPointIndex++;
      }
    }
  }

  HalconCpp::HTuple tuplePointsX(pointsX.data(), pointsX.size());
  HalconCpp::HTuple tuplePointsY(pointsY.data(), pointsY.size());
  HalconCpp::HTuple tuplePointsZ(pointsZ.data(), pointsZ.size());
  HalconCpp::HTuple tupleNormalsX(normalsX.data(), normalsX.size());
  HalconCpp::HTuple tupleNormalsY(normalsY.data(), normalsY.size());
  HalconCpp::HTuple tupleNormalsZ(normalsZ.data(), normalsZ.size());
  HalconCpp::HTuple tupleColorsR(colorsR.data(), colorsR.size());
  HalconCpp::HTuple tupleColorsG(colorsR.data(), colorsG.size());
  HalconCpp::HTuple tupleColorsB(colorsR.data(), colorsB.size());
  HalconCpp::HTuple tupleXYZMapping(xyzMapping.data(), xyzMapping.size());

  auto t2 = steady_clock::now();

  // std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
  HalconCpp::HObjectModel3D objectModel3D(tuplePointsX, tuplePointsY,
                                          tuplePointsZ);

  auto t3 = steady_clock::now();

  // std::cout << "Mapping ObjectModel3D data" << std::endl;
  HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "xyz_mapping", "object",
                                       tupleXYZMapping);

  auto t4 = steady_clock::now();

  // std::cout << "Adding normals to ObjectModel3D" << std::endl;
  HalconCpp::HTuple normalsAttribNames, normalsAttribValues;
  normalsAttribNames.Append("point_normal_x");
  normalsAttribNames.Append("point_normal_y");
  normalsAttribNames.Append("point_normal_z");

  normalsAttribValues.Append(tupleNormalsX);
  normalsAttribValues.Append(tupleNormalsY);
  normalsAttribValues.Append(tupleNormalsZ);

  HalconCpp::SetObjectModel3dAttribMod(objectModel3D, normalsAttribNames,
                                       "points", normalsAttribValues);

  auto t5 = steady_clock::now();

  // std::cout << "Adding RGB to ObjectModel3D" << std::endl;
  HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "red", "points",
                                       tupleColorsR);
  HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "green", "points",
                                       tupleColorsG);
  HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "blue", "points",
                                       tupleColorsB);

  auto t6 = steady_clock::now();

  /*std::cout << " " << std::endl;

  std::cout << duration_cast<milliseconds>(t1 - t0).count() << " ms - t0 - "
            << "copy points, colors, and normals" << std::endl;
  std::cout << duration_cast<milliseconds>(t2 - t1).count() << " ms - t1 - "
            << "Initializing HTuples" << std::endl;
  std::cout << duration_cast<milliseconds>(t3 - t2).count() << " ms - t2 - "
            << "Constructing ObjectModel3D based on XYZ data: " << std::endl;
  std::cout << duration_cast<milliseconds>(t4 - t3).count() << " ms - t3 - "
            << "Mapping ObjectModel3D data" << std::endl;
  std::cout << duration_cast<milliseconds>(t5 - t4).count() << " ms - t4 - "
            << "Adding normals to ObjectModel3D" << std::endl;
  std::cout << duration_cast<milliseconds>(t6 - t5).count() << " ms - t5 - "
            << "Adding RGB to ObjectModel3D" << std::endl;*/
  std::cout << duration_cast<milliseconds>(t6 - t1).count()
            << " ms - t1+t2+t3+t4+t5 - "
            << "Finalize Halcon" << std::endl;

  return objectModel3D;
}

int main() {
  try {
    auto t01 = steady_clock::now();
    HalconCpp::HSystem::GetSystem("is_license_valid");
    auto t02 = steady_clock::now();
    std::cout << duration_cast<milliseconds>(t02 - t01).count() << " ms - "
              << "Halcon license check" << std::endl;

    std::cout << "Connecting to camera" << std::endl;
    Zivid::Application zivid;
    auto camera = zivid.connectCamera();

    std::cout << "Configuring settings" << std::endl;
    const auto settings = Zivid::Settings{
        Zivid::Settings::Acquisitions{Zivid::Settings::Acquisition{
            Zivid::Settings::Acquisition::Aperture{5.66},
            Zivid::Settings::Acquisition::ExposureTime{
                std::chrono::microseconds{8333}}}},
        Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
        Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{5},
        Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
        Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{1.5}};

    for (int i = 0; i < 25; i++) {
      // std::cout << "Capturing frame" << std::endl;
      const auto frame = camera.capture(settings);
      const auto zividPointCloud = frame.pointCloud();

      // std::cout << "Converting to Halcon point cloud" << std::endl;
      const auto halconPointCloud = zividToHalconPointCloud(zividPointCloud);

      const auto pointCloudFile = "Zivid3D.ply";
      // std::cout << "Saving point cloud to file: " << pointCloudFile <<
      // std::endl;
      savePointCloud(halconPointCloud, pointCloudFile);
    }
  }

  catch (HalconCpp::HException &except) {
    std::cerr << "Error: " << except.ErrorMessage() << std::endl;
    return EXIT_FAILURE;
  }

  catch (const std::exception &e) {
    std::cerr << "Error: " << Zivid::toString(e) << std::endl;
    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
