#include <gtest/gtest.h>
#include "fluid_sim.hpp"

class FLUIDTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 5x5 grid
        sim = new FluidSim(100, 100, 7);
    }

    void TearDown() override {
        delete sim;
    }

    FluidSim* sim;
};

TEST_F(FLUIDTest, GridInitialized_Given_EmptySevenBySeven_Equals_Expected) {
    //Arrange
    Eigen::MatrixXf expected = Eigen::MatrixXf::Ones(7, 7);

    //set the vertical walls on the side to 0
    for (int j = 0; j < expected.cols(); ++j) {
        //expected(0, j) = 0.0;
        expected(expected.rows() - 1, j) = 0.0;
    }
    //set the horozontial walls on the top and bottom to 0
    for (int i = 0; i < expected.rows(); ++i) {
        expected(i, 0) = 0.0;
        expected(i, expected.cols() - 1) = 0.0;
    }

    //Act & Assert
    EXPECT_TRUE(sim->getGrid().isApprox(expected));
}


TEST_F(FLUIDTest, VelocityIntegration_Given_OneTimeStep_Equals_Expected){
    // Arrange
    Eigen::MatrixXf expectedV(8, 7);
    expectedV << 
      0,           0,           0,           0,           0,           0,           0,
      0,  -2.494793e-6, -8.12309e-7, -1.81189e-6, -1.920087e-6, -4.98871e-7,         0,
      0,  -9.21663e-7, -2.987353e-6, -1.271888e-6, -2.045216e-6, -2.927233e-6,       0,
      0,  -2.885689e-6, -2.217963e-6, -3.622335e-6, -2.505903e-6, -2.451218e-6,      0,
      0,  -1.991603e-6, -1.759642e-6, -1.623133e-6, -2.560193e-6, -3.114945e-6,      0,
      0,   3.9698e-8,  -7.69565e-7,   9.53045e-7,   1.94556e-7,   4.45103e-7,        0,
      0,           0,           0,           0,           0,           0,           0,
      0,           0,           0,           0,           0,           0,           0;

    Eigen::MatrixXf expectedU(7, 8);
    expectedU << 
      0,           0,           0,           0,           0,           0,           0,           0,
      0,           0,  -1.683386e-6,   9.97520e-7,   1.23851e-7,  -1.430960e-6,      0,           0,
      0,           0,   3.84527e-7,  -7.19308e-7,   8.96624e-7,  -5.58297e-7,        0,           0,
      0,           0,  -3.01321e-7,   6.83801e-7,  -2.19395e-7,  -6.20982e-7,        0,           0,
      0,           0,  -5.33708e-7,   5.47902e-7,   7.20608e-7,  -6.2232e-8,         0,           0,
      0,           0,   2.89202e-7,  -1.17884e-6,   1.485157e-6, -3.03008e-7,        0,           0,
      0,           0,           0,           0,           0,           0,           0,           0;

    // Act
    sim->initialize();
    sim->simulate();

    // Assert
    ASSERT_TRUE(sim->getVelocityV().isApprox(expectedV, 1e-6)) << "Vertical velocity V does not match expected.";
    ASSERT_TRUE(sim->getVelocityU().isApprox(expectedU, 1e-6)) << "Horizontal velocity U does not match expected.";
}


TEST_F(FLUIDTest, Pressure_Given_OneTimeStep_Equals_Expected){
        //Arrange
    Eigen::MatrixXf expected = Eigen::MatrixXf::Zero(7, 7);
    expected << 
        0,           0,           0,           0,           0,           0,           0,
        0.0000,	-3.5638,	-1.1498,	-2.5814,	-2.7593,	-0.7200,	0.0000,	
        0.0000,	140137.9062,	140137.4219,	140138.4531,	140137.1562,	140138.0000,	0.0000,
        0.0000,	280276.5312,	280277.0000,	280276.2500,	280276.3438,	280277.4062,	0.0000,
        0.0000,	420416.5938,	420417.3438,	420416.5938,	420415.7500,	420415.8750,	0.0000,
        0.0000,	560559.3125,	560559.3125,	560560.8750,	560558.7500,	560559.1250,	0.0000,
        0,           0,           0,           0,           0,           0,           0,
    //Act
    sim->initialize();
    sim->simulate();

    //Assert
    EXPECT_TRUE(sim->getPressure().isApprox(expected));

}