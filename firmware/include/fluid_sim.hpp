#pragma once

#include <Eigen/Dense>
#include <cstdint>

#include "Eigen/src/Core/Matrix.h"
#include "types.hpp"

class FluidSim {
    public:
        FluidSim(uint8_t screenHeight, uint8_t screenWidth, uint8_t gridSize);
        void initialize();
        void simulate();

        Eigen::MatrixXf& getGrid();
        Eigen::MatrixXf& getVelocityU();
        Eigen::MatrixXf& getVelocityV();
        Eigen::MatrixXf& getPressure();


    private:
        void integrate();
        void projection();
        void printMatrix(Eigen::MatrixXf& matrix, const char* name);
        void computeDivergence(Eigen::MatrixXf& divergence);

        float getCellsTopVelocity(int i, int j);
        float getCellsBottomVelocity(int i, int j);
        float getCellsLeftVelocity(int i, int j);
        float getCellsRightVelocity(int i, int j);

        uint8_t screenHeight_;
        uint8_t screenWidth_;
        uint8_t gridSize_; 

        float gravity_;
        float dt_;

        Eigen::MatrixXf grid_;
        Eigen::MatrixXf pressure_;


        Eigen::MatrixXf v_; //velocity - vertical component
        Eigen::MatrixXf u_; //velocity - horozontial component

        static constexpr int TEXT_WIDTH = 6;  //Font8 width - debuggign purposes
        static constexpr int TEXT_HEIGHT = 3; //Font8 height
        static constexpr uint8_t GRID_START = 5; //Start grid 5 pixels out
};
