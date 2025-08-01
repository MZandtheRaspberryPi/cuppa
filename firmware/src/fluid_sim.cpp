#include "fluid_sim.hpp"

#include <Eigen/Dense>
// extern "C" {
//   #include "GUI_Paint.h"
//   #include "LCD_0in99.h"
// }


FluidSim::FluidSim(uint8_t screenHeight, uint8_t screenWidth, uint8_t gridSize)
{
    screenHeight_ = screenHeight;
    screenWidth_ = screenWidth;
    gridSize_ = gridSize;

    dt_ = 0.01;
    gravity_ = -9.81;
    initialize();
}

Eigen::MatrixXf& FluidSim::getGrid() {
   return grid_; 
}

Eigen::MatrixXf& FluidSim::getVelocityU() {
   return u_; 
}Eigen::MatrixXf& FluidSim::getVelocityV() {
   return v_; 
}Eigen::MatrixXf& FluidSim::getPressure() {
   return pressure_; 
}

void FluidSim::initialize(){
    grid_ = Eigen::MatrixXf::Ones(gridSize_, gridSize_);
    for (int j = 0; j < grid_.cols(); j++)
    {
     //   grid_(0,j) = 0.0;
        grid_(grid_.rows()-1,j) = 0.0;

    }
    for (int i = 0; i < grid_.rows(); i++)
    {
        grid_(i,0) = 0.0;
        grid_(i,grid_.cols()-1) = 0.0;
    }
    printMatrix(grid_, "Grid");
    v_ = Eigen::MatrixXf::Zero(gridSize_ + 1, gridSize_);
    u_ = Eigen::MatrixXf::Zero(gridSize_, gridSize_ + 1);
}


// void FluidSim::paintGrid(){
//     float xSpacing = static_cast<float>(screenWidth_) / gridSize_;
//     float ySpacing = static_cast<float>(screenHeight_) / gridSize_;

//     char xSpace_str[64];
//     snprintf(xSpace_str, sizeof(xSpace_str), "xspacing: %.2f\r\n", xSpacing);
//     printf(xSpace_str);
//     char ySpace_str[64];
//     snprintf(ySpace_str, sizeof(ySpace_str), "yspacing: %.2f\r\n", ySpacing);
//     printf(ySpace_str);

//     for (int i = 0; i <= gridSize_; i++) {
//         int x = static_cast<int>(i * xSpacing);
//         int y = static_cast<int>(i * ySpacing);
//         if((x == 0) && (y ==0)){
//             x = GRID_START;
//             y = GRID_START;
//         }

//         // cols
//         Paint_DrawLine(x, GRID_START, x, screenHeight_, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

//         // rows
//         Paint_DrawLine(GRID_START, y, screenWidth_, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

//     }
//     // printf("Start");

//     for (int i = 0; i < v_.rows(); ++i)
//     {
//         for (int j = 0; j < v_.cols(); ++j)
//         {
//             int x = static_cast<int>(j * xSpacing);
//             int y = static_cast<int>(i * ySpacing);

//             if (x == 0) x = GRID_START;
//             if (y == 0) y = GRID_START;

//             char vert_velocity[64];
//             snprintf(vert_velocity, sizeof(vert_velocity), "%.1f", v_(i, j)); 

//             int text_width = strlen(vert_velocity) * TEXT_WIDTH;
//             x = x + (xSpacing / 2) - (text_width / 2);
//             y -= GRID_START;

            
//             // printf("\nPainting Line: \n");
//             Paint_DrawString_EN(x, y, vert_velocity, &Font8, WHITE, RED);
//             // printf("X: %d Y: %d data: %.2f", x, y, v_(i,j));  
//         }
//     }

//     for (int i = 0; i < u_.rows(); ++i)
//     {
//         for (int j = 0; j < u_.cols(); ++j)
//         {
//             int x = static_cast<int>(j * xSpacing);
//             int y = static_cast<int>(i * ySpacing);

//             if (x == 0) x = GRID_START;
//             if (y == 0) y = GRID_START;

//             char horz_velocity[64];
//             snprintf(horz_velocity, sizeof(horz_velocity), "%.1f", u_(i, j)); 

//             int text_width = strlen(horz_velocity) * TEXT_WIDTH;
//             x -= GRID_START;
//             y = y + (ySpacing / 2) - TEXT_HEIGHT;

            
//             // printf("\nPainting Line: \n");
//             Paint_DrawString_EN(x, y, horz_velocity, &Font8, WHITE, BLUE);
//             // printf("X: %d Y: %d data: %.2f", x, y, v_(i,j));  
//         }
//     }
    // printf("Done\n");
    
// }
void FluidSim::simulate(){
    integrate();
   // printMatrix(v_, "Vertical Velocity (v)");
   // printMatrix(u_, "Horozontial Velocity (u)");
    pressure_ = Eigen::MatrixXf::Zero(gridSize_, gridSize_);
    projection();
}

void FluidSim::integrate(){
    for (int i = 2; i < grid_.rows(); i++)
    {
        for (int j = 0; j < grid_.cols(); j++)
        {
            //This might be a sketchy thing to do since grid_(i+1) is undefined at points, runtime error?
          if ((grid_(i-1,j) != 0.0) && (grid_(i,j) != 0.0))
          {
            /* code */
            v_(i, j) += (gravity_) * dt_;
          }
        //   v_(i, j) = i + j;
        }
    }

    // for (int i = 0; i < v_.rows()-1; i++)
    // {
    //     for (int j = 0; j < v_.cols(); j++)
    //     {
    //       v_(i, j) += (gravity_) * dt_;
    //     }
    // }


    // u_(4,1) = 0.5; 
    // u_(5,1) = 0.5; 
    
    
}

void FluidSim::projection(){
    Eigen::MatrixXf divergence = Eigen::MatrixXf::Zero(gridSize_, gridSize_);
   // printMatrix(divergence, "divergence (d)");

    //TODO: make this private, can use it above as well
    float spacing = static_cast<float>(screenHeight_) / gridSize_;
    printMatrix(grid_, "Grid");
    computeDivergence(divergence);
    printMatrix(divergence, "Initial Divergence (d)");
    printMatrix(v_, "Vertical Vel");
    float pressureConstant = 1000 * spacing/dt_;

    for(int n = 0; n < 100; n++){
        //Propogate from bottom to top of the grid
        for (int i = gridSize_-1; i > 0; i--)
        {
            for (int j = 1; j < gridSize_; j++)
            {
                //TODO: Dont forget to change this to uint_8 and Eigen int, dont need float here I believe
                if(grid_(i,j) == 0.0){
                    continue;
                }
                //TODO: Same as above
                uint8_t sx0 = grid_(i,j-1);
                uint8_t sx1 = grid_(i,j+1);
                uint8_t sy0 = grid_(i-1,j);
                uint8_t sy1 = grid_(i+1,j);
                uint8_t s = sx0 + sx1 + sy0 + sy1;
                // printf("Hello");
                // char sx0_str[64];
                // snprintf(sx0_str, sizeof(sx0_str), "sx0: %d\r\n\0", (int)sx0);
                // char sx1_str[10];
                // snprintf(sx1_str, sizeof(sx1_str), "sx1: %d\r\n", (int)sx1);
                // char sy0_str[10];
                // snprintf(sy0_str, sizeof(sy0_str), "sy0: %d\r\n", (int)sy0);
                // char sy1_str[10];
                // snprintf(sy1_str, sizeof(sy1_str), "sy1: %d\r\n", (int)sy1);
                // char s_str[10];
                // snprintf(s_str, sizeof(s_str), "s: %d\r\n", s);


                // printf(sx0_str);
                // printf(sx1_str);
                // printf(sy0_str);
                // printf(sy1_str);
                if(s == 0.0){
                    continue;
                }
                // char ij_str[64];
                // snprintf(ij_str, sizeof(ij_str), "\r\ni: %d, j: %d\r\n", i, j);
                // printf(ij_str);
                
                float upperVelocity = getCellsTopVelocity(i,j);
                float lowerVelocity = getCellsBottomVelocity(i,j);
                float leftVelocity = getCellsLeftVelocity(i,j);
                float rightVelocity = getCellsRightVelocity(i,j);

                float div = (rightVelocity - leftVelocity) + (upperVelocity - lowerVelocity);

                // char vij_str[64];
                // snprintf(vij_str, sizeof(vij_str), "\r\nv(i+1, j): %.4f, v(i,j): %.4f\r\n", v_(i+1,j), v_(i,j));
                // printf(vij_str);

                float p = -div/(float)s;
                p *= 1.9;
                // char p_str[64];
                // printf(s_str);
                // snprintf(p_str, sizeof(p_str), "p: %.4f\r\n", p);
                // printf(p_str);
                pressure_(i, j) += pressureConstant * p;
    
                
                // u_(i,j) += divergence(i,j)/4;
                // u_(i,j+1) -= divergence(i,j)/4;
                // v_(i,j) -= divergence(i,j)/4;
                // v_(i+1,j) += divergence(i,j)/4;

                // u_(i,j) += sx0 * p;
                // u_(i,j+1) -= sx1 * p;
                // v_(i,j) -= sy0 * p;
                // v_(i+1,j) += sy1 * p;
                // float div = (u_(i, j+1) - u_(i,j)) + (v_(i+1,j) - v_(i,j));

                float updatedValue = div * ((float)sx1/s);
                char updatedValue_str[64];
                // snprintf(updatedValue_str, sizeof(updatedValue_str), "\r\nvalue: %.3f\r\n", updatedValue);
                // printf(updatedValue_str);

                u_(i,j) -= ((float)sx0) * p;
                u_(i,j+1) += ((float)sx1) * p; 
                v_(i,j) += ((float)sy0) * p;
                v_(i+1,j) -= ((float)sy1) * p; 
            }
        }

        // printMatrix(divergence, "Divergence (d)");

    }
    
    printMatrix(pressure_, "Pressure (p)");
    computeDivergence(divergence);
    printMatrix(divergence, "Final Divergence (d)");
    printMatrix(u_, "Horozontial (u)");
    printMatrix(v_, "Vertical  (v_)");


   // printMatrix(divergence, "divergence after forcing incompresibility (d)");


}
float FluidSim::getCellsTopVelocity(int i, int j){ return v_(i,j); }
float FluidSim::getCellsBottomVelocity(int i, int j){ return v_(i+1,j); }
float FluidSim::getCellsLeftVelocity(int i, int j){ return u_(i,j); }
float FluidSim::getCellsRightVelocity(int i, int j){ return u_(i,j+1); }

void FluidSim::computeDivergence(Eigen::MatrixXf& divergence){
    for (int i = 0; i < gridSize_; i++)
    {
        for (int j = 0; j < gridSize_; j++)
        {
            divergence(i,j) = -(u_(i, j+1) - u_(i,j)) + (v_(i,j) - v_(i+1,j));
        }
    }
}

void FluidSim::printMatrix(Eigen::MatrixXf& matrix, const char* name){
    printf("%s:\n", name);
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            printf("%.4f\t", matrix(i, j));
        }
        printf("\n");
    }
    printf("\n");
}





