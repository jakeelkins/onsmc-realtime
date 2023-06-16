#ifndef ONSMC_H
#define ONSMC_H

#include <vector>
#include "nn.h"

using namespace std;

class ONSMC {

    public:

        // ---- hypers ----
        int hidden_dim = 15;
        
        float eta = 6.0f;
        float Lambda_val = 4.0f;
        float D = 0.0f;
        float phi = 0.1f;
        float H_val = 0.005f;

        float F_val = 0.8f;
        float G_val = 0.6f;

        NeuralNetwork NN;

        // ---- calc values ----
        int input_dim;
        int output_dim;
        float dt;

        vector<float> M_hat;
        vector<float> x;
        vector<float> u;
        vector<float> sigma_x_prime;
        vector<float> sigma_x_prime_matrix;
        vector<float> V_dot;
        vector<float> s;
        vector<float> e;
        vector<float> e_dot;
        vector<float> sat_s;
        vector<float> y_ddot_r;
        vector<float> s_delta;
        vector<float> s_delta_T;
        vector<float> Lambda_e;
        vector<float> Lambda_e_dot;
        vector<float> sigma_x_s_delta;
        vector<float> W_T_sigma_x_prime;
        vector<float> G_x_s_delta_T;
        vector<float> W_dot;
        vector<float> M_hat_dot;

        vector<float> F;
        vector<float> G;
        vector<float> H;
        vector<float> Lambda;

        // constructor
        ONSMC(unsigned int _input_dim, unsigned int _output_dim, float _dt);

        // control method
        void get_control(float* u, float* y, float* y_dot,
                        float* yd, float* yd_dot, float* yd_ddot);

        // --internal methods--
        void sat(float* s_out, float* s_in, float phi, int N);
        void gemv(float* y, float* A, float* x, int N, int M);
        void gemm(float* C, float* A, float* B, int N, int P, int M);
        void transpose(float* A_T, float* A, int N, int M);
        void sigmoid_prime(float* sigma_x_prime, float* sigma_x, int hidden_dim);
        void elementwise_subtraction(float* y, float* a, float* b, int N);
        void elementwise_addition(float* y, float* a, float* b, int N);
        void elementwise_multiplication(float* y, float* a, float* b, int N);
};

#endif