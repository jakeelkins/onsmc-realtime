#include <vector>
#include <cmath>
#include "onsmc_rt/nn.h"
#include "onsmc_rt/onsmc.h"

using namespace std;

ONSMC::ONSMC(unsigned int _input_dim, unsigned int _output_dim, float _dt)
: NN(_input_dim, hidden_dim, _output_dim)
{

    input_dim = _input_dim;
    output_dim = _output_dim;

    dt = _dt;

    // ---- ICs ----

    vector<float> _M_hat (
        output_dim,
        1.0f
    );

    vector<float> _x (
        input_dim,
        0.0f
    );

    vector<float> _sigma_x_prime (
        hidden_dim,
        0.0f
    );

    vector<float> _sigma_x_prime_matrix (
        hidden_dim*hidden_dim,
        0.0f
    );

    vector<float> _V_dot (
        input_dim*hidden_dim,
        0.0f
    );

    vector<float> _s (
        output_dim,
        0.0f
    );

    vector<float> _e (
        output_dim,
        0.0f
    );

    vector<float> _e_dot (
        output_dim,
        0.0f
    );

    vector<float> _sat_s (
        output_dim,
        0.0f
    );

    vector<float> _y_ddot_r (
        output_dim,
        0.0f
    );

    vector<float> _s_delta (
        output_dim,
        0.0f
    );

    vector<float> _s_delta_T (
        output_dim,
        0.0f
    );

    vector<float> _Lambda_e (
        output_dim,
        0.0f
    );

    vector<float> _Lambda_e_dot (
        output_dim,
        0.0f
    );

    vector<float> _sigma_x_s_delta (
        output_dim,
        0.0f
    );

    vector<float> _W_T_sigma_x_prime (
        output_dim*hidden_dim,
        0.0f
    );

    vector<float> _G_x_s_delta_T (
        input_dim*output_dim,
        0.0f
    );

    vector<float> _W_dot (
        hidden_dim*output_dim,
        0.0f
    );

    vector<float> _M_hat_dot (
        output_dim,
        0.0f
    );

    vector<float> _F (
        hidden_dim*hidden_dim,
        0.0f
    );

    vector<float> _G (
        input_dim*input_dim,
        0.0f
    );

    vector<float> _H (
        output_dim,
        H_val
    );

    vector<float> _Lambda (
        output_dim*output_dim,
        0.0f
    );

    // -- add everything as the class members
    M_hat = _M_hat;
    x = _x;
    sigma_x_prime = _sigma_x_prime;
    sigma_x_prime_matrix = _sigma_x_prime_matrix;
    V_dot =  _V_dot;
    s = _s;
    e = _e;
    e_dot = _e_dot;
    sat_s = _sat_s;
    y_ddot_r = _y_ddot_r;
    s_delta = _s_delta;
    s_delta_T = _s_delta_T;
    Lambda_e = _Lambda_e;
    Lambda_e_dot = _Lambda_e_dot;
    sigma_x_s_delta = _sigma_x_s_delta;
    W_T_sigma_x_prime = _W_T_sigma_x_prime;
    G_x_s_delta_T = _G_x_s_delta_T;
    W_dot = _W_dot;
    M_hat_dot = _M_hat_dot;

    F = _F;
    G = _G;
    H = _H;
    Lambda = _Lambda;

    // look I know this is weird but how the heck else do you do it without
    // the crazy initializer lists

    // --- populate F, G, H and Lambda ---

    for (unsigned int i = 0; i<(hidden_dim); ++i){
        for (unsigned int j = 0; j<(hidden_dim); ++j){
            if (i==j){
                F[hidden_dim*i + j] = F_val;
            }
        }
    }

    for (unsigned int i = 0; i<(input_dim); ++i){
        for (unsigned int j = 0; j<(input_dim); ++j){
            if (i==j){
                G[input_dim*i + j] = G_val;
            }
        }
    }

    for (unsigned int i = 0; i<(output_dim); ++i){
        for (unsigned int j = 0; j<(output_dim); ++j){
            if (i==j){
                Lambda[output_dim*i + j] = Lambda_val;
            }
        }
    }

}

// control method
void ONSMC::get_control(float* u, float* y, float* y_dot,
                        float* yd, float* yd_dot, float* yd_ddot){

    // e = yd - y
    elementwise_subtraction(e.data(), yd, y, output_dim);
    // e_dot = yd_dot - y_dot;
    elementwise_subtraction(e_dot.data(), yd_dot, y_dot, output_dim);
    // Lambda_e = Lambda*e
    gemv(Lambda_e.data(), Lambda.data(), e.data(), output_dim, output_dim);
    // s = e_dot + Lambda*e;
    elementwise_addition(s.data(), e_dot.data(), Lambda_e.data(), output_dim);
    // Lambda_e_dot = Lambda*e_dot
    gemv(Lambda_e_dot.data(), Lambda.data(), e_dot.data(), output_dim, output_dim);
    // y_ddot_r = yd_ddot + Lambda*e_dot;
    elementwise_addition(y_ddot_r.data(), yd_ddot, Lambda_e_dot.data(), output_dim);

    // sat(s)
    sat(sat_s.data(), s.data(), phi, output_dim);

    float sign_q = 1.0f;

    if (y[0] < 0){
        sign_q = -1.0f;
    }

    // make state vector
    x[0] = y_ddot_r[0];
    x[1] = s[0] + y_dot[0];
    x[2] = sin(y[0]);
    x[3] = cos(y[0]);
    x[4] = y_dot[0];
    x[5] = sign_q;
    x[6] = 1.0f; // appended 1

    //get f_x
    //printf("forward pass... \n");
    NN.forward(x.data());
    //printf("finished forward pass. \n");

    // control law is u = M_hat*y_ddot_r + NN.y_hat + M_hat*(D + eta)*sat_s;
    // i.e. u = M_hat*(y_ddot_r + (D + eta)*sat_s) + NN.y_hat
    // u <- (D + eta)*sat_s
    for (unsigned int i = 0; i<output_dim; ++i){
        u[i] = (D + eta)*sat_s[i];
    }
    // now add y_ddot_r with (D + eta)*sat_s
    // u <- y_ddot_r + (D + eta)*sat_s, so u <- y_ddot_r + u
    elementwise_addition(u, y_ddot_r.data(), u, output_dim);
    // u <- M_hat*(y_ddot_r + (D + eta)*sat_s), so u <- M_hat*u
    //gemv(u.data(), M_hat.data(), u.data(), output_dim, output_dim);  // non diagonal M case
    elementwise_multiplication(u, M_hat.data(), u, output_dim); // diagonal M case
    // u <- M_hat*(y_ddot_r + (D + eta)*sat_s) + NN.y_hat, so u <- u + NN.y_hat
    elementwise_addition(u, u, NN.y_hat.data(), output_dim);


    // printf("V: \n");
    // print_matrix(NN.V, input_dim, hidden_dim);
    // printf("W: \n");
    // print_matrix(NN.W, hidden_dim, output_dim);

    //printf("t: %f \t e[0]: %f \n", t, e[0]);

    // --- updates ---
    // get sigma prime
    sigmoid_prime(sigma_x_prime.data(), NN.sigma_x.data(), hidden_dim);

    // populate sigma_x_prime_matrix with diagonal
    for (unsigned int i = 0; i<hidden_dim; ++i){
        for (unsigned int j = 0; j<hidden_dim; ++j){
            if (i == j){
                sigma_x_prime_matrix[i*hidden_dim + j] = sigma_x_prime[i];
            }
        }
    }

    //s_delta = s - phi*sat_s;
    for (unsigned int i = 0; i<output_dim; ++i){
        s_delta[i] = s[i] - phi*sat_s[i];
    }

    // W
    // sigma_x*s_delta^T
    //printf("updating... \n");
    transpose(s_delta_T.data(), s_delta.data(), hidden_dim, 1);
    gemm(sigma_x_s_delta.data(), NN.sigma_x.data(), s_delta_T.data(), hidden_dim, 1, output_dim);
    // F * sigma_x*s_delta^T
    gemm(W_dot.data(), F.data(), sigma_x_s_delta.data(), hidden_dim, hidden_dim, output_dim);
    NN.update_W(W_dot.data(), dt);

    // V
    // W^T sigma_x_prime
    gemm(W_T_sigma_x_prime.data(), NN.W_T.data(), sigma_x_prime_matrix.data(), output_dim, hidden_dim, hidden_dim);
    // x s_delta^T ( use G_x_s_delta because same size)
    gemm(G_x_s_delta_T.data(), x.data(), s_delta_T.data(), input_dim, 1, output_dim);
    gemm(G_x_s_delta_T.data(), G.data(), G_x_s_delta_T.data(), input_dim, input_dim, output_dim);
    gemm(V_dot.data(), G_x_s_delta_T.data(), W_T_sigma_x_prime.data(), input_dim, output_dim, hidden_dim);
    NN.update_V(V_dot.data(), dt);

    // H (do diagonal version)
    for (unsigned int i = 0; i<output_dim; ++i){
        M_hat_dot[i] = H[i]*(y_ddot_r[i] + (D + eta)*sat_s[i])*s_delta[i];
        // update
        M_hat[i] += M_hat_dot[i]*dt;
    }


}

// --internal methods--
void ONSMC::sat(float* s_out, float* s_in, float phi, int N){
    // vectorized sat function for SMC
    for (unsigned int i = 0; i<N; ++i){
        float s_in_i = s_in[i];
        if (s_in_i < -phi){
            s_out[i] = -1.0f;
        } else if (s_in_i > phi){
            s_out[i] = 1.0f;
        } else {
            s_out[i] = s_in_i/phi;
        }
    }
}

// void ONSMC::sat(float* s_out, float s_in, float phi){
//     // scalar sat function for SMC
//     if (s_in < -phi){
//         *s_out = -1.0f;
//     } else if (s_in > phi){
//         *s_out = 1.0f;
//     } else {
//         *s_out = s_in/phi;
//     }
// }

void ONSMC::gemv(float* y, float* A, float* x, int N, int M){
    // y = Ax, where A is NxM.
    for (unsigned int row = 0; row < N; ++row){
        float sum_ = 0.0f;
        for (unsigned int col = 0; col < M; ++col){
            sum_ += A[row*M + col]*x[col];
        }
        y[row] = sum_;
    }
}

void ONSMC::gemm(float* C, float* A, float* B, int N, int P, int M){
    // C = AB, where C is NxM, A NxP, B PxM.
    for (unsigned int i = 0; i < N; ++i){
        for (unsigned int j = 0; j < M; ++j){
            float sum_ = 0.0f;
            for (unsigned int k = 0; k < P; ++k){
                sum_ += A[i*P + k]*B[k*M + j];
            }
            C[i*M + j] = sum_;
        }
    }
}

void ONSMC::transpose(float* A_T, float* A, int N, int M){
    // single matrix transpose: A is NxM, A_T is MxN.
    for (unsigned int i = 0; i<N; ++i){
        for (unsigned int j = 0; j<M; ++j){
            A_T[j*N + i] = A[i*M + j];
        }
    }
}

void ONSMC::sigmoid_prime(float* sigma_x_prime, float* sigma_x, int hidden_dim){
    // makes vector of sigmoid prime
    for (unsigned int i = 0; i<hidden_dim; ++i){
        float sgm_x = sigma_x[i];
        sigma_x_prime[i] = sgm_x*(1 - sgm_x);
    }
}

void ONSMC::elementwise_subtraction(float* y, float* a, float* b, int N){
    // y = a - b, all Nx1.
    for (unsigned int i = 0; i<N; ++i){
        y[i] = a[i] - b[i];
    }
}

void ONSMC::elementwise_addition(float* y, float* a, float* b, int N){
    // y = a + b, all Nx1.
    for (unsigned int i = 0; i<N; ++i){
        y[i] = a[i] + b[i];
    }
}

void ONSMC::elementwise_multiplication(float* y, float* a, float* b, int N){
    // y = a * b, all Nx1.
    for (unsigned int i = 0; i<N; ++i){
        y[i] = a[i]*b[i];
    }
}