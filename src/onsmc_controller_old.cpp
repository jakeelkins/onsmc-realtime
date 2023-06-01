#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <cstdio>
#include <vector>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "onsmc_rt/msg/desired_trajectory.hpp"
#include "onsmc_rt/msg/control.hpp"
#include "onsmc_rt/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class NeuralNetwork {

    public:

        int input_dim;
        int hidden_dim;
        int output_dim;

        // network params
        vector<float> W;
        vector<float> V;
        vector<float> W_T;
        vector<float> V_T;

        // calcs used for backprop
        vector<float> sigma_x;
        vector<float> y_hat;

        // constructor
        NeuralNetwork(unsigned int _input_dim, unsigned int _hidden_dim, unsigned int _output_dim){

            input_dim = _input_dim;
            hidden_dim = _hidden_dim;
            output_dim = _output_dim;

            // first vector of weights
            vector<float> _V (
                input_dim*hidden_dim,
                0.0f
            );

            // second vector of weights
            vector<float> _W (
                hidden_dim*output_dim,
                0.0f
            );

            // transposes
            vector<float> _V_T (
                input_dim*hidden_dim,
                0.0f
            );

            vector<float> _W_T (
                hidden_dim*output_dim,
                0.0f
            );

            V = _V;
            W = _W;

            V_T = _V_T;
            W_T = _W_T;

            // -- call internal random_init method --

            mt19937 rng(5); // I put another RNG inbstance here just because I don't want to pass it to NN

            _random_init(rng, V.data(), input_dim, hidden_dim);
            _random_init(rng, W.data(), hidden_dim, output_dim);

        }

        void forward(float* x){
            // forward pass.
            // x is the input.
            // access the output outside the class by NN.y_hat.

            // -- init the interim calculations --
            vector<float> V_T_x (
                hidden_dim,
                0.0f
            );

            vector<float> _sigma_x (
                hidden_dim,
                0.0f
            );

            vector<float> _y_hat (
                output_dim,
                0.0f
            );

            sigma_x = _sigma_x;
            y_hat = _y_hat;

            // run the forward pass with our reference gemv: (note: x is already a pointer)
            // V^T * x
            _transpose(V_T.data(), V.data(), input_dim, hidden_dim);
            _gemv(V_T_x.data(), V_T.data(), x, hidden_dim, input_dim);

            // sigma_x = sigmoid(V_T_x)
            _sigmoid(sigma_x.data(), V_T_x.data(), hidden_dim);

            // append another 1 to the activation
            sigma_x[0] = 1.0f;

            // W^T * sigmoid(V^T x)
            _transpose(W_T.data(), W.data(), hidden_dim, output_dim);
            _gemv(y_hat.data(), W_T.data(), sigma_x.data(), output_dim, hidden_dim);

        }

        void update_W(float* W_dot, float dt){
            // W = W + dW
            // W is hidden_dim x output_dim
            for (unsigned int i = 0; i < (hidden_dim*output_dim); ++i){
                W[i] = W[i] + W_dot[i]*dt;
            }
        }

        void update_V(float* V_dot, float dt){
            // V = V + dV
            // V is input_dim x hidden_dim
            for (unsigned int i = 0; i < (input_dim*hidden_dim); ++i){
                V[i] = V[i] + V_dot[i]*dt;
            }
        }

        // --internal methods--
        void _random_init(mt19937 &rng, float* W, int N, int M);
        void _gemv(float* y, float* A, float* x, int N, int M);
        void _transpose(float* A_T, float* A, int N, int M);
        void _sigmoid(float* y, float* x, int N);
        void print_matrix(vector<float>& Z, unsigned int rows, unsigned int cols);
};

void print_matrix(vector<float>& Z, unsigned int rows, unsigned int cols);
//void dynamics(float* y_ddot, float* u, float* y, float* y_dot, int output_dim);
//void desired_trajectory(float* yd, float* yd_dot, float* yd_ddot, float t);
void sat(float* s_out, float* s_in, float phi, int N);
void gemv(float* y, float* A, float* x, int N, int M);
void gemm(float* C, float* A, float* B, int N, int P, int M);
void transpose(float* A_T, float* A, int N, int M);
void sigmoid_prime(float* sigma_x_prime, float* sigma_x, int hidden_dim);
void elementwise_subtraction(float* y, float* a, float* b, int N);
void elementwise_addition(float* y, float* a, float* b, int N);
void elementwise_multiplication(float* y, float* a, float* b, int N);

class Controller : public rclcpp::Node {
public:

  // nominal control frequency (s) CURRENT: ensure same as time publisher
  float control_period = 0.02;

  // needed for our timer: ms duration cast
  chrono::duration<long double, std::milli> control_period_ms = control_period*1000ms;

  NeuralNetwork NN;

  Controller()
  :Node("controller"),
  NN(input_dim, hidden_dim, output_dim)
  {

    // put every vector def in the constructor
    vector<float> y_ (
        output_dim,
        0.0f
    );

    y = y_;

    vector<float> y_dot_ (
        output_dim,
        0.0f
    );

    y_dot = y_dot_;

    vector<float> y_ddot_ (
        output_dim,
        0.0f
    );

    y_ddot = y_ddot_;

    vector<float> yd_ (
        output_dim,
        0.0f
    );

    yd = yd_;

    vector<float> yd_dot_ (
        output_dim,
        0.0f
    );

    yd_dot = yd_dot_;

    vector<float> yd_ddot_ (
        output_dim,
        0.0f
    );

    yd_ddot = yd_ddot_;

    vector<float> M_hat_ (
        output_dim,
        M_hat_val
    );

    M_hat = M_hat_;

    // --- populate F, G, H and Lambda ---
    vector<float> F_ (
        hidden_dim*hidden_dim,
        0.0f
    );

    F =  F_;

    for (unsigned int i = 0; i<(hidden_dim); ++i){
        for (unsigned int j = 0; j<(hidden_dim); ++j){
            if (i==j){
                F[hidden_dim*i + j] = F_val;
            }
        }
    }

    vector<float> G_ (
        input_dim*input_dim,
        0.0f
    );

    G = G_;

    for (unsigned int i = 0; i<(input_dim); ++i){
        for (unsigned int j = 0; j<(input_dim); ++j){
            if (i==j){
                G[input_dim*i + j] = G_val;
            }
        }
    }

    vector<float> H_ (
        output_dim,
        H_val
    );

    H = H_;

    vector<float> Lambda_ (
        output_dim*output_dim,
        0.0f
    );

    Lambda = Lambda_;

    for (unsigned int i = 0; i<(output_dim); ++i){
        for (unsigned int j = 0; j<(output_dim); ++j){
            if (i==j){
                Lambda[output_dim*i + j] = Lambda_val;
            }
        }
    }

    // build NN
    //NN = NN_;

    // subscribe to current state x as input
    subscription_x = this->create_subscription<onsmc_rt::msg::State>(
      "state", 10, std::bind(&Controller::state_receive_callback, this, _1));

    // subscribe to topic desired_trajectory as input
    subscription_xd = this->create_subscription<onsmc_rt::msg::DesiredTrajectory>(
      "desired_trajectory", 10, std::bind(&Controller::traj_receive_callback, this, _1));

    // publish to control topic
    publisher_ = this->create_publisher<onsmc_rt::msg::Control>("control", 10);

    // wall timer for publishing control
    timer_ = this->create_wall_timer(
        control_period_ms, std::bind(&Controller::control_callback, this));

    printf("Controller node successfully built. \n");

  }

private:

  rclcpp::Subscription<onsmc_rt::msg::State>::SharedPtr subscription_x;
  rclcpp::Subscription<onsmc_rt::msg::DesiredTrajectory>::SharedPtr subscription_xd;
  rclcpp::Publisher<onsmc_rt::msg::Control>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --  state ICs --
  // match these to the desireds so u is 0 before the experiment runs
  float q = 0.0f;
  float q_dot = 0.0f;

  // -- desired trajectory ICs --
  float qd = 0.0f;
  float qd_dot = 0.0f;
  float qd_ddot = 0.0f;

  float e_int = 0.0f;

  // onsmc stuff

  // ---- hypers ----
  int input_dim = 5;
  int hidden_dim = 15;
  int output_dim = 1;

  float eta = 2.0;
  float Lambda_val = 4.0;
  float D = 0.5;
  float phi = 0.0005;
  float M_hat_val = 2.5;
  float H_val = 0.95;

  float F_val = 50.0;
  float G_val = 30.0;

  //NeuralNetwork NN(input_dim, hidden_dim, output_dim);
  //NeuralNetwork NN;

  vector<float> y;
  vector<float> y_dot;
  vector<float> y_ddot;
  vector<float> yd;
  vector<float> yd_dot;
  vector<float> yd_ddot;
  vector<float> M_hat;

  vector<float> F;
  vector<float> G;
  vector<float> H;
  vector<float> Lambda;

  void state_receive_callback(const onsmc_rt::msg::State &state_msg){

    // -- record current state inputs --
    q = state_msg.q;
    q_dot = state_msg.q_dot;

    y[0] = q;
    y_dot[0] = q_dot;

  }

  void traj_receive_callback(const onsmc_rt::msg::DesiredTrajectory &desired_traj_msg){

    // -- record desired trajectory inputs --
    qd = desired_traj_msg.qd;
    qd_dot = desired_traj_msg.qd_dot;
    qd_ddot = desired_traj_msg.qd_ddot;

    yd[0] = qd;
    yd_dot[0] = qd_dot;
    yd_ddot[0] = qd_ddot;

  }

  // THE MEAT: controller.
  void control_callback(){

    // out msg:
    auto control_msg = onsmc_rt::msg::Control();

    //RCLCPP_INFO(this->get_logger(), "curr q: %f  curr q_dot: %f  curr qd: %f", q, q_dot, qd);

    // variables needed
    vector<float> x (
        input_dim,
        0.0f
    );

    vector<float> u (
        output_dim,
        0.0f
    );

    vector<float> sigma_x_prime (
        hidden_dim,
        0.0f
    );

    vector<float> sigma_x_prime_matrix (
        hidden_dim*hidden_dim,
        0.0f
    );

    vector<float> V_dot (
        input_dim*hidden_dim,
        0.0f
    );

    vector<float> s (
        output_dim,
        0.0f
    );

    vector<float> e (
        output_dim,
        0.0f
    );

    vector<float> e_dot (
        output_dim,
        0.0f
    );

    vector<float> sat_s (
        output_dim,
        0.0f
    );

    vector<float> y_ddot_r (
        output_dim,
        0.0f
    );

    vector<float> s_delta (
        output_dim,
        0.0f
    );

    vector<float> s_delta_T (
        output_dim,
        0.0f
    );

    vector<float> Lambda_e (
        output_dim,
        0.0f
    );

    vector<float> Lambda_e_dot (
        output_dim,
        0.0f
    );

    // -- interim update calcs --
    vector<float> sigma_x_s_delta (
        output_dim,
        0.0f
    );

    vector<float> W_T_sigma_x_prime (
        output_dim*hidden_dim,
        0.0f
    );

    vector<float> G_x_s_delta_T (
        input_dim*output_dim,
        0.0f
    );

    vector<float> W_dot (
        hidden_dim*output_dim,
        0.0f
    );

    vector<float> M_hat_dot (
        output_dim,
        0.0f
    );

    // ---- control calcs ----
    // e = yd - y
    elementwise_subtraction(e.data(), yd.data(), y.data(), output_dim);
    // e_dot = yd_dot - y_dot;
    elementwise_subtraction(e_dot.data(), yd_dot.data(), y_dot.data(), output_dim);
    // Lambda_e = Lambda*e
    gemv(Lambda_e.data(), Lambda.data(), e.data(), output_dim, output_dim);
    // s = e_dot + Lambda*e;
    elementwise_addition(s.data(), e_dot.data(), Lambda_e.data(), output_dim);
    // Lambda_e_dot = Lambda*e_dot
    gemv(Lambda_e_dot.data(), Lambda.data(), e_dot.data(), output_dim, output_dim);
    // y_ddot_r = yd_ddot + Lambda*e_dot;
    elementwise_addition(y_ddot_r.data(), yd_ddot.data(), Lambda_e_dot.data(), output_dim);

    // sat(s)
    sat(sat_s.data(), s.data(), phi, output_dim);

    // make state vector
    x[0] = y_ddot_r[0];
    x[1] = cos(y[0]);
    x[2] = sin(y[0]);
    x[3] = y_dot[0];
    x[4] = 1.0f; // appended 1

    //get f_x
    NN.forward(x.data());

    // control law is u = M_hat*y_ddot_r + NN.y_hat + M_hat*(D + eta)*sat_s;
    // i.e. u = M_hat*(y_ddot_r + (D + eta)*sat_s) + NN.y_hat
    // u <- (D + eta)*sat_s
    for (unsigned int i = 0; i<output_dim; ++i){
        u[i] = (D + eta)*sat_s[i];
    }
    // now add y_ddot_r with (D + eta)*sat_s
    // u <- y_ddot_r + (D + eta)*sat_s, so u <- y_ddot_r + u
    elementwise_addition(u.data(), y_ddot_r.data(), u.data(), output_dim);
    // u <- M_hat*(y_ddot_r + (D + eta)*sat_s), so u <- M_hat*u
    //gemv(u.data(), M_hat.data(), u.data(), output_dim, output_dim);  // non diagonal M case
    elementwise_multiplication(u.data(), M_hat.data(), u.data(), output_dim); // diagonal M case
    // u <- M_hat*(y_ddot_r + (D + eta)*sat_s) + NN.y_hat, so u <- u + NN.y_hat
    elementwise_addition(u.data(), u.data(), NN.y_hat.data(), output_dim);

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
    transpose(s_delta_T.data(), s_delta.data(), hidden_dim, 1);
    gemm(sigma_x_s_delta.data(), NN.sigma_x.data(), s_delta_T.data(), hidden_dim, 1, output_dim);
    // F * sigma_x*s_delta^T
    gemm(W_dot.data(), F.data(), sigma_x_s_delta.data(), hidden_dim, hidden_dim, output_dim);
    NN.update_W(W_dot.data(), control_period);

    // V
    // W^T sigma_x_prime
    gemm(W_T_sigma_x_prime.data(), NN.W_T.data(), sigma_x_prime_matrix.data(), output_dim, hidden_dim, hidden_dim);
    // x s_delta^T ( use G_x_s_delta because same size)
    gemm(G_x_s_delta_T.data(), x.data(), s_delta_T.data(), input_dim, 1, output_dim);
    gemm(G_x_s_delta_T.data(), G.data(), G_x_s_delta_T.data(), input_dim, input_dim, output_dim);
    gemm(V_dot.data(), G_x_s_delta_T.data(), W_T_sigma_x_prime.data(), input_dim, output_dim, hidden_dim);
    NN.update_V(V_dot.data(), control_period);

    // H (do diagonal version)
    for (unsigned int i = 0; i<output_dim; ++i){
        M_hat_dot[i] = H[i]*(y_ddot_r[i] + (D + eta)*sat_s[i])*s_delta[i];
        // update
        M_hat[i] += M_hat_dot[i]*control_period;
    }

    float u_ = u[0];

    // pid ----------------------
    e[0] = qd - q;
    e_dot[0] = qd_dot - q_dot;
    e_int += e[0]*control_period;

    // worked with our usual internal gains
    float Kp = 8.0f;
    float Ki = 0.0f;
    float Kd = 1.5f;
    // float Kp = 10.0f;
    // float Ki = 0.0f;
    // float Kd = 5.0f;

    float int_clip = 0.2;
    float int_term = Ki*e_int;

    if (int_term > int_clip){
      int_term = int_clip;
    } else if (int_term < -int_clip){
      int_term = -int_clip;
    }

    u_ = Kp*e[0] + int_term + Kd*e_dot[0];

    // deadzone inverse
    float dz = 0.0f;  // start small

    if (u_ > 0.0f){
      u_ = u_ + dz;
    } else if (u_ < 0.0f){
      u_ = u_ - dz;
    }

    // clip
    float clip = 10.0f;

    if (u_ > clip){
      u_ = clip;
    } else if (u_ < -clip){
      u_ = -clip;
    }

    

    // ------------------------

    control_msg.u = u_;

    publisher_->publish(control_msg);
    
  }
  
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<Controller>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}




void print_matrix(vector<float>& Z, unsigned int rows, unsigned int cols){
    for (unsigned int row = 0; row < rows; ++row){
        for (unsigned int col = 0; col < cols; ++col){
            printf("%.4f ",Z[row*cols + col]);
        }
        printf("\n");
    }
}

// void dynamics(float* y_ddot, float* u, float* y, float* y_dot, int output_dim){
//     float h = 1.0f;
//     float a_1 = 1.0f;
//     float a_2 = 1.0f;

//     // would loop here if ouput_dim > 0.

//     y_ddot[0] = (u[0] - a_1*(y[0]*y[0]) - a_2*y_dot[0])/h;
// }

// void desired_trajectory(float* yd, float* yd_dot, float* yd_ddot, float t){
//     float y1d = sin(2*t);
//     float y1d_dot = 2*cos(2*t);
//     float y1d_ddot = -4*sin(2*t);

//     yd[0] = y1d;
//     yd_dot[0] = y1d_dot;
//     yd_ddot[0] = y1d_ddot;
// }

void sat(float* s_out, float* s_in, float phi, int N){
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

// void sat(float* s_out, float s_in, float phi){
//     // scalar sat function for SMC
//     if (s_in < -phi){
//         *s_out = -1.0f;
//     } else if (s_in > phi){
//         *s_out = 1.0f;
//     } else {
//         *s_out = s_in/phi;
//     }
// }

void gemv(float* y, float* A, float* x, int N, int M){
    // y = Ax, where A is NxM.
    for (unsigned int row = 0; row < N; ++row){
        float sum_ = 0.0f;
        for (unsigned int col = 0; col < M; ++col){
            sum_ += A[row*M + col]*x[col];
        }
        y[row] = sum_;
    }
}

void gemm(float* C, float* A, float* B, int N, int P, int M){
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


void transpose(float* A_T, float* A, int N, int M){
    // single matrix transpose: A is NxM, A_T is MxN.
    for (unsigned int i = 0; i<N; ++i){
        for (unsigned int j = 0; j<M; ++j){
            A_T[j*N + i] = A[i*M + j];
        }
    }
}

void sigmoid_prime(float* sigma_x_prime, float* sigma_x, int hidden_dim){
    // makes vector of sigmoid prime
    for (unsigned int i = 0; i<hidden_dim; ++i){
        float sgm_x = sigma_x[i];
        sigma_x_prime[i] = sgm_x*(1 - sgm_x);
    }
}

void elementwise_subtraction(float* y, float* a, float* b, int N){
    // y = a - b, all Nx1.
    for (unsigned int i = 0; i<N; ++i){
        y[i] = a[i] - b[i];
    }
}

void elementwise_addition(float* y, float* a, float* b, int N){
    // y = a + b, all Nx1.
    for (unsigned int i = 0; i<N; ++i){
        y[i] = a[i] + b[i];
    }
}

void elementwise_multiplication(float* y, float* a, float* b, int N){
    // y = a * b, all Nx1.
    for (unsigned int i = 0; i<N; ++i){
        y[i] = a[i]*b[i];
    }
}

void NeuralNetwork::_random_init(mt19937 &rng, float* W, int N, int M){
    // samples random values for weights and biases.
    // generalized for a matrix NxM.
    // init like pytorch does, uniform ()
    //float stddev = 1/sqrt((float)N);
    float stddev = 0.01;
    uniform_real_distribution<float> uni(-stddev, stddev);

    for (unsigned int i = 0; i<N; ++i){
        for (unsigned int j = 0; j<M; ++j){
            auto rand_val = uni(rng);
            W[i*M + j] = rand_val;
        }
    }
}

void NeuralNetwork::_gemv(float* y, float* A, float* x, int N, int M){
    // y = Ax, where A is NxM.
    for (unsigned int row = 0; row < N; ++row){
        float sum_ = 0.0f;
        for (unsigned int col = 0; col < M; ++col){
            sum_ += A[row*M + col]*x[col];
        }
        y[row] = sum_;
    }
}

void NeuralNetwork::_transpose(float* A_T, float* A, int N, int M){
    // single matrix transpose: A is NxM, A_T is MxN.
    for (unsigned int i = 0; i<N; ++i){
        for (unsigned int j = 0; j<M; ++j){
            A_T[j*N + i] = A[i*M + j];
        }
    }
}

void NeuralNetwork::_sigmoid(float* y, float* x, int N){
    // elementwise sigmoid activation
    // input x, output y, N is length of x.
    for (unsigned int i = 0; i<N; ++i){
        y[i] = 1/(1 + exp(-x[i]));
    }
}

void NeuralNetwork::print_matrix(vector<float>& Z, unsigned int rows, unsigned int cols){
    for (unsigned int row = 0; row < rows; ++row){
        for (unsigned int col = 0; col < cols; ++col){
            printf("%.4f ",Z[row*cols + col]);
        }
        printf("\n");
    }
}
