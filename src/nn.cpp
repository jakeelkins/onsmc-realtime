#include <random>
#include <vector>
#include "onsmc_rt/nn.h"

using namespace std;

// constructor
NeuralNetwork::NeuralNetwork(unsigned int _input_dim, unsigned int _hidden_dim, unsigned int _output_dim){

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

void NeuralNetwork::forward(float* x){
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

void NeuralNetwork::update_W(float* W_dot, float dt){
    // W = W + dW
    // W is hidden_dim x output_dim
    for (unsigned int i = 0; i < (hidden_dim*output_dim); ++i){
        W[i] = W[i] + W_dot[i]*dt;
    }
}

void NeuralNetwork::update_V(float* V_dot, float dt){
    // V = V + dV
    // V is input_dim x hidden_dim
    for (unsigned int i = 0; i < (input_dim*hidden_dim); ++i){
        V[i] = V[i] + V_dot[i]*dt;
    }
}

void NeuralNetwork::_random_init(mt19937 &rng, float* W, int N, int M){
    // samples random values for weights and biases.
    // generalized for a matrix NxM.
    // init like pytorch does, uniform ()
    //float stddev = 1/sqrt((float)N);
    float stddev = 0.01f;
    //float stddev = 0.0f;
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