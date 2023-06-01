#ifndef NN_H
#define NN_H

#include <random>
#include <vector>
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
        NeuralNetwork(unsigned int _input_dim, unsigned int _hidden_dim, unsigned int _output_dim);

        void forward(float* x);

        void update_W(float* W_dot, float dt);

        void update_V(float* V_dot, float dt);

        // --internal methods--
        void _random_init(mt19937 &rng, float* W, int N, int M);
        void _gemv(float* y, float* A, float* x, int N, int M);
        void _transpose(float* A_T, float* A, int N, int M);
        void _sigmoid(float* y, float* x, int N);
        void print_matrix(vector<float>& Z, unsigned int rows, unsigned int cols);
};

#endif