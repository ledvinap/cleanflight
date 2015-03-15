
#define B10 1.f
#define B11 2.f
#define B12 1.f

#define A10 1.f
#define A11 -1.57524f
#define A12 0.62633f

#define B20 1.f
#define B21 2.f
#define B22 1.f

#define A20 1.f
#define A21 -1.76883f
#define A22 0.82620f

#define G 1.8322e-04f

float biquad(float x, float* state)
{
    float ct, y1, y2;

    ct = B10 * x  + B11 * state[0] + B12 * state[1];
    y1 = A10 * ct - A11 * state[2] - A12 * state[3];

    state[1] = state[0];
    state[0] = x;

    ct = B20 * y1 + B21 * state[2] + B22 * state[3];
    y2 = A20 * ct - A21 * state[4] - A22 * state[5];

    state[3] = state[2];
    state[2] = y1;

    state[5] = state[4];
    state[4] = y2;

    return y2 * G;
}
