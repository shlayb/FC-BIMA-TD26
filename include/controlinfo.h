// //pure example codes
// #include <FW_controller.h>
// #include <FW_config.h>
// #include <FW_roll_controller.h>
// #include <Fuzzy_FW_Roll.h>
// #include <math.h>

// // Define the state-space model for the fixed wing UAV
// typedef struct {
//     double A[2][2];
//     double B[2][1];
//     double C[1][2];
//     double D[1][1];
// } UAVModel;

// // Define the LQR controller
// typedef struct {
//     double K[1][2];
// } LQRController;

// // Define the fuzzy controller
// typedef struct {
//     double Kp;
//     double Ki;
//     double Kd;
// } FuzzyController;

// // Function to calculate the step response of the system
// void calculateStepResponse(UAVModel *model, LQRController *controller, FuzzyController *fuzzyController, double *output, double *error, double *prevError, double *riseTime, double *settlingTime, double *overshoot) {
//     // Calculate the control input using the LQR or fuzzy controller
//     double input;
//     if (fuzzyController->Kp != 0) { // use fuzzy controller
//         double de = (*error - *prevError) / 0.01; // calculate derivative of error
//         input = fuzzyController->Kp * *error + fuzzyController->Ki * *error * 0.01 + fuzzyController->Kd * de;
//     } else { // use LQR controller
//         input = -controller->K[0][0] * output[0] - controller->K[0][1] * output[1];
//     }

//     // Calculate the next state of the system
//     double nextState[2];
//     nextState[0] = model->A[0][0] * output[0] + model->A[0][1] * output[1] + model->B[0][0] * input;
//     nextState[1] = model->A[1][0] * output[0] + model->A[1][1] * output[1] + model->B[1][0] * input;

//     // Calculate the error and previous error
//     *error = 1.0 - nextState[0]; // assume reference signal is 1.0
//     *prevError = *error;

//     // Calculate the rise time, settling time, and overshoot
//     static double prevTime = 0;
//     double time = // current time;
//     if (time - prevTime < 0.01) { // check for small time steps
//         return;
//     }
//     prevTime = time;
//     if (*error > 0 && *riseTime == 0) {
//         *riseTime = time;
//     }
//     if (fabs(*error) < 0.01 && *settlingTime == 0) {
//         *settlingTime = time;
//         if (*overshoot < nextState[0]) {
//             *overshoot = nextState[0];
//         }
//     }

//     // Store the output
//     output[0] = nextState[0];
//     output[1] = nextState[1];
// }

// // Function to display the rise time, settling time, and overshoot
// void displayResults(double riseTime, double settlingTime, double overshoot) {
//     printf("Rise time: %.2f s\n", riseTime);
//     printf("Settling time: %.2f s\n", settlingTime);
//     printf("Overshoot: %.2f %%\n", (overshoot - 1.0) * 100);
// }

// int main() {
//     // Initialize the UAV model, LQR controller, fuzzy controller, and variables
//     UAVModel model = {{{0, 1}, {0, 0}}, {{0}, {1}}, {{1, 0}}, {{0}}};
//     LQRController lqrController = {{{1, 0}}};
//     FuzzyController fuzzyController = {1.0, 0.1, 0.5};
//     double output[2] = {0};
//     double error = 0;
//     double prevError = 0;
//     double riseTime = 0;
//     double settlingTime = 0;
//     double overshoot = 0;

//     // Calculate the step response and display the results
//     for (double time = 0; time < 10; time += 0.01) {
//         calculateStepResponse(&model, &lqrController, &fuzzyController, output, &error, &prevError, &riseTime, &settlingTime, &overshoot);
//         if (time > 1 && time - prevTime > 0.1) { // wait for the response to settle before displaying the results
//             displayResults(riseTime, settlingTime, overshoot);
//             break;
//         }
//     }

//     return