//
// Created by aM on 25-6-8.
//

#ifndef MOTOR_H
#define MOTOR_H

void Motor_SetLeftCompare(uint16_t LeftCompare);
int16_t Encoder_GetLeft(void);
int16_t Encoder_GetRight(void);
void Motor_SetLeftPWM(int16_t LeftPWM);
void Motor_SetRightPWM(int16_t RightPWM);
#endif //MOTOR_H
