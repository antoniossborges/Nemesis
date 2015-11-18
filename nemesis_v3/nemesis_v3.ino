/*
	PROJETO SEGUIDOR DE LINHA
	Acadêmicos: Fábio M. K. Lenzi, Joe J. Vogel
	Orientador: Manfred H. Junior
	Centro Universitário Católica de Santa Catarina
*/

//Declaração das variáveis utilizadas durante a execução do programa
float Kp = 500, Ki = 0.003, Kd = 12;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[5] = {0, 0, 0, 0, 0};
int initial_motor_speed = 160;
int motor_speed = 160;
unsigned long tempBefore = 0; 
int speedInZigZag = 110;
int speedOffZigZag = 160;
const int motorA = 2; //Motor Esquerda
const int motorB = 4; //Motor Direita
const int motorA_PWM = 3;
const int motorB_PWM = 5;

int line;
int outline;

const int ss1 = 8;  // sensor 1
const int ss2 = 9;  // sensor 2
const int ss3 = 10; // sensor 3
const int ss4 = 11; // sensor 4
const int ss5 = 12; // sensor 5
//const int ss6 = 13; // sensor Near

//Declaração das funções
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

//Função de Setup do Arduino
void setup()
{
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);

  pinMode(ss1, INPUT);
  pinMode(ss2, INPUT);
  pinMode(ss3, INPUT);
  pinMode(ss4, INPUT);
  pinMode(ss5, INPUT);
  //pinMode(ss6, INPUT);

  delay(500);
  tempBefore = millis();
}

//Função de execução continua do Arduino
void loop()
{
/*
if(millis() - tempBefore > 25730){

  stopMotors();
  delay(10000);

}else if((millis() - tempBefore) > 1873 && (millis() - tempBefore) < 4600){

  initial_motor_speed = speedInZigZag;
  motor_speed = speedInZigZag;

}else {

  initial_motor_speed = speedOffZigZag;
  motor_speed = speedOffZigZag;   

}*/

  read_sensor_values();
  calculate_pid();
  motor_control();
}

//Função responsável por fazer a leitura dos sensores do Robô
void read_sensor_values()
{
  line = 0;
  outline = 1;

  sensor[0] = digitalRead(ss1);
  sensor[1] = digitalRead(ss2);
  sensor[2] = digitalRead(ss3);
  sensor[3] = digitalRead(ss4);
  sensor[4] = digitalRead(ss5);

  if ((sensor[0] == outline) && (sensor[1] == outline) && (sensor[2] == outline) && (sensor[3] == outline) && (sensor[4] == line)) // 0 0 0 0 1 --> 4
     error = 4;
    //error = 3;
  else if ((sensor[0] == line) && (sensor[1] == outline) && (sensor[2] == outline) && (sensor[4] == outline) && (sensor[4] == outline)) // 1 0 0 0 0 --> -4
     error = -4;
    //error = -3;
  else if ((sensor[0] == outline) && (sensor[1] == outline) && (sensor[2] == outline) && (sensor[3] == line) && (sensor[4] == line)) // 0 0 0 1 1 --> 3
    error = 3;
    //error = 2;
  else if ((sensor[0] == line) && (sensor[1] == line) && (sensor[2] == outline) && (sensor[3] == outline) && (sensor[4] == outline)) // 1 1 0 0 0 --> -3
    error = -3;
    //error = -2;
  else if ((sensor[0] == outline) && (sensor[1] == outline) && (sensor[2] == outline) && (sensor[3] == line) && (sensor[4] == outline)) // 0 0 0 1 0 --> 2
     error=2;
    //error = 1;
  else if ((sensor[0] == outline) && (sensor[1] == line) && (sensor[2] == outline) && (sensor[3] == outline) && (sensor[4] == outline)) // 0 1 0 0 0 --> -2
   error=-2;
   //error = -1;
  else if ((sensor[0] == outline) && (sensor[1] == outline) && (sensor[2] == line) && (sensor[3] == line) && (sensor[4] == outline)) // 0 0 1 1 0 --> 1
    //error=1;
    error = 0.01;
  else if ((sensor[0] == outline) && (sensor[1] == line) && (sensor[2] == line) && (sensor[3] == outline) && (sensor[4] == outline)) // 0 1 1 0 0 --> -1
    //error=-1;
    error = -0.01;
  else if ((sensor[1] == outline) && (sensor[2] == line) && (sensor[3] == outline))                                   // 0 0 1 0 0 --> 0
    error = 0;
}

//Função responsável por fazer o cáculo PID
void calculate_pid()
{

  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;

}

//Função responsável por controlar a velocidade de cada motor
void motor_control()
{
  int left_motor_speed;
  int right_motor_speed;


  // Calculo da velocidade de cada motor:
  left_motor_speed = initial_motor_speed + PID_value;
  right_motor_speed = initial_motor_speed - PID_value;

  if (left_motor_speed < 0) {
    left_motor_speed = 0;
  } else if (left_motor_speed > motor_speed) {
    left_motor_speed = motor_speed;
  }

  if (right_motor_speed < 0) {
    right_motor_speed = 0;
  } else if (right_motor_speed > motor_speed) {
    right_motor_speed = motor_speed;
  }

  analogWrite(motorA_PWM, left_motor_speed);
  analogWrite(motorB_PWM, right_motor_speed * 1.1); //*1.17 devido a diferença entre motores

  digitalWrite(motorA, LOW);
  digitalWrite(motorB, LOW);
}

//Função responsável por parar o Robô
void stopMotors() {
  analogWrite(motorA_PWM, 0);
  analogWrite(motorB_PWM, 0);
}
